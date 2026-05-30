#include "comm_uart_dma.h"

#include <string.h>

#include "usart.h"

#define COMM_UART_DMA_RX_DMA_SIZE 128U
#define COMM_UART_DMA_RX_RING_SIZE 256U
#define COMM_UART_DMA_TX_SIZE 192U

static uint8_t g_rx_dma[COMM_UART_DMA_RX_DMA_SIZE];
static volatile uint16_t g_rx_dma_pos = 0U;
static uint8_t g_rx_ring[COMM_UART_DMA_RX_RING_SIZE];
static volatile uint16_t g_rx_head = 0U;
static volatile uint16_t g_rx_tail = 0U;
static uint8_t g_tx_buf[COMM_UART_DMA_TX_SIZE];
static volatile uint8_t g_tx_busy = 0U;
static volatile uint8_t g_rx_restart_requested = 0U;

static uint16_t CommUartDma_NextRingIndex(uint16_t index)
{
  index++;
  if (index >= COMM_UART_DMA_RX_RING_SIZE)
  {
    index = 0U;
  }
  return index;
}

static void CommUartDma_PushRxByte(uint8_t byte)
{
  uint16_t next = CommUartDma_NextRingIndex(g_rx_head);
  if (next == g_rx_tail)
  {
    return;
  }
  g_rx_ring[g_rx_head] = byte;
  g_rx_head = next;
}

static uint8_t CommUartDma_StartRx(void)
{
  HAL_StatusTypeDef status;

  g_rx_dma_pos = 0U;
  status = HAL_UART_Receive_DMA(&huart2, g_rx_dma, COMM_UART_DMA_RX_DMA_SIZE);
  if (status == HAL_OK)
  {
    if (huart2.hdmarx != NULL)
    {
      __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }
    g_rx_restart_requested = 0U;
    return 1U;
  }

  return 0U;
}

uint8_t CommUartDma_Init(void)
{
  g_rx_head = 0U;
  g_rx_tail = 0U;
  g_tx_busy = 0U;
  g_rx_restart_requested = 0U;
  return CommUartDma_StartRx();
}

void CommUartDma_PollRx(void)
{
  uint16_t pos;

  if (g_rx_restart_requested != 0U)
  {
    (void)HAL_UART_AbortReceive(&huart2);
    (void)CommUartDma_StartRx();
  }

  if (huart2.hdmarx == NULL)
  {
    return;
  }

  pos = (uint16_t)(COMM_UART_DMA_RX_DMA_SIZE -
                   (uint16_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx));
  if (pos >= COMM_UART_DMA_RX_DMA_SIZE)
  {
    pos = 0U;
  }

  while (g_rx_dma_pos != pos)
  {
    CommUartDma_PushRxByte(g_rx_dma[g_rx_dma_pos]);
    g_rx_dma_pos++;
    if (g_rx_dma_pos >= COMM_UART_DMA_RX_DMA_SIZE)
    {
      g_rx_dma_pos = 0U;
    }
  }
}

uint8_t CommUartDma_GetByte(uint8_t *byte)
{
  if ((byte == NULL) || (g_rx_tail == g_rx_head))
  {
    return 0U;
  }

  *byte = g_rx_ring[g_rx_tail];
  g_rx_tail = CommUartDma_NextRingIndex(g_rx_tail);
  return 1U;
}

uint8_t CommUartDma_Transmit(const char *data, size_t len)
{
  HAL_StatusTypeDef status;

  if ((data == NULL) || (len == 0U))
  {
    return 1U;
  }

  if (len > COMM_UART_DMA_TX_SIZE)
  {
    return 0U;
  }

  __disable_irq();
  if (g_tx_busy != 0U)
  {
    __enable_irq();
    return 0U;
  }
  g_tx_busy = 1U;
  __enable_irq();

  (void)memcpy(g_tx_buf, data, len);
  status = HAL_UART_Transmit_DMA(&huart2, g_tx_buf, (uint16_t)len);
  if (status != HAL_OK)
  {
    g_tx_busy = 0U;
    return 0U;
  }

  return 1U;
}

uint8_t CommUartDma_IsTxBusy(void)
{
  return g_tx_busy;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    g_tx_busy = 0U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    g_tx_busy = 0U;
    g_rx_restart_requested = 1U;
  }
}
