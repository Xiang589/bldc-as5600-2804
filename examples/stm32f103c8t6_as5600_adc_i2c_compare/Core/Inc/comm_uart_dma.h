#ifndef COMM_UART_DMA_H
#define COMM_UART_DMA_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t CommUartDma_Init(void);
void CommUartDma_PollRx(void);
uint8_t CommUartDma_GetByte(uint8_t *byte);
uint8_t CommUartDma_Transmit(const char *data, size_t len);
uint8_t CommUartDma_IsTxBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* COMM_UART_DMA_H */
