#include "touch_cal_storage.h"

#include "stm32f1xx_hal.h"

#define TOUCH_CAL_FLASH_ADDR 0x0800FC00U
#define TOUCH_CAL_MAGIC      0x54504341UL
#define TOUCH_CAL_VERSION    0x00000002UL

typedef struct
{
  uint32_t magic;
  uint32_t version;
  TouchCalibration_t cal;
  uint32_t checksum;
} TouchCalBlob_t;

static uint32_t CalcChecksum(const TouchCalBlob_t *b)
{
  const uint8_t *p = (const uint8_t *)b;
  uint32_t sum = 0U;
  for (uint32_t i = 0U; i < (sizeof(TouchCalBlob_t) - sizeof(uint32_t)); ++i)
  {
    sum = (sum << 5) - sum + p[i];
  }
  return sum;
}

uint8_t TouchCalStorage_Load(TouchCalibration_t *cal)
{
  if (cal == NULL)
  {
    return 0U;
  }

  const TouchCalBlob_t *b = (const TouchCalBlob_t *)TOUCH_CAL_FLASH_ADDR;
  if ((b->magic != TOUCH_CAL_MAGIC) || (b->version != TOUCH_CAL_VERSION))
  {
    return 0U;
  }

  if (b->checksum != CalcChecksum(b))
  {
    return 0U;
  }

  *cal = b->cal;
  return 1U;
}

uint8_t TouchCalStorage_Save(const TouchCalibration_t *cal)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_err = 0U;
  TouchCalBlob_t blob = {0};
  const uint16_t *data16 = NULL;

  if (cal == NULL)
  {
    return 0U;
  }

  blob.magic = TOUCH_CAL_MAGIC;
  blob.version = TOUCH_CAL_VERSION;
  blob.cal = *cal;
  blob.checksum = CalcChecksum(&blob);
  data16 = (const uint16_t *)&blob;

  HAL_FLASH_Unlock();

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = TOUCH_CAL_FLASH_ADDR;
  erase.NbPages = 1U;
  if (HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return 0U;
  }

  for (uint32_t i = 0U; i < (sizeof(TouchCalBlob_t) / 2U); ++i)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, TOUCH_CAL_FLASH_ADDR + (i * 2U), data16[i]) != HAL_OK)
    {
      HAL_FLASH_Lock();
      return 0U;
    }
  }

  HAL_FLASH_Lock();

  {
    TouchCalibration_t verify = {0};
    if (TouchCalStorage_Load(&verify) == 0U)
    {
      return 0U;
    }
  }

  return 1U;
}
