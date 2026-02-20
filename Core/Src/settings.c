#include "settings.h"

#include "pid.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash_ex.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern tpid pidMotor1Speed;
extern tpid pidMotor2Speed;
extern tpid pid_pidHW_Tracking;
extern float g_line_base_speed;
extern float g_line_search_speed;
extern float g_line_max_speed;
extern float g_line_min_speed;

#define SETTINGS_MAGIC            0x53455453UL /* 'SETS' */
#define SETTINGS_VERSION          1U
#define SETTINGS_COMMIT_MAGIC     0xA55AA55AUL
#define SETTINGS_SAVE_DELAY_MS    2000U

typedef struct
{
  float m1_kp;
  float m1_ki;
  float m1_kd;
  float m2_kp;
  float m2_ki;
  float m2_kd;
  float tr_kp;
  float tr_ki;
  float tr_kd;
  float line_base;
  float line_search;
  float line_max;
  float line_min;
} SettingsPayload;

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t payload_len;
  uint32_t seq;
  SettingsPayload payload;
  uint32_t crc32;
  uint32_t commit;
} SettingsRecord;

_Static_assert((sizeof(SettingsRecord) % 2U) == 0U, "SettingsRecord must be halfword aligned");

static uint8_t s_dirty = 0U;
static uint32_t s_dirty_tick = 0U;
static uint8_t s_has_active = 0U;
static uint32_t s_active_page = 0U;
static uint32_t s_active_seq = 0U;

static float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static uint32_t settings_crc32(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  size_t i;
  uint8_t bit;

  for (i = 0; i < len; i++)
  {
    crc ^= (uint32_t)data[i];
    for (bit = 0; bit < 8U; bit++)
    {
      if ((crc & 1U) != 0U)
      {
        crc = (crc >> 1U) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return ~crc;
}

static uint32_t flash_size_bytes(void)
{
  uint32_t flash_kb = (uint32_t)(*((uint16_t *)FLASH_SIZE_DATA_REGISTER));
  return flash_kb * 1024U;
}

static uint32_t settings_page_a_addr(void)
{
  uint32_t flash_end = FLASH_BASE + flash_size_bytes();
  return flash_end - (2U * FLASH_PAGE_SIZE);
}

static uint32_t settings_page_b_addr(void)
{
  uint32_t flash_end = FLASH_BASE + flash_size_bytes();
  return flash_end - FLASH_PAGE_SIZE;
}

static uint8_t settings_record_is_valid(const SettingsRecord *rec)
{
  uint32_t crc_calc;

  if (rec == NULL) return 0U;
  if (rec->magic != SETTINGS_MAGIC) return 0U;
  if (rec->version != SETTINGS_VERSION) return 0U;
  if (rec->payload_len != sizeof(SettingsPayload)) return 0U;
  if (rec->commit != SETTINGS_COMMIT_MAGIC) return 0U;

  crc_calc = settings_crc32((const uint8_t *)rec, offsetof(SettingsRecord, crc32));
  if (crc_calc != rec->crc32) return 0U;

  return 1U;
}

static void settings_capture_payload(SettingsPayload *p)
{
  if (p == NULL) return;

  p->m1_kp = pidMotor1Speed.kp;
  p->m1_ki = pidMotor1Speed.ki;
  p->m1_kd = pidMotor1Speed.kd;

  p->m2_kp = pidMotor2Speed.kp;
  p->m2_ki = pidMotor2Speed.ki;
  p->m2_kd = pidMotor2Speed.kd;

  p->tr_kp = pid_pidHW_Tracking.kp;
  p->tr_ki = pid_pidHW_Tracking.ki;
  p->tr_kd = pid_pidHW_Tracking.kd;

  p->line_base = g_line_base_speed;
  p->line_search = g_line_search_speed;
  p->line_max = g_line_max_speed;
  p->line_min = g_line_min_speed;
}

static void settings_apply_payload(SettingsPayload *p)
{
  if (p == NULL) return;

  p->m1_kp = clampf(p->m1_kp, -50.0f, 50.0f);
  p->m1_ki = clampf(p->m1_ki, -10.0f, 10.0f);
  p->m1_kd = clampf(p->m1_kd, -50.0f, 50.0f);

  p->m2_kp = clampf(p->m2_kp, -50.0f, 50.0f);
  p->m2_ki = clampf(p->m2_ki, -10.0f, 10.0f);
  p->m2_kd = clampf(p->m2_kd, -50.0f, 50.0f);

  p->tr_kp = clampf(p->tr_kp, -50.0f, 50.0f);
  p->tr_ki = clampf(p->tr_ki, -10.0f, 10.0f);
  p->tr_kd = clampf(p->tr_kd, -50.0f, 50.0f);

  p->line_base = clampf(p->line_base, 0.0f, 8.0f);
  p->line_search = clampf(p->line_search, 0.0f, 8.0f);
  p->line_max = clampf(p->line_max, 0.0f, 8.0f);
  p->line_min = clampf(p->line_min, 0.0f, 8.0f);

  if (p->line_min > p->line_max)
  {
    p->line_min = 0.5f;
    p->line_max = 4.0f;
  }

  pidMotor1Speed.kp = p->m1_kp;
  pidMotor1Speed.ki = p->m1_ki;
  pidMotor1Speed.kd = p->m1_kd;

  pidMotor2Speed.kp = p->m2_kp;
  pidMotor2Speed.ki = p->m2_ki;
  pidMotor2Speed.kd = p->m2_kd;

  pid_pidHW_Tracking.kp = p->tr_kp;
  pid_pidHW_Tracking.ki = p->tr_ki;
  pid_pidHW_Tracking.kd = p->tr_kd;

  g_line_base_speed = p->line_base;
  g_line_search_speed = p->line_search;
  g_line_max_speed = p->line_max;
  g_line_min_speed = p->line_min;
}

static uint8_t settings_read_best_record(SettingsRecord *best, uint32_t *best_page)
{
  const SettingsRecord *rec_a = (const SettingsRecord *)settings_page_a_addr();
  const SettingsRecord *rec_b = (const SettingsRecord *)settings_page_b_addr();
  uint8_t a_ok = settings_record_is_valid(rec_a);
  uint8_t b_ok = settings_record_is_valid(rec_b);

  if (!a_ok && !b_ok)
  {
    return 0U;
  }

  if (a_ok && (!b_ok || (rec_a->seq >= rec_b->seq)))
  {
    if (best != NULL) memcpy(best, rec_a, sizeof(SettingsRecord));
    if (best_page != NULL) *best_page = settings_page_a_addr();
    return 1U;
  }

  if (best != NULL) memcpy(best, rec_b, sizeof(SettingsRecord));
  if (best_page != NULL) *best_page = settings_page_b_addr();
  return 1U;
}

static uint8_t settings_flash_write_record(uint32_t target_page, const SettingsRecord *rec)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0U;
  HAL_StatusTypeDef status;
  const uint16_t *src16;
  uint32_t addr;
  uint32_t i;
  uint32_t words_before_commit;

  if (rec == NULL) return 0U;
  if (sizeof(SettingsRecord) > FLASH_PAGE_SIZE) return 0U;

  status = HAL_FLASH_Unlock();
  if (status != HAL_OK) return 0U;

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = target_page;
  erase.NbPages = 1U;
  status = HAL_FLASHEx_Erase(&erase, &page_error);
  if (status != HAL_OK)
  {
    HAL_FLASH_Lock();
    return 0U;
  }

  src16 = (const uint16_t *)rec;
  addr = target_page;
  words_before_commit = offsetof(SettingsRecord, commit) / 2U;

  for (i = 0U; i < words_before_commit; i++)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, src16[i]);
    if (status != HAL_OK)
    {
      HAL_FLASH_Lock();
      return 0U;
    }
    addr += 2U;
  }

  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, target_page + offsetof(SettingsRecord, commit), (uint16_t)(SETTINGS_COMMIT_MAGIC & 0xFFFFU));
  if (status != HAL_OK)
  {
    HAL_FLASH_Lock();
    return 0U;
  }

  status = HAL_FLASH_Program(
    FLASH_TYPEPROGRAM_HALFWORD,
    target_page + offsetof(SettingsRecord, commit) + 2U,
    (uint16_t)((SETTINGS_COMMIT_MAGIC >> 16U) & 0xFFFFU)
  );
  if (status != HAL_OK)
  {
    HAL_FLASH_Lock();
    return 0U;
  }

  HAL_FLASH_Lock();
  return 1U;
}

bool settings_save_now(void)
{
  SettingsRecord rec;
  uint32_t target_page;

  rec.magic = SETTINGS_MAGIC;
  rec.version = SETTINGS_VERSION;
  rec.payload_len = sizeof(SettingsPayload);
  rec.seq = s_has_active ? (s_active_seq + 1U) : 1U;
  settings_capture_payload(&rec.payload);
  rec.crc32 = settings_crc32((const uint8_t *)&rec, offsetof(SettingsRecord, crc32));
  rec.commit = SETTINGS_COMMIT_MAGIC;

  if (s_has_active)
  {
    target_page = (s_active_page == settings_page_a_addr()) ? settings_page_b_addr() : settings_page_a_addr();
  }
  else
  {
    target_page = settings_page_a_addr();
  }

  if (!settings_flash_write_record(target_page, &rec))
  {
    return false;
  }

  s_active_page = target_page;
  s_active_seq = rec.seq;
  s_has_active = 1U;
  return true;
}

void settings_load(void)
{
  SettingsRecord rec;
  uint32_t page;

  if (!settings_read_best_record(&rec, &page))
  {
    s_has_active = 0U;
    s_active_page = 0U;
    s_active_seq = 0U;
    return;
  }

  settings_apply_payload(&rec.payload);
  s_has_active = 1U;
  s_active_page = page;
  s_active_seq = rec.seq;
}

void settings_mark_dirty(void)
{
  s_dirty = 1U;
  s_dirty_tick = HAL_GetTick();
}

void settings_service(void)
{
  uint32_t now;

  if (s_dirty == 0U) return;

  now = HAL_GetTick();
  if ((uint32_t)(now - s_dirty_tick) < SETTINGS_SAVE_DELAY_MS)
  {
    return;
  }

  if (settings_save_now())
  {
    s_dirty = 0U;
  }
  else
  {
    s_dirty_tick = now;
  }
}
