#include "settings.h"

#include "pid.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash_ex.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*
 * 参数持久化模块（Flash）设计目标：
 * 1) 只保存可调参数，不保存运行时瞬态状态；
 * 2) 双页轮换 + CRC + commit 标志，降低断电写坏风险；
 * 3) UI 调参后延迟保存，减少擦写次数与卡顿。
 *
 * 典型调用链：
 * - 上电：settings_load()
 * - 参数改动：settings_mark_dirty()
 * - 后台任务周期调用：settings_service()
 */

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

/*
 * 真正需要保存到 Flash 的业务参数。
 * 这里只放“可调参数”，不放运行时状态（积分项、瞬时速度等），
 * 避免把无意义且变化很快的数据写进 Flash，缩短寿命。
 */
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
  /* 记录头：用于识别、版本兼容和选择最新记录。 */
  uint32_t magic;
  uint16_t version;
  uint16_t payload_len;
  uint32_t seq;
  SettingsPayload payload;
  /* crc32 覆盖从开头到 crc32 前一字节，用于校验整条记录是否完整。 */
  uint32_t crc32;
  /* commit 单独作为“提交标志”，并且在写入流程里最后写。 */
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
  /* 软件 CRC32（小端常见实现，使用反射多项式 0xEDB88320）。 */
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
  /* STM32F1 的 Flash 容量寄存器返回的是 KB，需要换算成字节。 */
  uint32_t flash_kb = (uint32_t)(*((uint16_t *)FLASH_SIZE_DATA_REGISTER));
  return flash_kb * 1024U;
}

static uint32_t settings_page_a_addr(void)
{
  /*
   * 使用 Flash 最后两页作为参数双缓冲区（A/B）。
   * 工程维护时要注意：链接脚本和固件体积不能覆盖这两页。
   */
  /* 使用 Flash 最后一页的前一页作为 A 区，尽量不占主程序常用区域。 */
  uint32_t flash_end = FLASH_BASE + flash_size_bytes();
  return flash_end - (2U * FLASH_PAGE_SIZE);
}

static uint32_t settings_page_b_addr(void)
{
  /* 使用 Flash 最后一页作为 B 区，和 A 区轮换写入。 */
  uint32_t flash_end = FLASH_BASE + flash_size_bytes();
  return flash_end - FLASH_PAGE_SIZE;
}

static uint8_t settings_record_is_valid(const SettingsRecord *rec)
{
  uint32_t crc_calc;

  /*
   * 有效记录判断顺序：
   * 1) 先检查魔数/版本/长度/提交标志（快）
   * 2) 再算 CRC（慢一点）
   * 这样可以减少不必要的 CRC 计算。
   */
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

  /* 把当前运行中的参数快照出来，供写 Flash 使用。 */
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

  /*
   * 先做边界约束，再写回全局参数。
   * 目的是防止旧版本数据/意外写坏数据把系统带到危险参数区间。
   */
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
    /* 下限大于上限说明数据不合理，回退到一组安全默认值。 */
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

  /*
   * 双页轮换策略：
   * - 两页都无效：认为没有保存过；
   * - 两页都有效：按 seq 选更新的那页。
   * 这样即使写入过程中断电，也通常还能保留上一份有效记录。
   */
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

  /*
   * 写入策略的关键点：
   * 1) 先擦页；
   * 2) 先写除 commit 外的所有内容；
   * 3) 最后写 commit 魔数。
   * 这样断电时记录大概率会因为 commit 不完整而判定无效，不会误用半条数据。
   */
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
    /* STM32F1 Flash 的最小编程粒度为半字（16bit）。 */
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

  /* 生成一条完整记录，再写到“另一页”，避免覆盖当前有效页。 */
  rec.magic = SETTINGS_MAGIC;
  rec.version = SETTINGS_VERSION;
  rec.payload_len = sizeof(SettingsPayload);
  rec.seq = s_has_active ? (s_active_seq + 1U) : 1U;
  settings_capture_payload(&rec.payload);
  rec.crc32 = settings_crc32((const uint8_t *)&rec, offsetof(SettingsRecord, crc32));
  rec.commit = SETTINGS_COMMIT_MAGIC;

  if (s_has_active)
  {
    /* 始终写入另一页，保留当前页作为回退副本。 */
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

  /* 写成功后再切换活动页，保证 RAM 中状态和 Flash 一致。 */
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
    /* 没有有效记录就保留编译时默认参数，不报错也能启动。 */
    s_has_active = 0U;
    s_active_page = 0U;
    s_active_seq = 0U;
    return;
  }

  settings_apply_payload(&rec.payload);
  /* 记录当前活动页和序号，供下一次保存选择目标页与递增 seq。 */
  s_has_active = 1U;
  s_active_page = page;
  s_active_seq = rec.seq;
}

void settings_mark_dirty(void)
{
  /*
   * 只做“打脏标记 + 记录时间”，真正写 Flash 放到后台任务里。
   * 这样 UI 连续按键调参时不会频繁阻塞在 Flash 擦写。
   */
  s_dirty = 1U;
  s_dirty_tick = HAL_GetTick();
}

void settings_service(void)
{
  uint32_t now;

  /*
   * 延迟保存的意义：
   * - UI 连续调参时不会每次按键都擦写 Flash；
   * - 降低磨损，也减少频繁写 Flash 带来的卡顿。
   */
  /* 本工程中该函数由 DefaultTask 周期调用。 */
  if (s_dirty == 0U) return;

  now = HAL_GetTick();
  if ((uint32_t)(now - s_dirty_tick) < SETTINGS_SAVE_DELAY_MS)
  {
    return;
  }

  if (settings_save_now())
  {
    /* 成功后清脏，直到下一次参数变更再写。 */
    s_dirty = 0U;
  }
  else
  {
    /* 保存失败后延后重试，避免在错误状态下高频重复写入。 */
    s_dirty_tick = now;
  }
}
