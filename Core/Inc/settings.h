#ifndef CAR_SETTINGS_H
#define CAR_SETTINGS_H

#include <stdbool.h>

/* 从 Flash 读取上次保存的参数；如果记录无效则保持当前默认值。 */
/* Load parameters from flash if a valid record exists. */
void settings_load(void);

/* 标记参数已修改，实际写 Flash 在后台延迟执行（保护 Flash 寿命）。 */
/* Mark parameters as dirty; a delayed save will be performed in settings_service(). */
void settings_mark_dirty(void);

/* 周期调用：处理延迟保存、失败重试等后台逻辑。 */
/* Service routine for deferred flash save, should be called periodically from a task. */
void settings_service(void);

/* 立即保存当前参数到 Flash（通常用于调试或特殊场景）。 */
/* Immediate save, returns true on success. */
bool settings_save_now(void);

#endif /* CAR_SETTINGS_H */
