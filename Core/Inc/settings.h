#ifndef CAR_SETTINGS_H
#define CAR_SETTINGS_H

#include <stdbool.h>

/* Load parameters from flash if a valid record exists. */
void settings_load(void);

/* Mark parameters as dirty; a delayed save will be performed in settings_service(). */
void settings_mark_dirty(void);

/* Service routine for deferred flash save, should be called periodically from a task. */
void settings_service(void);

/* Immediate save, returns true on success. */
bool settings_save_now(void);

#endif /* CAR_SETTINGS_H */
