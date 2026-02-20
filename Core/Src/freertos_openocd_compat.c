#include "FreeRTOS.h"

/*
 * OpenOCD FreeRTOS awareness in some versions still looks for uxTopUsedPriority.
 * Newer FreeRTOS ports use uxTopReadyPriority internally, so provide a
 * compatibility symbol with the expected name.
 */
#ifdef __GNUC__
__attribute__((used))
#endif
const unsigned int uxTopUsedPriority = (unsigned int)(configMAX_PRIORITIES - 1U);
