// Host-simulation stand-in for hardware/irq.h.
#pragma once
#include <stdbool.h>
#define IO_IRQ_BANK0 13
typedef void (*irq_handler_t)(void);
#ifdef __cplusplus
extern "C" {
#endif
void irq_set_exclusive_handler(unsigned num, irq_handler_t handler);
void irq_set_enabled(unsigned num, bool enabled);
#ifdef __cplusplus
}
#endif
