// Host-simulation stand-in for the Pico SDK's hardware/gpio.h. Only what the
// firmware sources under test use. Implemented in tests/sim/sim_core.cpp.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define GPIO_OUT 1
#define GPIO_IN 0
#define GPIO_IRQ_LEVEL_LOW 0x1u
#define GPIO_IRQ_LEVEL_HIGH 0x2u
#define GPIO_IRQ_EDGE_FALL 0x4u
#define GPIO_IRQ_EDGE_RISE 0x8u

enum gpio_function { GPIO_FUNC_SIO = 5, GPIO_FUNC_PIO0 = 6, GPIO_FUNC_PIO1 = 7 };

#ifdef __cplusplus
extern "C" {
#endif
bool gpio_get(unsigned pin);
void gpio_put(unsigned pin, bool value);
void gpio_init(unsigned pin);
void gpio_set_dir(unsigned pin, bool out);
void gpio_pull_up(unsigned pin);
void gpio_pull_down(unsigned pin);
void gpio_disable_pulls(unsigned pin);
void gpio_set_function(unsigned pin, enum gpio_function fn);
void gpio_set_irq_enabled(unsigned pin, uint32_t events, bool enabled);
uint32_t gpio_get_irq_event_mask(unsigned pin);
void gpio_acknowledge_irq(unsigned pin, uint32_t events);
#ifdef __cplusplus
}
#endif
