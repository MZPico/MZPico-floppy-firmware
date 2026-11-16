// Host-simulation stand-in for pico/time.h, backed by the virtual clock in
// tests/sim/sim_core.cpp. Calls made on simulated core0 consume virtual time.
#pragma once
#include <stdint.h>
typedef uint64_t absolute_time_t;
#ifdef __cplusplus
extern "C" {
#endif
uint32_t time_us_32(void);
uint64_t time_us_64(void);
void sleep_ms(uint32_t ms);
void sleep_us(uint64_t us);
void busy_wait_us_32(uint32_t us);
absolute_time_t get_absolute_time(void);
#ifdef __cplusplus
}
#endif
static inline uint32_t to_ms_since_boot(absolute_time_t t) { return (uint32_t)(t / 1000u); }
