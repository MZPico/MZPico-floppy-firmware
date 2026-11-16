// Derived from Adafruit_Floppy (https://github.com/adafruit/Adafruit_Floppy),
// MIT License, Copyright (c) 2021 Limor Fried for Adafruit Industries.
// See LICENSES/MIT-Adafruit_Floppy.txt. Changes: Apache-2.0, see LICENSE.
// greaseweazle USB command protocol, Pico-SDK/TinyUSB port of the Adafruit
// greaseweazle.ino command loop (MIT). Speaks the flux-level GW protocol over
// USB CDC so the mature `gw`/fluxengine host tools drive this firmware.
#pragma once

#include <cstddef>
#include <cstdint>

void gw_init(); // set up the floppy drive object
void gw_task(); // process one pending command; call from the main loop

// Shared big flux-capture buffer (MAX_FLUX_PULSE_PER_TRACK bytes). Reused by the
// FDC disk engine — safe because the USB greaseweazle path and the in-machine
// FDC path never run concurrently (bench-over-USB vs. plugged into the MZ-800).
uint8_t *mzp_flux_buffer(size_t *cap);
