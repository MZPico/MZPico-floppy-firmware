// greaseweazle USB command protocol, Pico-SDK/TinyUSB port of the Adafruit
// greaseweazle.ino command loop (MIT). Speaks the flux-level GW protocol over
// USB CDC so the mature `gw`/fluxengine host tools drive this firmware.
#pragma once

void gw_init(); // set up the floppy drive object
void gw_task(); // process one pending command; call from the main loop
