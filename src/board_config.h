// MZPico-800-Floppy-Board — pin map and firmware identity.
//
// GPIO map extracted from MZPico-800-Floppy-Board.kicad_sch (purple 16MB Pico
// clone). Floppy signals are active-low, wired direct to the Pico. See
// ../CLAUDE.md and docs/PHASE0.md. NB: RDATA (GPIO20) needs an external ~1k
// pull-up to 3.3V — the board has none; the Pico read pin has no internal one.
#pragma once

// ---- Floppy interface (34-pin) ----
#define PIN_DENSITY 13 // ~DEN_RDY (density out / ready in share this net)
#define PIN_STEP 14    // ~STEP
#define PIN_DIR 15     // ~DIR
#define PIN_MOTOR 16   // ~MOTOR_ON
#define PIN_WGATE 17   // ~WGATE
#define PIN_WDATA 18   // ~WDATA (flux out, PIO)
#define PIN_HDSEL 19   // ~HDSEL (side select)
#define PIN_RDATA 20   // ~RDATA (flux in, PIO) -- external pull-up required
#define PIN_INDEX 21   // ~INDEX
#define PIN_TRK00 22   // ~TRK00
#define PIN_WPT 23     // ~WPT (write protect)
#define PIN_DS0 24     // ~DS0 (drive select 0)
#define PIN_DS1 25     // ~DS1 (drive select 1)

// Which drive-select line the firmware asserts. Phase 0 used DS0 with a
// twisted PC cable (post-twist connector = select 0). Switch to PIN_DS1 for a
// straight cable + a drive strapped as select 1.
#define PIN_SELECT PIN_DS0

// No status LED on this board (GPIO25 = DS1). -1 disables LED code.
#define PIN_LED (-1)

// ---- MZ-800 expansion bus (unused in Phase 1; reserved for the FDC phase) ----
// GPIO0-7 = shared C0..C7 (addr/data via SN74LVC245), 8=IORQ, 9=RD, 10=WR,
// 11=EXWAIT, 12=INT, 26=EN0, 27=EN1, 28=RD1, 29=RESET.

// ---- greaseweazle identity (matches Adafruit Floppy Generic, so gw talks) ----
#define GW_FIRMVER_MAJOR 1
#define GW_FIRMVER_MINOR 0
#define GW_MAXCMD 21
#define GW_HW_MODEL 8    // "Adafruity" model id accepted by gw host tools
#define GW_HW_SUBMODEL 0 // generic
#define GW_USB_SPEED 0   // Full Speed

// RP2040 flux sample rate (see arch_rp2 clkdiv: 3 * 24MHz -> 24MHz samples).
#define GW_SAMPLE_FREQ 24000000UL

// Max flux transitions buffered per track (Adafruit sizing: 500kHz/5 * 1.5 rev).
// ~150 KB static buffer — large but fits RP2040's 264 KB SRAM.
#define MAX_FLUX_PULSE_PER_TRACK ((uint32_t)(500000UL / 5 * 1.5))
