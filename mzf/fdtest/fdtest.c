/*
 * FDTEST -- floppy controller test for the Sharp MZ-800.
 *
 * Exercises a WD179x-compatible controller at ports D8h-DFh the way the
 * MZ-800's own software does (Disk BASIC ROM, CP/M 4.1), and prints a verdict
 * per check. Built for the MZPico floppy board, but it only talks WD179x, so
 * it runs against the original Sharp card as well -- which is the point: the
 * same program, the same checks, on the real thing and on the bridge.
 *
 * The same binary runs in the firmware's host test suite (tests/mz800), where
 * the monitor calls it uses are trapped, the keys are scripted and the FDC is
 * the firmware itself on a simulated drive.
 *
 *   1  READ TEST   read-only; any MZ-800 disk (CP/M or Disk BASIC)
 *   2  WRITE TEST  writes and formats the inner cylinders -- SCRATCH DISK ONLY
 *   3  BOTH
 *
 * Conventions it follows from the ROM: every byte on D8h-DBh is complemented
 * on the bus (the card's data bus is inverted), DRQ is polled (no wait
 * states, no interrupts), a multi-sector READ is ended by FORCE INTERRUPT,
 * and the ROM's side 0 is the drive's physical head 1.
 *
 * Output and input use only the MZ-700/800 monitor entry points PRNT (0012h)
 * and GETL (0003h); it returns to the monitor (00ADh) when done.
 *
 * Copyright 2026 The MZPico floppy firmware authors. SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>

#define VERSION "1.5"

/* ---- WD179x ---- */
#define P_CMD 0xD8
#define P_TRK 0xD9
#define P_SEC 0xDA
#define P_DAT 0xDB
#define P_DRV 0xDC
#define P_SIDE 0xDD
#define P_DENS 0xDE
#define P_EINT 0xDF

#define ST_BUSY 0x01
#define ST_DRQ 0x02
#define ST_TRK0 0x04
#define ST_LOST 0x04
#define ST_CRC 0x08
#define ST_RNF 0x10
#define ST_SEEKERR 0x10
#define ST_DELETED 0x20
#define ST_WPROT 0x40
#define ST_NOTREADY 0x80
#define ST_ERRS (ST_NOTREADY | ST_WPROT | ST_RNF | ST_CRC | ST_LOST)

#define BUF ((uint8_t *)0x6000) /* transfer buffer, 18 x 512 max */
#define BUF2 ((uint8_t *)0xA800)

/* ---- primitives in assembly (globals in, globals out) ---- */
uint8_t *x_ptr;     /* transfer pointer */
uint16_t x_max;     /* bytes to move */
uint16_t x_cnt;     /* bytes moved */
uint16_t x_first;   /* idle polls before the first DRQ */
uint16_t x_polls;   /* idle polls in total */
uint8_t x_st;       /* final (true) status */
uint8_t x_mode;     /* bit0: Force Interrupt once x_max bytes are read
                       bit1: write: pad with 4Eh after x_max bytes (format) */

uint8_t io_in(uint8_t port) __z88dk_fastcall __naked {
__asm
    ld c,l
    in l,(c)
    ld h,0
    ret
__endasm;
}

/* (port << 8) | value */
void io_out(uint16_t pv) __z88dk_fastcall __naked {
__asm
    ld c,h
    out (c),l
    ret
__endasm;
}

void mon_putc(uint8_t c) __z88dk_fastcall __naked {
__asm
    ld a,l
    call 0x0012
    ret
__endasm;
}

/* GETL: read a line (ended by CR) into the buffer at DE. */
void mon_getl(char *buf) __z88dk_fastcall __naked {
__asm
    ex de,hl
    call 0x0003
    ret
__endasm;
}

void cpu_di(void) __naked {
__asm
    di
    ret
__endasm;
}

/*
 * READ data phase: poll STATUS, take a byte on every DRQ, stop when BUSY
 * drops. Idle poll = 139 T-states (39 us at 3.547 MHz); the 16-bit idle budget
 * is ~2.5 s. Times in polls are converted to ms by the C side.
 */
void x_read(void) __naked {
__asm
    ld hl,(_x_ptr)
    ld de,0
    ld (_x_cnt),de
    ld (_x_polls),de
    ld de,0xffff
    ld (_x_first),de
xr_loop:
    in a,(0xd8)
    cpl
    bit 1,a
    jr nz,xr_take
    bit 0,a
    jr z,xr_done
    push hl
    ld hl,(_x_polls)
    inc hl
    ld (_x_polls),hl
    ld a,h
    and l
    inc a
    pop hl
    jr z,xr_tmo
    jp xr_loop
xr_take:
    in a,(0xdb)
    cpl
    ld (hl),a
    inc hl
    ld de,(_x_first)
    inc de
    ld a,d
    or e
    jr nz,xr_nf
    ld de,(_x_polls)
    ld (_x_first),de
xr_nf:
    ld de,(_x_cnt)
    inc de
    ld (_x_cnt),de
    ld a,(_x_max)
    cp e
    jr nz,xr_loop
    ld a,(_x_max+1)
    cp d
    jr nz,xr_loop
    ld a,(_x_mode)
    bit 0,a
    jr z,xr_cap
    ld a,0x2f            ; Force Interrupt D0h, complemented
    out (0xd8),a
    ld b,20
xr_fi:
    djnz xr_fi
    in a,(0xd8)
    cpl
    ld (_x_st),a
    ret
xr_cap:
    dec hl               ; buffer full: keep overwriting the last byte
    jp xr_loop
xr_done:
    ld (_x_st),a
    ret
xr_tmo:
    ld a,0x2f
    out (0xd8),a
    ld a,0xff
    ld (_x_st),a
    ret
__endasm;
}

/* WRITE data phase: feed a byte on every DRQ until BUSY drops. */
void x_write(void) __naked {
__asm
    ld hl,(_x_ptr)
    ld de,0
    ld (_x_cnt),de
    ld (_x_polls),de
xw_loop:
    in a,(0xd8)
    cpl
    bit 1,a
    jr nz,xw_give
    bit 0,a
    jr z,xw_done
    push hl
    ld hl,(_x_polls)
    inc hl
    ld (_x_polls),hl
    ld a,h
    and l
    inc a
    pop hl
    jr z,xw_tmo
    jp xw_loop
xw_give:
    ld de,(_x_cnt)
    ld a,(_x_max)
    cp e
    jr nz,xw_data
    ld a,(_x_max+1)
    cp d
    jr nz,xw_data
    ld a,(_x_mode)       ; all data given
    bit 1,a
    jr z,xw_loop         ; sector: wait for the controller to finish
    ld a,0xb1            ; format: pad with 4Eh (complemented)
    out (0xdb),a
    jp xw_loop
xw_data:
    ld a,(hl)
    inc hl
    cpl
    out (0xdb),a
    inc de
    ld (_x_cnt),de
    jp xw_loop
xw_done:
    ld (_x_st),a
    ret
xw_tmo:
    ld a,0x2f
    out (0xd8),a
    ld a,0xff
    ld (_x_st),a
    ret
__endasm;
}

/* ---- console ---- */
static void putc(char c) { mon_putc(c == '\n' ? 0x0D : (uint8_t)c); }
static void puts(const char *s) {
  while (*s)
    putc(*s++);
}
static void putdec(uint16_t v) {
  char b[6];
  int8_t i = 0;
  do {
    b[i++] = (char)('0' + v % 10);
    v /= 10;
  } while (v);
  while (i)
    putc(b[--i]);
}
static void puthex(uint8_t v) {
  const char *h = "0123456789ABCDEF";
  putc(h[v >> 4]);
  putc(h[v & 15]);
}
/* Answers are typed as a line and confirmed with CR, through the monitor's
 * GETL (0003h): no key debouncing to get wrong (1.0 took one held key as
 * several answers; 1.1's release wait made it deaf on the machine), and the
 * monitor echoes and edits the input itself. GETL hands back the whole screen
 * line the cursor is on -- prompt included -- so the answer is the LAST
 * non-blank character before the CR. */
static char linebuf[100];
static char getkey(void) {
  int8_t i;
  char k;
  mon_getl(linebuf);
  for (i = 0; i < 99 && linebuf[i] != 0x0D; ++i)
    ;
  while (i > 0 && linebuf[i - 1] == ' ')
    --i;
  k = i > 0 ? linebuf[i - 1] : 0;
  if (k >= 'a' && k <= 'z')
    k -= 32;
  return k;
}

static uint16_t n_pass, n_fail;
static uint8_t n_data_fail; /* failures that make writing unsafe */
static uint8_t cur_blocks;  /* this check guards the write tests */
static uint8_t cur_ok;

static void begin(const char *name) {
  puts(name);
  cur_ok = 1;
  cur_blocks = 1;
}
static void fail(const char *why) {
  if (cur_ok)
    puts(" FAIL:");
  puts(" ");
  puts(why);
  cur_ok = 0;
}
static void failst(const char *what, uint8_t st) {
  fail(what);
  puts(" ST=");
  puthex(st);
}
static void end(void) {
  if (cur_ok) {
    puts(" OK");
    ++n_pass;
  } else {
    ++n_fail;
    if (cur_blocks)
      ++n_data_fail;
  }
  putc('\n');
}

/* polls -> ms: 139 T-states per idle poll at 3.546895 MHz */
static uint16_t polls_ms(uint16_t p) { return (uint16_t)(((uint32_t)p * 139u) / 3547u); }

/* ---- WD helpers ---- */
static uint8_t drive;

static void wout(uint8_t port, uint8_t v) {
  if (port <= P_DAT)
    v = (uint8_t)~v;
  io_out(((uint16_t)port << 8) | v);
}
static uint8_t win(uint8_t port) {
  uint8_t v = io_in(port);
  return port <= P_DAT ? (uint8_t)~v : v;
}
static void pause(uint16_t n) {
  while (n--)
    io_in(P_EINT);
}
static void command(uint8_t c) {
  wout(P_CMD, c);
  pause(8);
}
static uint8_t wait_ready(void) {
  uint16_t n = 0;
  uint8_t s;
  do {
    s = win(P_CMD);
    if (!(s & ST_BUSY))
      return s;
  } while (++n != 0);
  command(0xD0);
  return 0xFF;
}
static void motor_on(void) { wout(P_DRV, (uint8_t)(0x84 | drive)); }
static void side(uint8_t rom_side) { wout(P_SIDE, rom_side ? 0xFE : 0xFF); }
static uint8_t restore(void) {
  command(0x00);
  return wait_ready();
}
static uint8_t seek(uint8_t c, uint8_t verify) {
  wout(P_DAT, c);
  command(verify ? 0x14 : 0x10);
  return wait_ready();
}
static uint8_t read_sec(uint8_t t, uint8_t s, uint8_t *buf, uint16_t len, uint8_t multi) {
  wout(P_TRK, t);
  wout(P_SEC, s);
  x_ptr = buf;
  x_max = len;
  x_mode = multi ? 1 : 0;
  command(multi ? 0x90 : 0x80);
  x_read();
  return x_st;
}
static uint8_t write_sec(uint8_t t, uint8_t s, uint8_t *buf, uint16_t len, uint8_t deleted) {
  wout(P_TRK, t);
  wout(P_SEC, s);
  x_ptr = buf;
  x_max = len;
  x_mode = 0;
  command(deleted ? 0xA1 : 0xA0);
  x_write();
  return x_st;
}
static uint8_t read_addr(uint8_t *id) {
  x_ptr = id;
  x_max = 6;
  x_mode = 0;
  command(0xC0);
  x_read();
  return x_st;
}

/* ---- density: declared on port DEh as CP/M 4.1 does (00h DD, 81h HD) ---- */
static uint8_t g_hd;
static void declare_density(uint8_t hd) {
  g_hd = hd;
  wout(P_DENS, hd ? 0x81 : 0x00);
}

/* ---- geometry, discovered with READ ADDRESS ---- */
/* [0] ROM side 0, [1] ROM side 1 (data cylinders); [2] the boot cylinder */
static uint8_t g_nsec[3], g_n[3], g_first[3], g_maxcyl;
static uint16_t g_size[3];

static uint8_t discover(uint8_t rs, uint8_t slot) {
  uint8_t id[6], i, seen[32], cnt = 0, lo = 255;
  for (i = 0; i < 32; ++i)
    seen[i] = 0;
  side(rs);
  for (i = 0; i < 40; ++i) {
    uint8_t st = read_addr(id);
    if (st & ST_ERRS)
      continue;
    if (id[2] < 32 && !seen[id[2]]) {
      seen[id[2]] = 1;
      ++cnt;
      if (id[2] < lo)
        lo = id[2];
    }
    g_n[slot] = id[3] & 3;
  }
  g_nsec[slot] = cnt;
  g_first[slot] = lo;
  g_size[slot] = (uint16_t)128 << g_n[slot];
  return cnt;
}

/* ---- the checks ---- */
static void t_detect(void) {
  begin("DETECT CONTROLLER .....");
  wout(P_TRK, 0x5A);
  if (win(P_TRK) != 0x5A)
    fail("TRACK REG");
  wout(P_TRK, 0xA5);
  if (win(P_TRK) != 0xA5)
    fail("TRACK REG");
  end();
}

static void t_restore(void) {
  uint8_t st;
  begin("RESTORE ...............");
  motor_on();
  st = restore();
  if (st == 0xFF)
    fail("TIMEOUT");
  else if (st & ST_NOTREADY)
    failst("NOT READY", st);
  else if (!(st & ST_TRK0))
    failst("NO TRK0", st);
  end();
}

/* Which density are the data cylinders? Try DD, then HD, on cylinder 2. An
 * HD disk formatted by CP/M carries a DD boot track on cylinder 0, so cylinder
 * 0 cannot answer this. */
static void t_density(void) {
  uint8_t id[6], i, hd, found = 0;
  begin("DENSITY ...............");
  seek(2, 0);
  side(1);
  for (hd = 0; hd < 2 && !found; ++hd) {
    declare_density(hd);
    for (i = 0; i < 4 && !found; ++i)
      if (!(read_addr(id) & ST_ERRS))
        found = 1;
  }
  if (!found) {
    declare_density(0);
    fail("NO IDS AT DD OR HD");
  }
  puts(g_hd ? " HD" : " DD");
  end();
}

static void t_seek_verify(void) {
  static const uint8_t cyls[] = {1, 2, 5, 10, 20, 39, 0};
  uint8_t i, st;
  begin("SEEK + VERIFY .........");
  for (i = 0; i < sizeof(cyls); ++i) {
    st = seek(cyls[i], 1);
    if (st & (ST_SEEKERR | ST_CRC | ST_NOTREADY)) {
      failst("CYL", st);
      putdec(cyls[i]);
    }
    if (win(P_TRK) != cyls[i])
      fail("TRACK REG");
  }
  /* 80 or 40 cylinders? */
  st = seek(79, 1);
  g_maxcyl = (st & ST_SEEKERR) ? 39 : 79;
  seek(0, 0);
  end();
}

static void t_geometry(void) {
  uint8_t rs;
  begin("GEOMETRY (READ ADDR) ..");
  seek(2, 0);
  for (rs = 0; rs < 2; ++rs)
    if (!discover(rs, rs))
      fail("NO IDS");
  end();
  for (rs = 0; rs < 2; ++rs) {
    puts("  SIDE ");
    putdec(rs);
    puts(": ");
    putdec(g_nsec[rs]);
    puts(" X ");
    putdec(g_size[rs]);
    puts(" FROM ");
    putdec(g_first[rs]);
    putc('\n');
  }
  puts("  CYLINDERS: ");
  putdec((uint16_t)g_maxcyl + 1);
  putc('\n');
}

static uint16_t rd_ok, rd_err, lat_sum, lat_max, lat_n;

static void read_track_all(uint8_t c, uint8_t rs) {
  uint8_t r, st;
  for (r = 0; r < g_nsec[rs]; ++r) {
    uint8_t s = (uint8_t)(g_first[rs] + r);
    st = read_sec(c, s, BUF, g_size[rs], 0);
    if ((st & ST_ERRS) || x_cnt != g_size[rs]) {
      ++rd_err;
      if (rd_err <= 4) {
        failst("C", st);
        putdec(c);
        putc('/');
        putdec(rs);
        putc('/');
        putdec(s);
      }
    } else {
      uint16_t l = polls_ms(x_first);
      ++rd_ok;
      lat_sum += l;
      ++lat_n;
      if (l > lat_max)
        lat_max = l;
    }
  }
}

static void t_read_all(void) {
  static const uint8_t cyls[] = {2, 3, 10, 20, 39};
  uint8_t i;
  begin("READ SECTORS ..........");
  rd_ok = rd_err = lat_sum = lat_max = lat_n = 0;
  for (i = 0; i < sizeof(cyls); ++i) {
    seek(cyls[i], 0);
    read_track_all(cyls[i], 0);
    read_track_all(cyls[i], 1);
  }
  end();
  puts("  ");
  putdec(rd_ok);
  puts(" OK ");
  putdec(rd_err);
  puts(" BAD, LAT AVG ");
  putdec(lat_n ? lat_sum / lat_n : 0);
  puts(" MAX ");
  putdec(lat_max);
  puts(" MS\n"); /* fits the 40-column screen */
}

static void t_read_boot_track(void) {
  uint8_t st;
  begin("READ CYL 0 (BOOT) .....");
  /* The boot track is DD even on a CP/M HD disk, and DD on HD media is
   * marginal on drives that sense density from the media hole: a failure
   * here is reported but does not stop the write tests, which use the data
   * cylinders. */
  cur_blocks = 0;
  seek(0, 0);
  side(0);
  {
    /* The boot track is DD even on a CP/M HD disk (the boot ROM only reads
     * DD); a plain HD data disk has HD there. Try DD first, then HD. */
    uint8_t hd = g_hd, d, ok = 0;
    for (d = 0; d < 2 && !ok; ++d) {
      declare_density(d);
      ok = discover(0, 2) != 0;
    }
    if (!ok)
      fail("NO IDS ON SIDE 0");
    else {
      st = read_sec(0, g_first[2], BUF, g_size[2], 0);
      if (st & ST_ERRS)
        failst("READ", st);
    }
    declare_density(hd);
  }
  end();
  puts("  ");
  putdec(g_nsec[2]);
  puts(" X ");
  putdec(g_size[2]);
  putc('\n');
}

static void t_read_multiple(void) {
  uint8_t st, r;
  uint16_t n = (uint16_t)g_nsec[1] * g_size[1];
  begin("READ MULTIPLE + FI ....");
  seek(3, 0);
  side(1);
  st = read_sec(3, g_first[1], BUF, n, 1);
  if (x_cnt != n)
    fail("SHORT");
  if (st & ST_ERRS)
    failst("STATUS AFTER FI", st);
  /* the same data, sector by sector */
  for (r = 0; r < g_nsec[1] && cur_ok; ++r) {
    uint16_t i;
    uint8_t *a = BUF + (uint16_t)r * g_size[1];
    st = read_sec(3, (uint8_t)(g_first[1] + r), BUF2, g_size[1], 0);
    for (i = 0; i < g_size[1]; ++i)
      if (a[i] != BUF2[i]) {
        fail("DATA DIFFERS SEC");
        putdec(g_first[1] + r);
        break;
      }
  }
  end();
}

/* CP/M 4.1: STEP + FORCE INTERRUPT bursts, never waiting for BUSY. */
static void t_step_bursts(void) {
  uint8_t i, id[6], st;
  begin("STEP BURSTS (CP/M) ....");
  restore();
  for (i = 0; i < 4; ++i) { /* surplus STEP OUT at track 0: absorbed */
    command(0x60);
    command(0xD0);
  }
  for (i = 0; i < 7; ++i) {
    command(0x50);
    command(0xD0);
  }
  wait_ready();
  side(1);
  st = read_addr(id);
  if (st & ST_ERRS)
    failst("READ ADDR", st);
  else if (id[0] != 7) {
    fail("HEAD ON CYL");
    putdec(id[0]);
  }
  if (win(P_TRK) != 7)
    fail("TRACK REG");
  end();
}

static void fill(uint8_t *p, uint16_t n, uint8_t seed) {
  uint16_t i;
  for (i = 0; i < n; ++i)
    p[i] = (uint8_t)(seed + i * 7 + (i >> 8));
}
static uint8_t same(uint8_t *a, uint8_t *b, uint16_t n) {
  while (n--)
    if (*a++ != *b++)
      return 0;
  return 1;
}

static void t_write(void) {
  uint8_t c = (uint8_t)(g_maxcyl - 6), rs, r, st;
  begin("WRITE + READ BACK .....");
  seek(c, 0);
  for (rs = 0; rs < 2; ++rs) {
    side(rs);
    for (r = 0; r < g_nsec[1]; ++r) {
      uint8_t s = (uint8_t)(g_first[1] + r);
      fill(BUF, 512, (uint8_t)(c + rs * 64 + s));
      st = write_sec(c, s, BUF, 512, 0);
      if (st & ST_ERRS) {
        failst("WRITE", st);
        break;
      }
      st = read_sec(c, s, BUF2, 512, 0);
      if ((st & ST_ERRS) || !same(BUF, BUF2, 512)) {
        failst("VERIFY", st);
        break;
      }
    }
  }
  /* every sector still readable: the splices stayed inside their fields */
  for (rs = 0; rs < 2 && cur_ok; ++rs) {
    side(rs);
    for (r = 0; r < g_nsec[1]; ++r) {
      uint8_t s = (uint8_t)(g_first[1] + r);
      st = read_sec(c, s, BUF2, 512, 0);
      fill(BUF, 512, (uint8_t)(c + rs * 64 + s));
      if ((st & ST_ERRS) || !same(BUF, BUF2, 512)) {
        failst("REREAD", st);
        break;
      }
    }
  }
  end();
}

static void t_write_deleted(void) {
  uint8_t c = (uint8_t)(g_maxcyl - 6), st;
  begin("WRITE DELETED MARK ....");
  seek(c, 0);
  side(1);
  fill(BUF, 512, 0x33);
  st = write_sec(c, g_first[1], BUF, 512, 1);
  if (st & ST_ERRS)
    failst("WRITE", st);
  st = read_sec(c, g_first[1], BUF2, 512, 0);
  if (!(st & ST_DELETED))
    failst("NO DELETED BIT", st);
  if (!same(BUF, BUF2, 512))
    fail("DATA");
  end();
}

/* WRITE TRACK with a standard WD image, 9 x 512 (or the disk's geometry). */
static uint16_t build_format(uint8_t *p, uint8_t c, uint8_t h, uint8_t ns) {
  uint8_t *s = p, r;
  uint16_t i;
#define REP(n, b) for (i = 0; i < (n); ++i) *p++ = (b)
  REP(80, 0x4E); REP(12, 0); REP(3, 0xF6); *p++ = 0xFC; REP(50, 0x4E);
  for (r = 1; r <= ns; ++r) {
    REP(12, 0); REP(3, 0xF5); *p++ = 0xFE; *p++ = c; *p++ = h; *p++ = r; *p++ = 2;
    *p++ = 0xF7; REP(22, 0x4E); REP(12, 0); REP(3, 0xF5); *p++ = 0xFB;
    REP(512, 0xE5); *p++ = 0xF7; REP(84, 0x4E);
  }
#undef REP
  return (uint16_t)(p - s);
}

static void t_format(void) {
  uint8_t c = (uint8_t)(g_maxcyl - 4), rs, r, st, ns = g_nsec[1];
  uint16_t n, i;
  begin("FORMAT (WRITE TRACK) ..");
  if (ns != 9 && ns != 18)
    ns = 9;
  seek(c, 0);
  for (rs = 0; rs < 2; ++rs) {
    side(rs);
    n = build_format(BUF, c, rs ? 0 : 1, ns);
    x_ptr = BUF;
    x_max = n;
    x_mode = 2;
    command(0xF0);
    x_write();
    if (x_st & (ST_ERRS & ~ST_LOST)) {
      failst("WRITE TRACK", x_st);
      break;
    }
    for (r = 1; r <= ns; ++r) {
      st = read_sec(c, r, BUF2, 512, 0);
      if (st & ST_ERRS) {
        failst("READ", st);
        putdec(r);
        break;
      }
      for (i = 0; i < 512; ++i)
        if (BUF2[i] != 0xE5) {
          fail("FILL");
          break;
        }
    }
  }
  end();
}

static void read_tests(void) {
  t_detect();
  t_restore();
  t_density();
  t_seek_verify();
  t_geometry();
  t_read_boot_track();
  t_read_all();
  t_read_multiple();
  t_step_bursts();
}

static void write_tests(void) {
  t_write();
  t_write_deleted();
  t_format();
}

int main(void) {
  char k;
  cpu_di();
  mon_putc(0x16); /* CLR: clear the screen (Sharp ASCII 16h via PRNT) */
  puts("FDTEST " VERSION " - WD179X AT D8H\n");
  do {
    puts("DRIVE (0-3) + CR? ");
    k = getkey();
  } while (k < '0' || k > '3');
  drive = (uint8_t)(k - '0');
  do {
    puts("1=READ 2=WRITE(SCRATCH!) 3=BOTH? ");
    k = getkey();
  } while (k < '1' || k > '3');
  if (k != '1') {
    puts("WRITES NEAR THE HUB. Y=GO? ");
    if (getkey() != 'Y') {
      puts("CANCELLED\n");
      declare_density(0); /* as on the normal exit, below */
      return 0;
    }
  }
  wout(P_EINT, 0);
  read_tests();
  if (k != '1' && n_data_fail == 0)
    write_tests();
  else if (k != '1')
    puts("WRITE TESTS SKIPPED (READ FAILED)\n");
  /* Leave the controller as power-up found it: DD. The boot ROM and Disk
   * BASIC never write DEh, so an HD declaration left behind by an HD run
   * outlives the program and the next boot reads its DD disk at HD
   * (FD:Loading error until a bus RESET clears it). */
  declare_density(0);
  wout(P_DRV, 0); /* motor off */
  puts("FDTEST RESULT: ");
  puts(n_fail ? "FAIL" : "PASS");
  puts(" (");
  putdec(n_pass);
  puts(" OK, ");
  putdec(n_fail);
  puts(" FAILED)\n");
  return 0;
}
