#!/usr/bin/env python3
"""Fixtures for check_core1_ram.py: synthetic objdump bodies, no toolchain.

    python3 tools/ci/test_check_core1_ram.py
"""
import contextlib
import io
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_core1_ram as c  # noqa: E402

LOOP, ISR = "_ZL11listen_loopv", "_ZN12_GLOBAL__N_1L17fdc_bus_reset_isrEv"
LOCK = "_ZZ13fdc_disk_taskvENKUlvE_clEv"


def ram(addr, *lines):
    return (addr, [f"{addr + 2 * i:8x}:\t{ln}" for i, ln in enumerate(lines)])


def base(**extra):
    """The four roots and one locked region, all clean, plus `extra`."""
    funcs = {
        LOOP: ram(0x20000000, "nop", "b.n\t20000000 <_ZL11listen_loopv>"),
        "_Z13fdc_reg_writehh": ram(0x20000100, "bx\tlr"),
        "_Z12fdc_reg_readh": ram(0x20000200, "bx\tlr"),
        ISR: ram(0x20000300, "bx\tlr"),
        LOCK: ram(0x20000400, "bx\tlr"),
    }
    funcs.update(extra)
    return funcs


def run(funcs):
    err = io.StringIO()
    with contextlib.redirect_stderr(err), contextlib.redirect_stdout(io.StringIO()):
        rc = c.check(funcs, "fixture.elf")
    return rc, err.getvalue()


class CheckerFixtures(unittest.TestCase):
    def test_clean(self):
        self.assertEqual(run(base())[0], 0)

    def test_conditional_branch_inside_function_is_fine(self):
        f = base()
        f[ISR] = ram(0x20000300, "cmp\tr0, #0",
                     f"bne.n\t20000300 <{ISR}>", "bx\tlr")
        self.assertEqual(run(f)[0], 0)

    def test_conditional_tail_call_is_followed(self):
        # bne.n into a RAM helper that itself calls a flash veneer
        f = base(helper=ram(0x20000500, "bl\t20000600 <__flash_veneer>"),
                 __flash_veneer=ram(0x20000600, "bx\tlr"))
        f[ISR] = ram(0x20000300, "cmp\tr0, #0", "bne.n\t20000500 <helper>",
                     "bx\tlr")
        rc, err = run(f)
        self.assertEqual(rc, 1)
        self.assertIn("helper: calls flash", err)

    def test_conditional_branch_out_of_locked_region(self):
        f = base(helper=ram(0x20000500, "bx\tlr"))
        f[LOCK] = ram(0x20000400, "cmp\tr0, #0", "beq.w\t20000500 <helper>",
                      "bx\tlr")
        rc, err = run(f)
        self.assertEqual(rc, 1)
        self.assertIn("jumps out", err)

    def test_offset_entry_is_followed(self):
        f = base(helper=ram(0x20000500, "nop",
                            "bl\t10001000 <flash_fn>", "bx\tlr"))
        f[ISR] = ram(0x20000300, "b.n\t20000502 <helper+0x2>")
        rc, err = run(f)
        self.assertEqual(rc, 1)
        self.assertIn("helper: calls flash", err)

    def test_bl_and_bls_are_told_apart(self):
        # `bl` is a call (followed as one), `bls` a conditional branch
        f = base(helper=ram(0x20000500, "bx\tlr"))
        f[ISR] = ram(0x20000300, "bls.n\t20000500 <helper>",
                     "bl\t20000500 <helper>", "bx\tlr")
        self.assertEqual(run(f)[0], 0)


if __name__ == "__main__":
    unittest.main()
