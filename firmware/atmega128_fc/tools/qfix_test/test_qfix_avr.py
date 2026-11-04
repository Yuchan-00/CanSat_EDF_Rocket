"""pytest: the AVR inline-asm fx_mulhi equals its C definition on 200000 pairs in simavr (fw09 SPEC_firmware_v09
WP-X; run_qfix_test.py).  Skipped - never passed - when PlatformIO's avr-gcc or simavr is not installed."""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_qfix_test as rq  # noqa: E402

HAVE = rq.tool("toolchain-atmelavr", "avr-gcc") and rq.tool("tool-simavr", "simavr")


@pytest.mark.skipif(not HAVE, reason="PlatformIO toolchain-atmelavr / tool-simavr not installed")
def test_avr_fx_mulhi_equals_c_on_200000_pairs():
    res = rq.run()
    assert res["status"] == "PASS", res
    assert res["pairs"] == rq.PAIRS and res["asm_vs_ref_mismatch"] == 0 and res["outside_floor_or_floor_minus_1"] == 0
