"""pytest: the power builds' ADC conversion chain (src/hal/adc.c, the flight path that the benches and the co-simulation
replace by constant codes) runs in simavr with distinct input voltages and puts every code in its slot (fw09 review F2;
run_adc_chain_test.py).  Skipped - never passed - when PlatformIO's avr-gcc, libsimavr or zig cc is not installed."""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_adc_chain_test as ra  # noqa: E402

HAVE = (ra.tool("toolchain-atmelavr", "avr-gcc") and ra.tool("toolchain-atmelavr", "avr-objcopy") and ra.simavr_dir()
        and ra.have_zig())


@pytest.mark.skipif(not HAVE, reason="PlatformIO toolchain-atmelavr / tool-simavr or zig cc not installed")
def test_adc_chain_codes_in_their_slots_in_simavr():
    res = ra.run()
    assert res["status"] == "PASS", res
    for env in ra.ENVS:
        r = res["envs"][env]
        assert all(r["checks"].values()), r
