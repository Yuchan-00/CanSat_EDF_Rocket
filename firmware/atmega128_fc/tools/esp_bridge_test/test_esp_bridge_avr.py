"""pytest: the COM's ESP bridge backend (src/hal/esp_bridge.c) in simavr - reset hold and release, the boot-window drop,
bytes after the window unchanged, the reply on USART1 at ESP_UART_BAUD (run_esp_bridge_test.py).  Skipped - never
passed - when PlatformIO's avr-gcc, libsimavr or zig cc is not installed."""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_esp_bridge_test as rt  # noqa: E402

HAVE = (rt.tool("toolchain-atmelavr", "avr-gcc") and rt.tool("toolchain-atmelavr", "avr-objcopy") and rt.simavr_dir()
        and rt.have_zig())


@pytest.mark.skipif(not HAVE, reason="PlatformIO toolchain-atmelavr / tool-simavr or zig cc not installed")
def test_esp_bridge_backend_in_simavr():
    res = rt.run()
    assert res["status"] == "PASS", res
