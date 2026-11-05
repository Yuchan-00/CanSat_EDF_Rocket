"""pytest: the ESP8266 UART <-> ESP-NOW bridge (firmware/esp_bridge) and its COM side (2026-11-06 firmware round).

* the portable chunker / ring of the bridge sketch (firmware/esp_bridge/src/bridge_core.*) built with the venv's zig c++
  and run: byte transparency, the flush rule, the ring, the MAC parser (host_test/test_bridge_core.cpp);
* the generated config header is current and agrees with params.json and with the COM's params_gen.h (the same UART baud
  on both ends of the COM <-> ESP wire);
* the generator refuses impossible settings (channel, TX power, MAC, baud error);
* the ESP sketch builds nothing from numbers of its own (every radio / timing value comes from the generated header).

The sketch itself is not compiled by this test: firmware/esp_bridge/build_arduino_cli.py compiles it with arduino-cli and
the core esp8266:esp8266 3.1.2 (README.md "빌드").
"""
from __future__ import annotations

import copy
import json
import os
import re
import subprocess
import sys

import pytest

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
EB = os.path.join(ROOT, "firmware", "esp_bridge")
sys.path.insert(0, EB)
import gen_esp_bridge_config as gen  # noqa: E402

sys.path.insert(0, os.path.join(ROOT, "cad"))
import layout as layout_mod  # noqa: E402

P = layout_mod.load_params(layout_mod.PARAMS_PATH, None)
BUILD = os.path.join(ROOT, "firmware", "esp_bridge", "host_test", "build")


def _have_zig() -> bool:
    r = subprocess.run([sys.executable, "-m", "ziglang", "version"], capture_output=True, text=True, encoding="utf-8",
                       errors="replace")
    return r.returncode == 0


@pytest.mark.skipif(not _have_zig(), reason="zig (python -m ziglang) not installed in the venv")
def test_bridge_core_host_test_passes():
    os.makedirs(BUILD, exist_ok=True)
    exe = os.path.join(BUILD, "test_bridge_core" + (".exe" if os.name == "nt" else ""))
    r = subprocess.run([sys.executable, "-m", "ziglang", "c++", "-std=c++11", "-O1", "-Wall", "-Wextra", "-Werror",
                        "-o", exe, os.path.join(EB, "host_test", "test_bridge_core.cpp"),
                        os.path.join(EB, "src", "bridge_core.cpp")],
                       capture_output=True, text=True, encoding="utf-8", errors="replace")
    assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-3000:]
    t = subprocess.run([exe], capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=300)
    m = re.search(r"BRIDGE_CORE (\d+) checks, (\d+) failures", t.stdout)
    assert t.returncode == 0 and m and int(m.group(2)) == 0 and int(m.group(1)) > 100_000, t.stdout[-3000:]


def test_config_header_is_current():
    r = subprocess.run([sys.executable, os.path.join(EB, "gen_esp_bridge_config.py"), "--check"], capture_output=True,
                       text=True, encoding="utf-8", errors="replace")
    assert r.returncode == 0, r.stdout + r.stderr


def _defines(path: str) -> dict[str, str]:
    with open(path, "r", encoding="utf-8") as f:
        return dict(re.findall(r"^#define\s+(\w+)\s+(.*?)\s*(?:/\*.*)?$", f.read(), re.M))


def test_config_matches_params_and_the_com_header():
    eb = P["flight_computer"]["link"]["esp_bridge"]
    d = _defines(os.path.join(EB, "src", "esp_bridge_config_gen.h"))
    assert d["BRIDGE_UART_BAUD"] == f"{eb['uart_baud']}UL"
    assert int(d["BRIDGE_ESPNOW_CHANNEL"]) == eb["espnow_channel"]
    assert float(d["BRIDGE_TX_DBM_VEHICLE"].rstrip("f")) == eb["tx_power_dbm_vehicle"]
    assert float(d["BRIDGE_TX_DBM_GROUND"].rstrip("f")) == eb["tx_power_dbm_ground"]
    for k in ("vehicle", "ground"):
        mac = [int(x, 16) for x in eb[f"mac_{k}"].split(":")]
        assert d[f"BRIDGE_MAC_{k.upper()}"] == "{" + ", ".join(f"0x{v:02X}" for v in mac) + "}"
    from sim import link_proto as lp
    assert int(d["LINK_LEN_MAX"]) == lp.LEN_MAX and int(d["LINK_MAX_FRAME_BYTES"]) == lp.FRAME_OVERHEAD + lp.LEN_MAX
    byte_us = 10e6 / eb["uart_baud"]
    assert int(d["BRIDGE_IDLE_FLUSH_US"].rstrip("UL")) == round(eb["idle_flush_bytes"] * byte_us)
    # the COM side of the same wire (tools/gen_firmware_params.py ESP_UART_*)
    fw = _defines(os.path.join(ROOT, "firmware", "atmega128_fc", "src", "params_gen.h"))
    assert fw["ESP_UART_BAUD"] == f"{eb['uart_baud']}UL"
    ubrr = int(fw["ESP_UART_UBRR"])
    assert abs(16e6 / (16 * (ubrr + 1)) / eb["uart_baud"] - 1) <= eb["uart_max_error"] <= 0.002


@pytest.mark.parametrize("key,value,msg", [
    ("espnow_channel", 14, "not in 1..13"),
    ("tx_power_dbm_vehicle", 21.0, "outside"),
    ("mac_ground", "01:CA:5A:00:00:02", "multicast"),
    ("mac_vehicle", "00:CA:5A:00:00:01", "not locally administered"),
    ("mac_vehicle", "02:CA:5A:00:00", "not a MAC"),
    ("uart_baud", 57600, "error"),
    ("idle_flush_bytes", 0.5, "outside"),
])
def test_generator_refuses(key, value, msg):
    p = copy.deepcopy(P)
    p["flight_computer"]["link"]["esp_bridge"][key] = value
    with pytest.raises(SystemExit) as e:
        gen.render(p)
    assert msg in str(e.value)


def test_same_mac_refused():
    p = copy.deepcopy(P)
    eb = p["flight_computer"]["link"]["esp_bridge"]
    eb["mac_ground"] = eb["mac_vehicle"]
    with pytest.raises(SystemExit):
        gen.render(p)


def test_render_is_deterministic():
    assert gen.render(P) == gen.render(json.loads(json.dumps(P)))


def test_sketch_takes_every_number_from_the_generated_header():
    with open(os.path.join(EB, "src", "main.cpp"), "r", encoding="utf-8") as f:
        src = f.read()
    for name in ("BRIDGE_UART_BAUD", "BRIDGE_ESPNOW_CHANNEL", "BRIDGE_TX_DBM_VEHICLE", "BRIDGE_TX_DBM_GROUND",
                 "BRIDGE_MAC_VEHICLE", "BRIDGE_MAC_GROUND", "BRIDGE_IDLE_FLUSH_US", "BRIDGE_FRAME_WAIT_US", "LINK_SYNC0",
                 "LINK_LEN_MAX"):
        assert name in src, name
    code = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    assert not re.search(r"Serial\.print|Serial\.println|printf", code), "the UART carries only link bytes"
    assert not re.search(r"wifi_set_channel\(\s*\d", code) and not re.search(r"setOutputPower\(\s*\d", code)
