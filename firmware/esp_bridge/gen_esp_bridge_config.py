"""Generate ``src/esp_bridge_config_gen.h`` for the ESP8266 UART <-> ESP-NOW bridge (both ends: the vehicle's ESP-12E on
the COM's USART1, the ground's ESP-07S-class module on a USB-UART).

Every number the bridge sketch uses comes from here, never typed into the C++ source:

* ``flight_computer.link.esp_bridge`` in ``params.json``: the UART baud (the same on both ends; the COM's USART1 runs it,
  ``tools/gen_firmware_params.py`` ESP_UART_*), the ESP-NOW channel, the TX power of each end, the two station MAC
  addresses the sketch sets (so each end knows its peer without reading a module), the idle-flush length in byte times;
* frame constants (sync bytes, LEN range, frame overhead, the longest frame): ``sim/link_proto.py``.

The values are range-checked (channel 1..13, TX power 0..20.5 dBm - the range of the ESP8266 Arduino core's
``WiFi.setOutputPower``, A -, unicast locally administered MACs, the UART divider error on the ESP's 80 MHz UART clock and
on the COM's 16 MHz USART with U2X = 0); an impossible value stops the build.

Run: ``.venv/Scripts/python.exe firmware/esp_bridge/gen_esp_bridge_config.py [--check] [--preset NAME]``
(PlatformIO runs it before every build through ``extra_gen.py``).

Invariant: the header is a pure function of its sources (named with their sha256 in the header); the same sources give
the byte-identical header.
"""
from __future__ import annotations

import argparse
import hashlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
for _p in (ROOT, os.path.join(ROOT, "cad")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import layout as layout_mod  # noqa: E402  (cad/layout.py)
from sim import link_proto as lp  # noqa: E402

OUT = os.path.join(HERE, "src", "esp_bridge_config_gen.h")
ESP_UART_CLK_HZ = 80_000_000          # ESP8266 UART clock (APB 80 MHz; divider = clk / baud, A: ESP8266 TRM, not on disk)
AVR_F_CPU_HZ = 16_000_000             # the COM (flight_computer.com.f_cpu_hz, checked below)
CHANNELS = range(1, 14)               # 2.4 GHz channels 1..13 (Korea; A: check the rules for the site)
TX_DBM_RANGE = (0.0, 20.5)            # WiFi.setOutputPower range of the ESP8266 Arduino core (A)
ESPNOW_MAX = 250                      # ESP-NOW payload limit = BRIDGE_ESPNOW_MAX in src/bridge_core.h (A)
UART_BITS = 10                        # 8N1


class GenError(SystemExit):
    pass


def sha256(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def _rel(path: str) -> str:
    try:
        return os.path.relpath(path, ROOT).replace("\\", "/")
    except ValueError:
        return path.replace("\\", "/")


def parse_mac(s: str, name: str) -> list[int]:
    parts = str(s).split(":")
    try:
        mac = [int(x, 16) for x in parts]
    except ValueError:
        mac = []
    if len(parts) != 6 or len(mac) != 6 or any(len(x) != 2 or not 0 <= v <= 255 for x, v in zip(parts, mac)):
        raise GenError(f"gen_esp_bridge_config: {name} = {s!r} is not a MAC 'xx:xx:xx:xx:xx:xx'")
    if mac[0] & 0x01:
        raise GenError(f"gen_esp_bridge_config: {name} = {s!r} is a multicast address (bit 0 of the first byte)")
    if not mac[0] & 0x02:
        raise GenError(f"gen_esp_bridge_config: {name} = {s!r} is not locally administered (bit 1 of the first byte): "
                       "the sketch sets it with wifi_set_macaddr, so it must not be a vendor address")
    return mac


def avr_uart_error(baud: int) -> tuple[int, float]:
    """(UBRR, relative error) of the COM's USART1 at U2X = 0 (as the ilink: simavr ignores U2X)"""
    ubrr = max(0, round(AVR_F_CPU_HZ / (16 * baud) - 1))
    return ubrr, AVR_F_CPU_HZ / (16 * (ubrr + 1)) / baud - 1.0


def esp_uart_error(baud: int) -> float:
    div = round(ESP_UART_CLK_HZ / baud)
    return ESP_UART_CLK_HZ / div / baud - 1.0


def settings(p: dict) -> dict:
    lk = p["flight_computer"]["link"]
    eb = lk.get("esp_bridge")
    if not isinstance(eb, dict):
        raise GenError("gen_esp_bridge_config: params.json flight_computer.link.esp_bridge missing")
    if int(p["flight_computer"]["com"]["f_cpu_hz"]) != AVR_F_CPU_HZ:
        raise GenError("gen_esp_bridge_config: the COM is not a 16 MHz part (flight_computer.com.f_cpu_hz)")
    baud, max_err = int(eb["uart_baud"]), float(eb["uart_max_error"])
    ubrr, avr_err = avr_uart_error(baud)
    esp_err = esp_uart_error(baud)
    if abs(avr_err) > max_err or abs(esp_err) > max_err:
        raise GenError(f"gen_esp_bridge_config: {baud} baud: COM error {avr_err:+.3%}, ESP error {esp_err:+.3%} > "
                       f"link.esp_bridge.uart_max_error {max_err:.2%}")
    ch = int(eb["espnow_channel"])
    if ch not in CHANNELS:
        raise GenError(f"gen_esp_bridge_config: link.esp_bridge.espnow_channel {ch} not in 1..13")
    tx = {k: float(eb[f"tx_power_dbm_{k}"]) for k in ("vehicle", "ground")}
    for k, v in tx.items():
        if not TX_DBM_RANGE[0] <= v <= TX_DBM_RANGE[1]:
            raise GenError(f"gen_esp_bridge_config: link.esp_bridge.tx_power_dbm_{k} {v} outside {TX_DBM_RANGE} dBm")
    macs = {k: parse_mac(eb[f"mac_{k}"], f"link.esp_bridge.mac_{k}") for k in ("vehicle", "ground")}
    if macs["vehicle"] == macs["ground"]:
        raise GenError("gen_esp_bridge_config: the vehicle and the ground MAC are the same")
    idle_bytes = float(eb["idle_flush_bytes"])
    if not 1.0 <= idle_bytes <= 32.0:
        raise GenError(f"gen_esp_bridge_config: link.esp_bridge.idle_flush_bytes {idle_bytes} outside 1..32")
    max_frame = lp.FRAME_OVERHEAD + lp.LEN_MAX
    if max_frame > ESPNOW_MAX:
        raise GenError(f"gen_esp_bridge_config: the longest link frame {max_frame} B exceeds the ESP-NOW payload {ESPNOW_MAX} B")
    byte_us = UART_BITS * 1_000_000 / baud
    return {"baud": baud, "ubrr": ubrr, "avr_err": avr_err, "esp_err": esp_err, "ch": ch, "tx": tx, "macs": macs,
            "idle_us": int(round(idle_bytes * byte_us)), "frame_wait_us": int(round((max_frame + idle_bytes) * byte_us)),
            "max_frame": max_frame, "byte_us": byte_us, "idle_bytes": idle_bytes}


def render(p: dict) -> str:
    s = settings(p)
    srcs = [os.path.join(ROOT, "params.json"), os.path.join(ROOT, "sim", "link_proto.py"), os.path.abspath(__file__)]
    L: list[str] = []
    w = L.append

    def mac_c(m: list[int]) -> str:
        return "{" + ", ".join(f"0x{v:02X}" for v in m) + "}"

    w("/* esp_bridge_config_gen.h - GENERATED by firmware/esp_bridge/gen_esp_bridge_config.py. DO NOT EDIT.")
    w(" *")
    w(" * Sources (sha256):")
    for src in srcs:
        w(f" *   {_rel(src):<42} {sha256(src)}")
    w(f" * Preset: {p['meta'].get('preset', '?')}")
    w(" *")
    w(f" * UART {s['baud']} 8N1 on both ends: byte {s['byte_us']:.1f} us; COM USART1 UBRR {s['ubrr']} (U2X 0) error "
      f"{s['avr_err']:+.3%}, ESP divider error {s['esp_err']:+.4%};")
    w(f" * the longest link frame {s['max_frame']} B = {s['max_frame'] * s['byte_us'] / 1e3:.1f} ms on the UART.")
    w(" */")
    w("#ifndef ESP_BRIDGE_CONFIG_GEN_H")
    w("#define ESP_BRIDGE_CONFIG_GEN_H")
    w("")
    w("/* UART0 of both ESPs - flight_computer.link.esp_bridge.uart_baud, 8N1 (= the COM's ESP_UART_BAUD) */")
    w(f"#define BRIDGE_UART_BAUD          {s['baud']}UL")
    w("")
    w("/* ESP-NOW - flight_computer.link.esp_bridge (station interface, fixed channel, 802.11b PHY) */")
    w(f"#define BRIDGE_ESPNOW_CHANNEL     {s['ch']}")
    w(f"#define BRIDGE_TX_DBM_VEHICLE     {s['tx']['vehicle']:.1f}f")
    w(f"#define BRIDGE_TX_DBM_GROUND      {s['tx']['ground']:.1f}f")
    w(f"#define BRIDGE_MAC_VEHICLE        {mac_c(s['macs']['vehicle'])}   /* {':'.join(f'{v:02X}' for v in s['macs']['vehicle'])} */")
    w(f"#define BRIDGE_MAC_GROUND         {mac_c(s['macs']['ground'])}   /* {':'.join(f'{v:02X}' for v in s['macs']['ground'])} */")
    w("")
    w("/* frame format - sim/link_proto.py (SPEC 10.3): A5 5A | LEN | VEH TYPE SEQ PAYLOAD | CRC16 */")
    w(f"#define LINK_SYNC0                0x{lp.SYNC0:02X}")
    w(f"#define LINK_SYNC1                0x{lp.SYNC1:02X}")
    w(f"#define LINK_LEN_MIN              {lp.LEN_MIN}")
    w(f"#define LINK_LEN_MAX              {lp.LEN_MAX}")
    w(f"#define LINK_FRAME_OVERHEAD       {lp.FRAME_OVERHEAD}   /* sync x2, LEN, CRC x2: frame bytes = LEN + this */")
    w(f"#define LINK_MAX_FRAME_BYTES      {s['max_frame']}")
    w("")
    w("/* UART -> air packet boundaries (src/bridge_core.h BridgeChunker): a complete frame at once; other bytes after")
    w(f" * link.esp_bridge.idle_flush_bytes = {s['idle_bytes']:g} byte times of UART silence, an incomplete frame after the")
    w(" * longest frame + that gap; at most BRIDGE_ESPNOW_MAX (250) bytes per packet */")
    w(f"#define BRIDGE_IDLE_FLUSH_US      {s['idle_us']}UL")
    w(f"#define BRIDGE_FRAME_WAIT_US      {s['frame_wait_us']}UL")
    w("")
    w("#endif /* ESP_BRIDGE_CONFIG_GEN_H */")
    w("")
    return "\n".join(L)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--check", action="store_true", help="exit 1 if the header on disk is not up to date")
    ap.add_argument("--preset", default=None, help="params.json preset (default: the file's preset)")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args(argv)
    p = layout_mod.load_params(layout_mod.PARAMS_PATH, a.preset)
    text = render(p)
    old = None
    if os.path.exists(a.out):
        with open(a.out, "r", encoding="utf-8", newline="") as f:
            old = f.read()
    if a.check:
        ok = old == text
        print(f"{_rel(a.out)}: {'up to date' if ok else 'OUT OF DATE - run gen_esp_bridge_config.py'}")
        return 0 if ok else 1
    if old != text:
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        with open(a.out, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    print(f"{_rel(a.out)}: {'written' if old != text else 'unchanged'} (sha256 {hashlib.sha256(text.encode()).hexdigest()[:16]})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
