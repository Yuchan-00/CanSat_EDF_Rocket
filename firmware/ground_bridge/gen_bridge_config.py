"""Generate ``src/bridge_config_gen.h`` for the USB <-> LoRa ground bridge (SPEC §10.3, ``sx127x`` backend).

Every number the bridge firmware uses comes from here, never typed into the C++ source:

* radio settings: ``flight_computer.link.sx127x`` in ``params.json`` (frequency, SF, bandwidth, coding rate,
  sync word, TX power, preamble) - the same block the flight computer's SX127x driver is built from, so both
  ends always match; explicit header and payload CRC on (SPEC §10.3);
* serial baud: ``flight_computer.link.uart_baud`` (``sim.link_model`` assumes the PC reaches the ground
  SX127x through a USB-UART at this baud, so the latency model and the ground station's reply timeout hold);
* frame constants (sync bytes, LEN range, frame overhead): ``sim/link_proto.py``;
* the idle flush time: ``sim.link_model.EBYTE_IDLE_GAP_BYTES_ASSUMED`` byte times at that baud (the same
  "end of packet after this much UART silence" rule as a transparent Ebyte module).

The values are range-checked against the SX1276 / sandeepmistry LoRa library limits; an impossible value
stops the build instead of producing a radio that silently runs other settings than the vehicle.

Run: ``.venv/Scripts/python.exe firmware/ground_bridge/gen_bridge_config.py [--check] [--preset NAME]``
(PlatformIO runs it before every build through ``extra_gen.py``).

Invariant: the header is a pure function of its sources (named with their sha256 in the header); the same
sources give the byte-identical header.
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
from sim import link_model as lm  # noqa: E402
from sim import link_proto as lp  # noqa: E402

OUT = os.path.join(HERE, "src", "bridge_config_gen.h")
# SX1276/77/78 datasheet + sandeepmistry/LoRa 0.8.0 API limits (checked, not used as settings)
SX127X_FREQ_RANGE_HZ = (137_000_000, 1_020_000_000)
SX127X_BANDWIDTHS_HZ = (7_800, 10_400, 15_600, 20_800, 31_250, 41_700, 62_500, 125_000, 250_000, 500_000)
SX127X_SF_RANGE = (6, 12)
SX127X_CR_DENOM_RANGE = (5, 8)
PA_BOOST_DBM_RANGE = (2, 20)          # LoRa.setTxPower(level, PA_OUTPUT_PA_BOOST_PIN)
PREAMBLE_RANGE = (6, 65535)
AVR_STD_BAUDS = (1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200)


def sha256(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def _rel(path: str) -> str:
    try:
        return os.path.relpath(path, ROOT).replace("\\", "/")
    except ValueError:                     # --out on another drive (Windows): keep the absolute path
        return path.replace("\\", "/")


def _check(name: str, ok: bool, value) -> None:
    if not ok:
        raise SystemExit(f"gen_bridge_config: {name} = {value!r} is not possible on an SX127x / this bridge")


def render(p: dict) -> str:
    lk = p["flight_computer"]["link"]
    sx = lk["sx127x"]
    freq, sf, bw = int(sx["freq_hz"]), int(sx["spreading_factor"]), int(sx["bandwidth_hz"])
    cr, sw, dbm, pre = int(sx["coding_rate_denom"]), int(sx["sync_word"]), int(sx["tx_power_dbm"]), int(sx["preamble_symbols"])
    baud = int(lk["uart_baud"])
    _check("link.sx127x.freq_hz", SX127X_FREQ_RANGE_HZ[0] <= freq <= SX127X_FREQ_RANGE_HZ[1], freq)
    _check("link.sx127x.spreading_factor", SX127X_SF_RANGE[0] <= sf <= SX127X_SF_RANGE[1], sf)
    _check("link.sx127x.bandwidth_hz", bw in SX127X_BANDWIDTHS_HZ, bw)
    _check("link.sx127x.coding_rate_denom", SX127X_CR_DENOM_RANGE[0] <= cr <= SX127X_CR_DENOM_RANGE[1], cr)
    _check("link.sx127x.sync_word", 0 <= sw <= 0xFF, sw)
    _check("link.sx127x.tx_power_dbm", PA_BOOST_DBM_RANGE[0] <= dbm <= PA_BOOST_DBM_RANGE[1], dbm)
    _check("link.sx127x.preamble_symbols", PREAMBLE_RANGE[0] <= pre <= PREAMBLE_RANGE[1], pre)
    _check("link.uart_baud", baud in AVR_STD_BAUDS, baud)
    max_frame = lp.FRAME_OVERHEAD + lp.LEN_MAX
    byte_us = lm.UART_BITS_PER_BYTE * 1_000_000 / baud
    idle_us = int(round(lm.EBYTE_IDLE_GAP_BYTES_ASSUMED * byte_us))
    frame_wait_us = int(round((max_frame + lm.EBYTE_IDLE_GAP_BYTES_ASSUMED) * byte_us))
    cfg = lm.LinkConfig.from_params(p, radio="sx127x")
    toa_cmd = lm.lora_time_on_air(lp.CMD_FRAME_BYTES, sf, bw, cr, pre)
    toa_tlm = lm.lora_time_on_air(lp.TELEM_FRAME_BYTES, sf, bw, cr, pre)
    toa_tlm2 = lm.lora_time_on_air(lp.TELEM2_FRAME_BYTES, sf, bw, cr, pre)
    toa_tlm3 = lm.lora_time_on_air(lp.TELEM3_FRAME_BYTES, sf, bw, cr, pre)
    # v0.9 (fw09 RP1): the power builds' TELEM v4 / v5 once sim/link_proto.py defines them (LEN_MAX follows lp.LEN_MAX)
    v09 = [(n, int(getattr(lp, f"{n}_FRAME_BYTES"))) for n in ("TELEM4", "TELEM5") if hasattr(lp, f"{n}_FRAME_BYTES")]
    longest = [n for n in ("TELEM5", "TELEM4", "TELEM3", "TELEM2", "TELEM")
               if getattr(lp, f"{n}_FRAME_BYTES", None) == max_frame]
    rt = lm.round_trip(cfg)
    srcs = [os.path.join(ROOT, "params.json"), os.path.join(ROOT, "sim", "link_proto.py"),
            os.path.join(ROOT, "sim", "link_model.py"), os.path.abspath(__file__)]
    L: list[str] = []
    w = L.append
    w("/* bridge_config_gen.h - GENERATED by firmware/ground_bridge/gen_bridge_config.py. DO NOT EDIT.")
    w(" *")
    w(" * Sources (sha256):")
    for s in srcs:
        w(f" *   {_rel(s):<42} {sha256(s)}")
    w(f" * Preset: {p['meta'].get('preset', '?')}")
    w(" *")
    w(" * Timing from sim/link_model.py (sx127x backend, informative):")
    w(f" *   time on air CMD {lp.CMD_FRAME_BYTES} B = {toa_cmd * 1e3:.1f} ms, TELEM {lp.TELEM_FRAME_BYTES} B = {toa_tlm * 1e3:.1f} ms"
      f" (v0.7 VTOL TELEM2 {lp.TELEM2_FRAME_BYTES} B = {toa_tlm2 * 1e3:.1f} ms, v0.8 GPS TELEM3 {lp.TELEM3_FRAME_BYTES} B = "
      f"{toa_tlm3 * 1e3:.1f} ms" + ("" if not v09 else ",") + (");" if not v09 else ""))
    if v09:
        w(" *   v0.9 power builds " + ", ".join(f"{n} {b} B = {lm.lora_time_on_air(b, sf, bw, cr, pre) * 1e3:.1f} ms"
                                            for n, b in v09) + ");")
    w(f" *   round trip {rt['rtt_mean_s'] * 1e3:.0f} ms mean (TELEM v1),")
    w(f" *   ground reply timeout {rt['reply_timeout_s'] * 1e3:.0f} ms, max poll rate {rt['max_poll_hz']:.2f} Hz"
      f" (link.poll_hz {cfg.poll_hz:g}).")
    w(" */")
    w("#ifndef BRIDGE_CONFIG_GEN_H")
    w("#define BRIDGE_CONFIG_GEN_H")
    w("")
    w("/* USB serial (PC side) - flight_computer.link.uart_baud, 8N1 */")
    w(f"#define BRIDGE_SERIAL_BAUD        {baud}UL")
    w("")
    w("/* LoRa radio - flight_computer.link.sx127x (explicit header, payload CRC on: SPEC 10.3) */")
    w(f"#define BRIDGE_LORA_FREQ_HZ       {freq}L")
    w(f"#define BRIDGE_LORA_SF            {sf}")
    w(f"#define BRIDGE_LORA_BW_HZ         {bw}L")
    w(f"#define BRIDGE_LORA_CR_DENOM      {cr}")
    w(f"#define BRIDGE_LORA_SYNC_WORD     0x{sw:02X}")
    w(f"#define BRIDGE_LORA_TX_POWER_DBM  {dbm}")
    w(f"#define BRIDGE_LORA_PREAMBLE      {pre}")
    w("")
    w("/* frame format - sim/link_proto.py (SPEC 10.3): A5 5A | LEN | VEH TYPE SEQ PAYLOAD | CRC16 */")
    w(f"#define LINK_SYNC0                0x{lp.SYNC0:02X}")
    w(f"#define LINK_SYNC1                0x{lp.SYNC1:02X}")
    w(f"#define LINK_LEN_MIN              {lp.LEN_MIN}")
    w(f"#define LINK_LEN_MAX              {lp.LEN_MAX}")
    w(f"#define LINK_FRAME_OVERHEAD       {lp.FRAME_OVERHEAD}   /* sync x2, LEN, CRC x2: frame bytes = LEN + this */")
    if longest and longest[0] in ("TELEM5", "TELEM4"):
        w(f"#define LINK_MAX_FRAME_BYTES      {max_frame}   /* the longest frame of any build (LEN_MAX {lp.LEN_MAX}; v0.9: the "
          f"{longest[0]} frame of the power builds) */")
    else:
        w(f"#define LINK_MAX_FRAME_BYTES      {max_frame}   /* the longest frame of any build (LEN_MAX {lp.LEN_MAX}; v0.8: the "
          f"TELEM3 frame, v0.7: TELEM2) */")
    w("")
    w("/* serial -> radio packet boundaries: a complete frame goes out at once; bytes that are not a frame go out")
    w(" * after EBYTE_IDLE_GAP_BYTES_ASSUMED byte times of silence (sim/link_model.py), an incomplete frame after")
    w(" * a whole maximum frame time + that gap (transparent: nothing is ever dropped) */")
    w(f"#define BRIDGE_IDLE_FLUSH_US      {idle_us}UL")
    w(f"#define BRIDGE_FRAME_WAIT_US      {frame_wait_us}UL")
    w("")
    w("#endif /* BRIDGE_CONFIG_GEN_H */")
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
        print(f"{_rel(a.out)}: {'up to date' if ok else 'OUT OF DATE - run gen_bridge_config.py'}")
        return 0 if ok else 1
    if old != text:
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        with open(a.out, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    print(f"{_rel(a.out)}: {'written' if old != text else 'unchanged'} (sha256 {hashlib.sha256(text.encode()).hexdigest()[:16]})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
