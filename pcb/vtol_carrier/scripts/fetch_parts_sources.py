"""Verify ONLINE (read-only) that every BOM line of the VTOL carrier exists at JLCPCB/LCSC or devicemart.

User rule 2026-11-05 ("모든 부품은 JLCPCB의 부품 라이브러리 혹은 디바이스마트에 있는 부품들로만 넣어줘"): every populated
part, every DNP part with a BOM line and every lead / mating part needs a source that exists in the JLCPCB assembly
parts library (= LCSC catalogue, C-number) or at devicemart.co.kr (goods no=). This script never logs in, never adds to
a cart and never buys: it only sends the same anonymous GET / search requests a browser sends when a page is opened.

* LCSC / JLCPCB: GET https://cart.jlcpcb.com/shoppingCart/smtGood/getComponentDetail?componentCode=<C> (the JSON the
  jlcpcb.com/partdetail page loads) -> MPN, brand, package, stock, JLC library type ('base' = Basic, 'expand' =
  Extended). The raw JSON is saved under out/reports/sourcing/ with its sha256.
* devicemart: the search stream https://www.devicemart.co.kr/goods/search_sse_stream?search_text=<no> (what the site's
  search box loads) -> goods name, sale price, goods_status; and the goods page https://www.devicemart.co.kr/goods/view?no=
  <no> -> og:title + the spec summary line (sha256 of the raw page recorded, text excerpt saved).

Output: spec/parts_sources.json (input of build_spec.py; build_spec.py itself never touches the network). Each record
states the URL fetched and the fetch date. Re-running on another day may give other stock numbers - that is evidence,
not a deterministic artifact.

Run: B:/Claude/.venv/Scripts/python.exe -X utf8 scripts/fetch_parts_sources.py
"""
from __future__ import annotations

import datetime as _dt
import hashlib
import html
import json
import re
import time
import urllib.parse
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
OUT = HERE / "spec" / "parts_sources.json"
RAW = HERE / "out" / "reports" / "sourcing"
UA = {"User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64)"}

# key -> (source, code, expect substring in MPN / name (case-insensitive), what it is for)
LCSC = {
    "mcu": ("C6510", "ATMEGA128A-AU", "U1-U4 ATmega128A-AU TQFP-64"),
    "ldo": ("C6186", "AMS1117-3.3", "U6 AMS1117-3.3 SOT-223"),
    "esp": ("C89297", "ESP-12E", "U7 ESP-12E module (JLC SMT assembly; castellated)"),
    "bss138": ("C40912", "BSS138", "Q1-Q8, Q11 BSS138 SOT-23"),
    "pfet": ("C15127", "AO3401A", "Q10 AO3401A SOT-23 reverse-polarity P-FET"),
    "npn": ("C20526", "MMBT3904", "Q9 MMBT3904 SOT-23 buzzer driver"),
    "xtal": ("C70562", "X322516MRB4SI", "Y1-Y4 16 MHz 3225 4-pin crystal, CL 18 pF (C13738 X322516MLB4SI, the JLC Basic one, has CL 9 pF - does not match the 22 pF load caps)"),
    "schottky": ("C8598", "B5819W", "D6 B5819W SOD-123 buzzer flyback"),
    "led": ("C2286", "KT-0603R", "D1-D4 red LED 0603"),
    "ind10u": ("C1046", "SDFL2012S100KTF", "L1-L4 10 uH 0805 AVCC filter"),
    "r10k": ("C25804", "0603WAF1002T5E", "10 kOhm 1 % 0603"),
    "r1k": ("C21190", "0603WAF1001T5E", "1 kOhm 1 % 0603"),
    "r4k7": ("C23162", "0603WAF4701T5E", "4.7 kOhm 1 % 0603"),
    "r47k": ("C25819", "0603WAF4702T5E", "47 kOhm 1 % 0603"),
    "r220": ("C22962", "0603WAF2200T5E", "220 Ohm 1 % 0603"),
    "r100k": ("C25803", "0603WAF1003T5E", "100 kOhm 1 % 0603"),
    "c100n0402": ("C1525", "CL05B104KO5NNNC", "100 nF 16 V X7R 0402"),
    "c22p0402": ("C1555", "0402CG220J500NT", "22 pF 50 V C0G 0402"),
    "c27p0402": ("C1557", "0402CG270J500NT", "Y1-Y4 crystal load caps 27 pF 50 V C0G 0402 (rev 2c, CL 18 pF crystal)"),
    "c100n0603": ("C14663", "CC0603KRX7R9BB104", "100 nF 50 V X7R 0603"),
    "c10u0805": ("C15850", "CL21A106KAYNNNE", "10 uF 25 V X5R 0805"),
    "c22u1206": ("C12891", "CL31A226KAHNNNE", "22 uF 25 V X5R 1206"),
    "tant22u": ("C7183", "TAJA226M010RNJ", "C45 22 uF 10 V tantalum case A 3216-18 (AMS1117 output)"),
    "tant100u": ("C7196", "TAJB107M010RNJ", "C52 ESP bulk 100 uF 10 V tantalum case B 3528-21 (rev 2c; 6.3 V C16133 before)"),
    "elec220u": ("C2833309", "RVT220UF10V67RV0037", "C47 220 uF 10 V electrolytic 6.3 x 5.4 (servo rail)"),
    "hdr2x3": ("C65114", "2.54-2*3PPin", "J19-J22 ISP + J3 ESP flash 2x3 2.54 mm straight header"),
    "hdr1x2v": ("C55629809", "SH-PZ254V-1x2P", "J24-J26 1x2 2.54 mm straight header"),
    "hdr1x3ra": ("C55629817", "SH-PZ254R-1x3P", "J8-J12 servo / ESC 1x3 right-angle header"),
    "hdr1x2ra": ("C55629816", "SH-PZ254R-1x2P", "J16 arm 1x2 right-angle header"),
    "hdr1x4ra": ("C55629818", "SH-PZ254R-1x4P", "J18 sense 1x4 right-angle header"),
    "sock1x6ra": ("C55629861", "SH-PM254R-1x6P", "J2 BMP280 / J5 OpenLog 1x6 right-angle female socket"),
    "shunt": ("C100114", "Short Circuit Cap", "jumper shunts: J24, J25, J26, J3 1-2, J15 bench"),
    "spox2": ("C5364642", "22035025", "J14 Molex Mini-SPOX 22-03-5025 (= 5267-02A) 2-pin vertical"),
    "gh4": ("C161692", "BM04B-GHS-TBT", "J7 JST GH 4-pin vertical"),
    "ph2": ("C131337", "B2B-PH-K-S", "J15 power-switch lead + J17 buzzer JST PH 2-pin vertical"),
    "ghr4": ("C160418", "GHR-04V-S", "GPS lead: JST GH 4-pin housing (lead, hand-crimped)"),
    "ghcrimp": ("C189897", "SSHL-002T-P0.2", "GPS lead: GH crimp contacts x 8 (both ends)"),
    "phr2": ("C157955", "PHR-2", "buzzer lead: JST PH 2-pin housing"),
    "phcrimp": ("C111515", "SPH-002T-P0.5S", "buzzer lead: PH crimp contacts x 2"),
    "hdr1x6ra": ("C55629820", "SH-PZ254R-1x6P", "1x6 right-angle male header soldered on the OpenLog (lies flat in J5)"),
    "openlog_lcsc": ("C17253836", "DEV-13712", "SparkFun OpenLog at LCSC (listed, stock 0 at the search) - info only"),
}
DEVICEMART = {
    "gy521": ("1247052", "GY-521", "J1 IMU module GY-521 MPU-6050 [SZH-EK007] (user part; its straight 1x8 header goes into J1)"),
    "bmp280": ("15442588", "BMP280", "J2 barometer module BMP280 [TYE-BMP29] (user part, right-angle header fitted)"),
    "neo7m": ("10895400", "NEO-7M", "GPS module NEO-7M [SZH-NT07] (user part, top cap; carrier has J7 only)"),
    "logic_battery": ("16021596", "DTP105565P", "logic battery 1S 3.7 V 4200 mAh (user part, plugs into J14)"),
    "boost": ("15855364", "S9V11F5S6CMA", "J6 5 V step-up/step-down module with adjustable low-voltage cut-off (Pololu #2870)"),
    "openlog": ("1313116", "OpenLog", "J5 SparkFun OpenLog DEV-13712"),
    "toggle": ("13029107", "MTS-102", "logic power switch MTS-102 with Dupont harness (plugs onto J15)"),
    "esp_alt": ("1378118", "ESP-12E", "ALTERNATIVE source for U7 if hand-soldered (not the BOM default)"),
}


def _get(url: str, data: bytes | None = None, headers: dict | None = None, timeout: int = 40) -> bytes:
    req = urllib.request.Request(url, data=data, headers={**UA, **(headers or {})})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read()


def _text(raw: bytes) -> str:
    t = raw.decode("utf-8", "replace")
    t = re.sub(r"<script.*?</script>|<style.*?</style>", " ", t, flags=re.S)
    t = html.unescape(re.sub(r"<[^>]+>", " ", t))
    return re.sub(r"\s+", " ", t)


def lcsc(key: str, code: str, expect: str, use: str, today: str) -> dict:
    url = f"https://cart.jlcpcb.com/shoppingCart/smtGood/getComponentDetail?componentCode={code}"
    raw = _get(url)
    (RAW / f"LCSC_{code}.json").write_bytes(raw)
    d = (json.loads(raw).get("data") or {})
    model = d.get("componentModelEn") or ""
    lib = {"base": "Basic", "expand": "Extended"}.get(d.get("componentLibraryType"), d.get("componentLibraryType"))
    return {
        "key": key, "source": "JLCPCB/LCSC", "part_number": code, "for": use,
        "mpn": model, "manufacturer": d.get("componentBrandEn"), "package": d.get("componentSpecificationEn"),
        "description": (d.get("describe") or "").encode("ascii", "replace").decode()[:220],
        "stock": d.get("stockCount"), "jlc_library_type": lib, "assembly_process": d.get("assemblyProcess"),
        "found": bool(d) and expect.lower() in model.lower(),
        "urls": [f"https://jlcpcb.com/partdetail/{code}", url, d.get("lcscGoodsUrl") or ""],
        "fetched": today, "raw_sha256": hashlib.sha256(raw).hexdigest(),
        "raw_file": f"out/reports/sourcing/LCSC_{code}.json",
    }


def dm_search(no: str) -> dict | None:
    url = "https://www.devicemart.co.kr/goods/search_sse_stream?" + urllib.parse.urlencode({"search_text": no})
    req = urllib.request.Request(url, headers=UA)
    ev = None
    with urllib.request.urlopen(req, timeout=20) as r:
        for rawl in r:
            line = rawl.decode("utf-8", "replace").rstrip("\n")
            if line.startswith("event:"):
                ev = line[6:].strip()
            elif line.startswith("data:") and ev == "sphinx_result":
                recs = ((json.loads(line[5:]).get("list") or {}).get("record") or [])
                for it in recs:
                    if str(it.get("goods_seq")) == no:
                        return it
                return None
    return None


def devicemart(key: str, no: str, expect: str, use: str, today: str) -> dict:
    page = f"https://www.devicemart.co.kr/goods/view?no={no}"
    raw = _get(page, timeout=20)
    t = _text(raw)
    m = re.search(r'<meta property="og:title" content="([^"]*)"', raw.decode("utf-8", "replace"))
    title = html.unescape(m.group(1)) if m else ""
    # spec summary: the text just before the A/S block on the goods page
    spec = ""
    i = t.find("A/S 정보")
    if i > 0:
        spec = t[max(0, i - 400):i].strip()
    (RAW / f"DM_{no}.txt").write_text(f"{page}\n{title}\n{spec}\n", encoding="utf-8", newline="\n")
    rec = dm_search(no)
    return {
        "key": key, "source": "devicemart", "part_number": f"no={no}", "for": use,
        "product_name": title or (rec or {}).get("goods_name"),
        "search_record": None if rec is None else {
            "goods_name": rec.get("goods_name"), "sale_price_krw": rec.get("sale_price"),
            "goods_status": rec.get("goods_status"), "stock_field": rec.get("stock"),
            "manufacture": rec.get("manufacture")},
        "spec_excerpt": spec[-300:],
        "found": bool(title) and expect.lower() in title.lower(),
        "urls": [page, "https://www.devicemart.co.kr/goods/search_sse_stream?search_text=" + no],
        "fetched": today, "raw_sha256": hashlib.sha256(raw).hexdigest(),
        "raw_file": f"out/reports/sourcing/DM_{no}.txt (text excerpt; sha256 is of the full page)",
        "note": "devicemart 'stock' field meaning is not documented (many sellable goods show 0); goods_status "
                "'normal' = listed for sale, 'purchasing' = being restocked",
    }


MANUAL = HERE / "spec" / "parts_sources_manual.json"


def fallback(key: str, prior: dict) -> dict | None:
    """devicemart refused this PC after ~15 requests (2026-11-05 05:32 KST): reuse an earlier successful direct fetch
    (same output file, its own date) or, failing that, the search-listing record in spec/parts_sources_manual.json."""
    old = (prior.get("devicemart") or {}).get(key)
    if old and old.get("found") and old.get("verification", "goods page + search stream") != "devicemart search listing":
        return {**old, "verification": old.get("verification", "goods page + search stream"), "note_fallback": "direct fetch failed on this run; record kept from the earlier successful "
                                        f"fetch of {old.get('fetched')}"}
    man = (json.loads(MANUAL.read_text(encoding="utf-8")).get("devicemart") or {}).get(key) if MANUAL.exists() else None
    if not man:
        return None
    no, expect, use = DEVICEMART[key]
    return {"key": key, "source": "devicemart", "part_number": f"no={no}", "for": use,
            "product_name": man["goods_name"],
            "search_record": {k: man.get(k) for k in ("goods_name", "sale_price_krw", "goods_status", "stock_field")},
            "found": expect.lower() in man["goods_name"].lower() and man.get("goods_status") == "normal",
            "verification": "devicemart search listing",
            "urls": [f"https://www.devicemart.co.kr/goods/view?no={no}",
                     "https://www.devicemart.co.kr/goods/search_sse_stream?search_text="
                     + urllib.parse.quote(man["search_keyword"])],
            "fetched": man["fetched"], "raw_sha256": None,
            "raw_file": "spec/parts_sources_manual.json (search listing, goods page not reachable)"}


def main() -> int:
    RAW.mkdir(parents=True, exist_ok=True)
    today = _dt.date.today().isoformat()
    prior = json.loads(OUT.read_text(encoding="utf-8")) if OUT.exists() else {}
    out = {"schema": "vtol_carrier_parts_sources/1", "fetched": today,
           "method": __doc__.split("\n\n")[1].strip(), "lcsc": {}, "devicemart": {}, "errors": []}
    for key, (code, expect, use) in LCSC.items():
        try:
            out["lcsc"][key] = lcsc(key, code, expect, use, today)
        except Exception as e:  # noqa: BLE001 - record the failure as evidence
            out["errors"].append(f"{key} {code}: {e!r}")
        time.sleep(0.4)
    for key, (no, expect, use) in DEVICEMART.items():
        try:                                   # one try: devicemart refuses repeated requests from one PC
            out["devicemart"][key] = {**devicemart(key, no, expect, use, today),
                                      "verification": "goods page + search stream"}
        except Exception as e:  # noqa: BLE001
            fb = fallback(key, prior)
            if fb is None:
                out["errors"].append(f"{key} no={no}: {e!r}")
            else:
                fb["direct_fetch_error"] = repr(e)[:160]
                out["devicemart"][key] = fb
        time.sleep(3)
    for sec in ("lcsc", "devicemart"):
        for k, v in out[sec].items():
            if not v["found"]:
                out["errors"].append(f"{sec} {k} {v['part_number']}: expected '{(LCSC.get(k) or DEVICEMART.get(k))[1]}' "
                                     f"not in '{v.get('mpn') or v.get('product_name')}'")
    OUT.write_text(json.dumps(out, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    for sec in ("lcsc", "devicemart"):
        for k, v in out[sec].items():
            print(f"{sec:10} {k:14} {v['part_number']:12} found={v['found']} "
                  f"{v.get('mpn') or v.get('product_name')} | {v.get('package', '')} | "
                  f"{v.get('jlc_library_type') or (v.get('search_record') or {}).get('goods_status')} | "
                  f"stock {v.get('stock', (v.get('search_record') or {}).get('stock_field'))}")
    print("errors:", out["errors"])
    return 1 if out["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
