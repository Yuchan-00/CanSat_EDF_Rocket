"""JLCPCB parts-library evidence for the rev 3 BOM -> spec/parts_sources.json (read-only, cache-first).

Invariant: a part number is "orderable" only if a JLCPCB component-detail record that was FETCHED says so (rule I7 of
spec/REV3_PLAN.md). Nothing is typed from memory: the code list and the quantity per board come from
spec/rev3_plan.json, every other number from the newest cached record of the public endpoint
``cart.jlcpcb.com/shoppingCart/smtGood/getComponentDetail`` (no login, no cookie, no cart, nothing is ordered).

I7:  available = min(stockCount, canPresaleNumber)  >=  max(20 x qty_per_board + lossNumber, leastNumber),
     isBuyComponent == "1", noBuyReason empty, assemblyMode in {smtWeld, manualWeld}.
     available < 1000 is reported as THIN (not a failure).

A record older than ``--today`` (default: the local date) is stale: with ``--online`` it is fetched again (one GET
per code, >= 2.5 s apart, the cache file is replaced only by a valid record of the same code); without ``--online`` a
stale BOM line is an error. A failing BOM line is an error (exit 1) - the file is still written so the failure can be
read. Alternates and option parts are reported but do not fail the run.

The output carries the fetch time of every record (evidence, not a deterministic artifact): the same cache gives a
byte-identical file, a re-fetch does not. ``carrier_spec.json`` therefore takes only code / MPN / library type /
assembly from here, never stock numbers.

Run:  cd <board>/scripts && B:/Claude/.venv/Scripts/python.exe -X utf8 -B jlc_sources.py
          [--online] [--refetch C1,C2,...] [--today YYYY-MM-DD]
Cache: $JLC_CACHE_DIR (os.pathsep-separated list) or the scratch directories named in DEFAULT_CACHE.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import re
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
PLAN = HERE / "spec" / "rev3_plan.json"
OUT = HERE / "spec" / "parts_sources.json"

_SCR = Path("C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e"
            "/scratchpad/pcb_rev3")
#: first entry = the directory a re-fetch writes to
DEFAULT_CACHE = [_SCR / "jlc_cache", _SCR / "verify" / "jlc_refetch"]

UA = ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
      "(KHTML, like Gecko) Chrome/130.0.0.0 Safari/537.36")
DETAIL = "https://cart.jlcpcb.com/shoppingCart/smtGood/getComponentDetail?componentCode={code}"
SPACING_S = 2.5
BOARDS = 20          # I7 is evaluated for 20 assembled boards (plan section 2)
THIN = 1000
MODES = {"smtWeld", "manualWeld"}

#: quantity per board of the parts that are not a BOM line (alternates / options): the refs they would be fitted on
EXTRA_QTY = {
    # alternates (plan "alternates")
    "C55629861": 2, "C54876735": 2,                    # J2, J5
    "C167203": 1,                                      # L5
    "C52037397": 1, "C2829262": 1,                     # J7
    "C1852149": 1,                                     # U8
    "C46061676": 5, "C46061675": 1, "C46061677": 1,    # J8..J12 / J16 / J18 as SMD right-angle headers
    "C160352": 2,                                      # J15, J17
    # options (plan "option_parts")
    "C6510": 4,                                        # D4: U1..U4 fitted by JLC
    "C32713283": 5, "C32713268": 3, "C41417359": 3,    # D6 option B: J19..J22 + J3 (2x3), J24..J26 (1x2)
    "C7183": 1, "C55629817": 5, "C55629816": 1, "C55629818": 1,   # D3: rev 2c part numbers
    "C7420339": 8,                                     # D8: other BSS138
}


def cache_dirs() -> list[Path]:
    env = os.environ.get("JLC_CACHE_DIR")
    if env:
        return [Path(x) for x in env.split(os.pathsep) if x]
    return list(DEFAULT_CACHE)


def newest_record(code: str, dirs: list[Path]) -> tuple[dict, Path] | None:
    """Newest valid record of ``code`` over all cache directories (valid = http 200 and the same componentCode)."""
    best: tuple[dict, Path] | None = None
    for d in dirs:
        f = d / f"{code}.json"
        if not f.is_file():
            continue
        try:
            rec = json.loads(f.read_text(encoding="utf-8"))
        except ValueError:
            continue
        data = (rec.get("response") or {}).get("data") if isinstance(rec.get("response"), dict) else None
        if rec.get("http_status") != 200 or not isinstance(data, dict) or data.get("componentCode") != code:
            continue
        if best is None or str(rec.get("fetched_at")) > str(best[0].get("fetched_at")):
            best = (rec, f)
    return best


_last_request = [0.0]


def fetch_live(code: str, target: Path) -> str:
    """One read-only GET; replaces ``target/<code>.json`` only with a valid record. Returns a status text."""
    wait = SPACING_S - (time.monotonic() - _last_request[0])
    if wait > 0:
        time.sleep(wait)
    url = DETAIL.format(code=code)
    req = urllib.request.Request(url, headers={"User-Agent": UA, "Accept": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            status, raw = r.status, r.read(5_000_000).decode("utf-8", errors="replace")
    except urllib.error.HTTPError as e:
        status, raw = e.code, ""
    except Exception as e:  # noqa: BLE001 - transport errors are reported, never fatal by themselves
        _last_request[0] = time.monotonic()
        return f"transport error: {type(e).__name__}: {e}"
    _last_request[0] = time.monotonic()
    try:
        body = json.loads(raw)
    except ValueError:
        return f"http {status}, body is not JSON"
    data = body.get("data") if isinstance(body, dict) else None
    if status != 200 or not isinstance(data, dict) or data.get("componentCode") != code:
        return f"http {status}, no record for {code}"
    rec = {"fetched_at": dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat(timespec="seconds"),
           "url": url, "http_status": status, "response": body}
    target.mkdir(parents=True, exist_ok=True)
    (target / f"{code}.json").write_text(json.dumps(rec, ensure_ascii=False, indent=1), encoding="utf-8",
                                         newline="\n")
    return "fetched"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def as_int(v: object) -> int:
    try:
        return int(v)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return 0


def evaluate(code: str, qty: int, rec: dict, path: Path, today: str) -> dict:
    d = rec["response"]["data"]
    stock, presale = as_int(d.get("stockCount")), as_int(d.get("canPresaleNumber"))
    loss, least = as_int(d.get("lossNumber")), as_int(d.get("leastNumber"))
    available = min(stock, presale)
    need = max(BOARDS * qty + loss, least)
    reasons = []
    if available < need:
        reasons.append(f"available {available} < need {need}")
    if str(d.get("isBuyComponent")) != "1":
        reasons.append(f"isBuyComponent = {d.get('isBuyComponent')!r}")
    if d.get("noBuyReason"):
        reasons.append(f"noBuyReason = {d.get('noBuyReason')!r}")
    if d.get("assemblyMode") not in MODES:
        reasons.append(f"assemblyMode = {d.get('assemblyMode')!r}")
    fetched = str(rec.get("fetched_at"))
    stale = fetched[:10] < today
    lib = {"base": "Basic", "expand": "Extended"}.get(str(d.get("componentLibraryType")),
                                                      str(d.get("componentLibraryType")))
    return {
        "jlc_code": code,
        "qty_per_board": qty,
        "mpn": d.get("componentModelEn"),
        "brand": d.get("componentBrandEn"),
        "package": d.get("componentSpecificationEn"),
        "describe": d.get("describe"),
        "library_type": lib,
        "assembly_process": d.get("assemblyProcess"),
        "assembly_mode": d.get("assemblyMode"),
        "stock": stock,
        "can_presale": presale,
        "available": available,
        "loss_number": loss,
        "least_number": least,
        "need_20_boards": need,
        "is_buy_component": d.get("isBuyComponent"),
        "no_buy_reason": d.get("noBuyReason"),
        "i7": "ORDERABLE" if not reasons else "NOT ORDERABLE",
        "i7_reasons": reasons,
        "thin_stock": available < THIN,
        "fetched_at": fetched,
        "stale": stale,
        "url": rec.get("url"),
        "evidence_file": path.as_posix(),
        "evidence_sha256": sha256(path),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--online", action="store_true", help="allow read-only GETs for stale / missing / --refetch codes")
    ap.add_argument("--refetch", default="", help="comma-separated codes to fetch again even if fresh (needs --online)")
    ap.add_argument("--today", default=dt.date.today().isoformat(), help="records older than this date are stale")
    args = ap.parse_args()
    forced = {c.strip() for c in args.refetch.split(",") if c.strip()}
    if forced and not args.online:
        print("--refetch needs --online", file=sys.stderr)
        return 2
    for c in forced:
        if not re.fullmatch(r"C\d+", c):
            print(f"not a JLC code: {c}", file=sys.stderr)
            return 2

    plan = json.loads(PLAN.read_text(encoding="utf-8"))
    dirs = cache_dirs()

    # ---- the code list, from the plan ---------------------------------------------------------------------
    lines = [(b["line"], b["jlc_code"], int(b["qty_per_board"]), b) for b in plan["bom_lines"]]
    extras: list[tuple[str, str, str]] = []          # (group, for, code)
    for a in plan["alternates"]:
        for x in a["alternates"]:
            extras.append(("alternate", a["for"], x["jlc_code"]))
    for name, items in plan["option_parts"].items():
        for x in items:
            extras.append(("option", name, x["jlc_code"]))
    missing_qty = sorted({c for _, _, c in extras if c not in EXTRA_QTY})
    if missing_qty:
        print("no quantity known for", missing_qty, "- add them to EXTRA_QTY", file=sys.stderr)
        return 2

    all_codes = [c for _, c, _, _ in lines] + [c for _, _, c in extras]
    fetch_log = []
    seen: set[str] = set()
    for code in all_codes:
        if code in seen:
            continue
        seen.add(code)
        got = newest_record(code, dirs)
        stale = got is None or str(got[0].get("fetched_at"))[:10] < args.today
        if code in forced or (stale and args.online):
            why = "forced" if code in forced else ("no record" if got is None else "stale")
            res = fetch_live(code, dirs[0])
            fetch_log.append({"jlc_code": code, "why": why, "result": res})
            print(f"  fetch {code:<10} ({why}): {res}")

    # ---- evaluate -----------------------------------------------------------------------------------------
    failures: list[str] = []
    out_lines = []
    for line, code, qty, b in lines:
        got = newest_record(code, dirs)
        if got is None:
            failures.append(f"line {line} {code}: no cached record")
            out_lines.append({"line": line, "jlc_code": code, "qty_per_board": qty, "i7": "NO RECORD",
                              "designators": b["designators"], "comment": b["comment"]})
            continue
        e = evaluate(code, qty, got[0], got[1], args.today)
        if len(b["designators"]) != qty:
            failures.append(f"line {line} {code}: plan lists {len(b['designators'])} designators, qty {qty}")
        pref = "Preferred" in str(b.get("library_type"))
        if pref and e["library_type"] == "Extended":
            # the detail record carries no preferred flag; the plan read it from a search record (named below)
            e["library_type"] = "Extended (Preferred)"
            e["preferred_flag_source"] = b.get("preferred_flag_source")
        if str(b.get("library_type")).split(" ")[0] != e["library_type"].split(" ")[0]:
            failures.append(f"line {line} {code}: library type {e['library_type']} != plan {b.get('library_type')}")
        if (b.get("mpn") or "") != (e["mpn"] or ""):
            failures.append(f"line {line} {code}: MPN {e['mpn']!r} != plan {b.get('mpn')!r}")
        if e["i7"] != "ORDERABLE":
            failures.append(f"line {line} {code}: " + "; ".join(e["i7_reasons"]))
        if e["stale"]:
            failures.append(f"line {line} {code}: record of {e['fetched_at']} is older than {args.today} "
                            "(run with --online)")
        out_lines.append({"line": line, "comment": b["comment"], "designators": b["designators"],
                          "footprint": b["kicad_footprint"], "symbol": b["kicad_symbol"], **e})

    out_extras = []
    extra_fail = []
    for group, for_, code in extras:
        got = newest_record(code, dirs)
        if got is None:
            extra_fail.append(f"{group} {code}: no cached record")
            out_extras.append({"group": group, "for": for_, "jlc_code": code, "i7": "NO RECORD"})
            continue
        e = evaluate(code, EXTRA_QTY[code], got[0], got[1], args.today)
        if e["i7"] != "ORDERABLE":
            extra_fail.append(f"{group} {code}: " + "; ".join(e["i7_reasons"]))
        if e["stale"]:
            extra_fail.append(f"{group} {code}: record of {e['fetched_at']} is older than {args.today}")
        out_extras.append({"group": group, "for": for_, **e})

    ok_lines = [x for x in out_lines if x.get("i7") == "ORDERABLE"]
    doc = {
        "schema": "vtol_carrier_parts_sources/3",
        "what": "JLCPCB parts-library evidence for the rev 3 carrier BOM. Label M = read from a fetched JLCPCB "
                "record on the date in fetched_at; it is NOT a promise for the order day (re-run with --online then).",
        "rule_I7": "min(stockCount, canPresaleNumber) >= max(20 x qty_per_board + lossNumber, leastNumber); "
                   "isBuyComponent == '1'; noBuyReason empty; assemblyMode smtWeld or manualWeld; "
                   f"available < {THIN} = thin stock (reported)",
        "boards": BOARDS,
        "today": args.today,
        "plan": PLAN.as_posix(),
        "plan_sha256": sha256(PLAN),
        "endpoint": DETAIL.format(code="<code>"),
        "access": "read-only GET, no login, no cookie, no cart, >= 2.5 s between requests; nothing was ordered",
        "cache_dirs": [d.as_posix() for d in dirs],
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "counts": {
            "bom_lines": len(out_lines),
            "bom_lines_orderable": len(ok_lines),
            "smt_lines": sum(1 for x in out_lines if x.get("assembly_process") == "SMT"),
            "tht_lines": sum(1 for x in out_lines if x.get("assembly_process") == "THT"),
            "basic_lines": sum(1 for x in out_lines if x.get("library_type") == "Basic"),
            "extended_preferred_lines": sum(1 for x in out_lines if x.get("library_type") == "Extended (Preferred)"),
            "extended_lines": sum(1 for x in out_lines if x.get("library_type") == "Extended"),
            "parts_per_board": sum(x["qty_per_board"] for x in out_lines),
            "thin_stock_lines": [x["jlc_code"] for x in out_lines if x.get("thin_stock")],
        },
        "lines": out_lines,
        "alternates_and_options": out_extras,
        "alternates_and_options_not_orderable": extra_fail,
        "fetched_in_this_run": fetch_log,
    }
    OUT.write_text(json.dumps(doc, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    c = doc["counts"]
    print(f"wrote {OUT.as_posix()}")
    print(f"  BOM lines {c['bom_lines']} ({c['smt_lines']} SMT + {c['tht_lines']} THT), orderable "
          f"{c['bom_lines_orderable']}, parts per board {c['parts_per_board']}; Basic {c['basic_lines']}, "
          f"Extended (Preferred) {c['extended_preferred_lines']}, Extended {c['extended_lines']}; "
          f"thin stock {c['thin_stock_lines']}")
    for x in out_extras:
        print(f"  {x['group']:<9} {x['jlc_code']:<10} {x.get('i7'):<14} qty {x.get('qty_per_board')} "
              f"available {x.get('available')} need {x.get('need_20_boards')} {x.get('assembly_mode')} "
              f"{x.get('fetched_at', '')[11:19]}  for {x['for'][:44]}")
    for m in extra_fail:
        print("  note (not a BOM line):", m)
    for m in failures:
        print("  FAIL:", m)
    print("status:", doc["status"])
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
