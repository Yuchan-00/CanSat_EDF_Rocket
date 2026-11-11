"""Order-day JLC source gate (hard gate, no warning-only mode) -> out/jlc_gate_report.json.

Invariant this module enforces: a BOM line passes only if a JLCPCB component-detail record FETCHED IN THIS RUN says
so (rule I7 of spec/REV3_PLAN.md section 6). Cached records of earlier days are never used for the verdict. The BOM
lines come from spec/carrier_spec.json through fitsets.bom_groups (the same function the BOM / CPL writer uses), the
alternates and option parts from spec/rev3_plan.json.

I7:  available = min(stockCount, canPresaleNumber) >= max(20 x qty_per_board + lossNumber, leastNumber),
     isBuyComponent == "1", noBuyReason empty, assemblyMode in {smtWeld, manualWeld}.
     Extra identity checks per line: MPN == spec MPN, library type == spec, assemblyMode matches the spec assembly
     (SMT -> smtWeld, THT -> manualWeld), componentProductType recorded (0 = Economic and Standard).
     available < 1000 = THIN (listed, not a failure).

A failing main-pair line is looked up in the plan's alternates (listed order). If an alternate passes it is reported
as a swap CANDIDATE only: the swap needs build_spec.py and an identical footprint, which this script does not decide.
The gate status stays FAIL until the spec is changed and the gate passes again.

Network: read-only GET of the public endpoint (no login, no cookie, no cart), >= 2.5 s between requests
(jlc_sources.SPACING_S). Every response is cached under <SCR>/jlc_cache/gate_<YYYYmmddTHHMM>/<code>.json with its
fetch time; the 2026-11-10 snapshot files that spec/parts_sources.json cites are left untouched.

Run:  cd <board>/scripts && B:/Claude/.venv/Scripts/python.exe -X utf8 -B jlc_gate.py [--offline-dir DIR]
      --offline-dir re-evaluates the records of an earlier gate run (no network); the report then says so.
Exit 0 = PASS (main pair orderable), 1 = FAIL / NOT_VERIFIED.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
from pathlib import Path

import fitsets
import jlc_sources as js

ROOT = Path(__file__).resolve().parent.parent
SPEC = ROOT / "spec" / "carrier_spec.json"
PLAN = ROOT / "spec" / "rev3_plan.json"
OUT = ROOT / "out" / "jlc_gate_report.json"
KST = dt.timezone(dt.timedelta(hours=9))
MODE_OF = {"SMT": "smtWeld", "THT": "manualWeld"}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--offline-dir", default="", help="evaluate the records of an earlier gate run (no network)")
    args = ap.parse_args()
    spec = json.loads(SPEC.read_text(encoding="utf-8"))
    plan = json.loads(PLAN.read_text(encoding="utf-8"))
    main_lines = fitsets.bom_groups(spec["parts"], with_mcu=False)
    mcu_lines = [g for g in fitsets.bom_groups(spec["parts"], with_mcu=True)
                 if g["jlc_code"] not in {m["jlc_code"] for m in main_lines}]
    extras: list[tuple[str, str, str]] = []
    for a in plan["alternates"]:
        for x in a["alternates"]:
            extras.append(("alternate", a["for"], x["jlc_code"]))
    for name, items in plan["option_parts"].items():
        for x in items:
            if x["jlc_code"] not in {g["jlc_code"] for g in mcu_lines}:
                extras.append(("option", name, x["jlc_code"]))
    missing_qty = sorted({c for _, _, c in extras if c not in js.EXTRA_QTY})
    if missing_qty:
        print("no quantity known for", missing_qty, file=sys.stderr)
        return 1

    started = dt.datetime.now(KST)
    if args.offline_dir:
        gate_dir = Path(args.offline_dir)
        mode = f"offline re-evaluation of {gate_dir.as_posix()} (no network in this run)"
    else:
        gate_dir = js.DEFAULT_CACHE[0] / f"gate_{started.strftime('%Y%m%dT%H%M')}"
        mode = "online: every code fetched in this run"
    codes = []
    for c in [g["jlc_code"] for g in main_lines + mcu_lines] + [c for _, _, c in extras]:
        if c not in codes:
            codes.append(c)
    fetch_log = []
    if not args.offline_dir:
        for c in codes:
            res = js.fetch_live(c, gate_dir)
            fetch_log.append({"jlc_code": c, "result": res})
            print(f"  fetch {c:<10} {res}", flush=True)
    today = started.date().isoformat() if not args.offline_dir else None

    def judge(code: str, qty: int) -> dict:
        f = gate_dir / f"{code}.json"
        got = js.newest_record(code, [gate_dir])
        if got is None:
            return {"jlc_code": code, "qty_per_board": qty, "i7": "NOT_VERIFIED",
                    "i7_reasons": [f"no valid record fetched in this run ({f.as_posix()})"]}
        e = js.evaluate(code, qty, got[0], got[1], today or str(got[0].get("fetched_at"))[:10])
        e["component_product_type"] = got[0]["response"]["data"].get("componentProductType")
        return e

    failures, lines_out = [], []
    for g in main_lines:
        e = judge(g["jlc_code"], g["qty"])
        reasons = list(e.get("i7_reasons", []))
        if e["i7"] == "ORDERABLE":
            if (g.get("mpn") or "") != (e.get("mpn") or ""):
                reasons.append(f"MPN {e.get('mpn')!r} != spec {g.get('mpn')!r}")
            if str(g.get("library_type")).split(" ")[0] != str(e.get("library_type")).split(" ")[0]:
                reasons.append(f"library type {e.get('library_type')} != spec {g.get('library_type')}")
            if MODE_OF.get(g["assembly"]) != e.get("assembly_mode"):
                reasons.append(f"assemblyMode {e.get('assembly_mode')} does not match spec assembly {g['assembly']}")
        verdict = "PASS" if e["i7"] == "ORDERABLE" and not reasons else (
            "NOT_VERIFIED" if e["i7"] == "NOT_VERIFIED" else "FAIL")
        row = {"line": g["line"], "jlc_code": g["jlc_code"], "comment": g["comment"], "designators": g["designators"],
               "qty_per_board": g["qty"], "assembly": g["assembly"], "verdict": verdict, "reasons": reasons,
               **{k: e.get(k) for k in ("mpn", "library_type", "assembly_mode", "stock", "can_presale", "available",
                                        "need_20_boards", "thin_stock", "component_product_type", "fetched_at",
                                        "evidence_file", "evidence_sha256")}}
        if verdict != "PASS":
            failures.append(f"line {g['line']} {g['jlc_code']} ({','.join(g['designators'])}): {verdict}: "
                            + "; ".join(reasons))
            cands = []
            for a in plan["alternates"]:
                if g["jlc_code"] in a["for"]:
                    for x in a["alternates"]:
                        ea = judge(x["jlc_code"], js.EXTRA_QTY.get(x["jlc_code"], g["qty"]))
                        cands.append({"jlc_code": x["jlc_code"], "i7": ea["i7"], "reasons": ea.get("i7_reasons"),
                                      "note": a.get("note")})
            row["alternates"] = cands or "none named in the plan"
        lines_out.append(row)

    mcu_out = []
    for g in mcu_lines:
        e = judge(g["jlc_code"], g["qty"])
        mcu_out.append({"jlc_code": g["jlc_code"], "designators": g["designators"], "qty_per_board": g["qty"],
                        "verdict": "PASS" if e["i7"] == "ORDERABLE" else e["i7"], "reasons": e.get("i7_reasons"),
                        **{k: e.get(k) for k in ("mpn", "stock", "can_presale", "available", "need_20_boards",
                                                 "thin_stock", "fetched_at", "evidence_file")}})
    extra_out = []
    for group, for_, code in extras:
        e = judge(code, js.EXTRA_QTY[code])
        extra_out.append({"group": group, "for": for_, "jlc_code": code, "verdict": "PASS" if e["i7"] == "ORDERABLE"
                          else e["i7"], "reasons": e.get("i7_reasons"),
                          **{k: e.get(k) for k in ("mpn", "qty_per_board", "stock", "can_presale", "available",
                                                   "need_20_boards", "thin_stock", "fetched_at")}})
    thin = [f"{x['jlc_code']} ({','.join(x['designators'])}) available {x['available']}"
            for x in lines_out + mcu_out if x.get("thin_stock")]
    status = "PASS" if not failures and len(lines_out) == len(main_lines) else "FAIL"
    doc = {
        "tool": "scripts/jlc_gate.py", "mode": mode, "started_at": started.isoformat(timespec="seconds"),
        "rule_I7": "min(stockCount, canPresaleNumber) >= max(20 x qty_per_board + lossNumber, leastNumber); "
                   "isBuyComponent == '1'; noBuyReason empty; assemblyMode smtWeld / manualWeld; plus MPN, library "
                   "type and assembly mode equal to the spec",
        "boards_for_I7": js.BOARDS, "thin_threshold": js.THIN,
        "spec_sha256": js.sha256(SPEC), "plan_sha256": js.sha256(PLAN), "cache_dir": gate_dir.as_posix(),
        "access": "read-only GET of " + js.DETAIL.format(code="<code>") + ", no login / cookie / cart, >= "
                  f"{js.SPACING_S} s between requests; nothing was ordered",
        "status": status, "main_pair_lines": len(lines_out),
        "main_pair_lines_pass": sum(1 for x in lines_out if x["verdict"] == "PASS"),
        "failures": failures, "thin_lines": thin,
        "with_mcu_extra_lines": mcu_out,
        "with_mcu_status": "PASS" if status == "PASS" and all(x["verdict"] == "PASS" for x in mcu_out) else "FAIL",
        "lines": lines_out, "alternates_and_options": extra_out, "fetch_log": fetch_log,
        "note": "label M for the day and time in fetched_at only; JLC's stock changes by the hour - run again on "
                "the order day, directly before uploading. The quote on JLC's site is the final word (NOT_VERIFIED)."}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(doc, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    for x in lines_out:
        print(f"  L{x['line']:>2} {x['jlc_code']:<10} {x['verdict']:<12} avail {x.get('available')!s:>9} need "
              f"{x.get('need_20_boards')!s:>4} {str(x.get('fetched_at'))[11:19]} {'THIN' if x.get('thin_stock') else ''}")
    for x in mcu_out:
        print(f"  with_mcu {x['jlc_code']} {x['verdict']} avail {x.get('available')} need {x.get('need_20_boards')}")
    for m in failures:
        print("  FAIL:", m)
    print("status:", status, "| thin:", thin, "| wrote", OUT.as_posix())
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
