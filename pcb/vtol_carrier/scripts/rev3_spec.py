"""rev 3 side of the electrical spec: plan equality, JLC part sources, rev 3 texts, the Korean summary.

build_spec.py builds parts / nets from the project files; this module ties the result to the settled plan
(spec/rev3_plan.json) and to the JLC part sources (spec/parts_sources.json).

Invariants this module enforces:

* every JLC-fitted part maps to exactly one orderable line of spec/parts_sources.json (rule I7, written by
  scripts/jlc_sources.py) and that line's footprint and designators are the spec's - a part without a line is an
  error, never a silent omission;
* the spec equals the plan part by part: value, symbol, footprint, fit, LCSC code, role of the added parts and the
  complete pin -> net table (a plan "NC" is a declared no-connect); item / part / fit / BOM-line counts are the
  plan's. A difference is an error unless it is one of the declared ones (listed with its reason in the report);
* narrative sections that still describe rev 2c (plug-in 5 V module, ESP-12E, 2 layers, 22 pF load caps) are replaced
  with guarded replacements (the old text must still contain the expected words), and a scan for stale keywords
  outside the history sections must come back empty;
* carrier_spec.json carries no stock number and no fetch time (those are evidence in parts_sources.json), so the
  same inputs give a byte-identical spec.
"""
from __future__ import annotations

import hashlib
import json
import re
from collections import Counter
from pathlib import Path

import fitsets

HERE = Path(__file__).resolve().parent.parent
PLAN_PATH = HERE / "spec" / "rev3_plan.json"
SOURCES_PATH = HERE / "spec" / "parts_sources.json"
PROBE_PATH = HERE / "spec" / "library_probe.json"
PLAN: dict = json.loads(PLAN_PATH.read_text(encoding="utf-8"))
PLAN_SHA256 = hashlib.sha256(PLAN_PATH.read_bytes()).hexdigest()
PROBE: dict = json.loads(PROBE_PATH.read_text(encoding="utf-8"))

SCR = "C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3"
DATASHEETS = {
    "tps61023": f"{SCR}/datasheets/tps61023/tps61023.txt (text extract of the TI TPS61023 datasheet SLVSF14B; sha256 "
                "db5a2ea386e175f6062f84b450919e809d82d583006790b1e0b299994a8f8cad). Pin table: lines 148-160, 'Figure "
                "5-1. DRL Package 6-Pin SOT563 Top View' and the pin functions below it",
    "tlv3012": f"{SCR}/verify/electrical_work/tlv3011.txt (text extract of the TI TLV3011 / TLV3012 datasheet SBOS300C; "
               "sha256 bc930405eef950466c3522ef976e71974eb38b6c2e38bf8aaebf32fc22bc2d05). Pin table: line 154 'Table "
               "5-1. Pin Functions', rows 158-163",
}
#: datasheet pin tables of the two new ICs (pad number -> pin name); used by rev3_checks.py u8-u9
DATASHEET_PINS = {
    "U8": {"part": "TPS61023DRLR (SOT-563, DRL)", "source": "tps61023.txt:148-160 (TI SLVSF14B, Figure 5-1 and the "
           "pin-function rows)", "pins": {"1": "FB", "2": "EN", "3": "VIN", "4": "GND", "5": "SW", "6": "VOUT"}},
    "U9": {"part": "TLV3012BIDBVR (SOT-23-6, DBV)", "source": "tlv3011.txt:154-163 (TI SBOS300C, Table 5-1 Pin "
           "Functions, DBV column)", "pins": {"1": "OUT", "2": "V-", "3": "IN+", "4": "IN-", "5": "REF", "6": "V+"}},
}

PROG_REFS = ("J3", "J19", "J20", "J21", "J22", "J24", "J25", "J26")
MCU_CODE = "C6510"
#: option B (D6) BOM lines: programming headers fitted by JLC. Footprint choice and pad numbering:
#: <scratch>/impl/foundation/optb_pads.json (EasyEDA record vs the on-disk KiCad footprint, F; drawing NOT_VERIFIED)
OPTION_B_LINES = {"C32713283": (41, "pin header 2x3 SMD 2.54 vertical"),
                  "C41417359": (42, "pin header 1x2 SMD 2.54 vertical")}

# ------------------------------------------------------------------------------------------------ LCSC code table
#: (footprint name, first word of the value) -> LCSC code, for parts whose footprint is shared by several values
_BY_FP_VALUE = {
    ("C_0402_1005Metric", "100n"): "C1525", ("C_0402_1005Metric", "27p"): "C1557",
    ("C_0603_1608Metric", "100n"): "C14663", ("C_0603_1608Metric", "1n"): "C1588",
    ("C_0805_2012Metric", "10u"): "C15850", ("C_1206_3216Metric", "22u"): "C12891",
    ("R_0603_1608Metric", "10k"): "C25804", ("R_0603_1608Metric", "1k"): "C21190",
    ("R_0603_1608Metric", "47k"): "C25819", ("R_0603_1608Metric", "220"): "C22962",
    ("R_0603_1608Metric", "4.7k"): "C23162", ("R_0603_1608Metric", "100k"): "C25803",
    ("R_0603_1608Metric", "150k"): "C22807", ("R_0603_1608Metric", "20k"): "C4184",
    ("R_0603_1608Metric", "470k"): "C23178", ("R_0603_1608Metric", "270k"): "C22965",
    ("R_0603_1608Metric", "3.3M"): "C26096", ("R_0603_1608Metric", "1M"): "C22935",
}
#: symbol -> code where the footprint (SOT-23) is shared by several parts
_BY_SYMBOL = {"Transistor_FET:BSS138": "C40912", "Transistor_BJT:MMBT3904": "C20526", "Transistor_FET:AO3401A": "C15127"}
#: footprint name -> code (one part per footprint on this board)
_BY_FP = {
    "CP_EIA-3216-18_Kemet-A": "C11366", "CP_Elec_6.3x5.4": "C2833309", "CP_EIA-3528-21_Kemet-B": "C7196",
    "LED_0603_1608Metric": "C2286", "D_SOD-123": "C8598", "PinSocket_1x06_P2.54mm_Horizontal": "C22436147",
    "JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical": "C161692", "PinHeader_1x03_P2.54mm_Horizontal": "C32713262",
    "Molex_SPOX_5267-02A_1x02_P2.50mm_Vertical": "C5364642", "JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical": "C131337",
    "PinHeader_1x02_P2.54mm_Horizontal": "C32713261", "PinHeader_1x04_P2.54mm_Horizontal": "C32713263",
    "L_0805_2012Metric": "C1046", "SOT-223-3_TabPin2": "C6186", "ESP-12E": "C82891",
    "Crystal_SMD_3225-4Pin_3.2x2.5mm": "C70562", "SOT-563": "C919459", "SOT-23-6": "C20345924",
    "L_Changjiang_FXL0420": "C53426685", "TQFP-64_14x14mm_P0.8mm": MCU_CODE,
    "PinHeader_2x03_P2.54mm_Vertical_SMD": "C32713283", "PinHeader_1x02_P2.54mm_Vertical_SMD_Pin1Right": "C41417359",
}


def jlc_code_for(part: dict) -> str | None:
    """LCSC code of a part from its footprint, value and symbol; None when the table has no entry (caller decides)."""
    name = part["footprint"].split(":", 1)[1]
    words = part["value"].split()
    code = _BY_FP_VALUE.get((name, words[0] if words else ""))
    if code:
        return code
    if name == "SOT-23":
        return _BY_SYMBOL.get(part["symbol"])
    return _BY_FP.get(name)


# ------------------------------------------------------------------------------------------------ part sources (BOM)
def sourcing(parts: list[dict], prog: str, errors: list[str]) -> dict:
    """Give every part its ``bom`` entry and return the spec's ``bom_sources`` section.

    fit jlc          -> the line of parts_sources.json with the part's LCSC code (must be ORDERABLE, same footprint)
    fit jlc_optional -> C6510, only in the ``*_with_mcu`` pair
    fit user / none  -> no line, with the reason
    """
    src = json.loads(SOURCES_PATH.read_text(encoding="utf-8"))
    if src.get("status") != "PASS":
        errors.append(f"{SOURCES_PATH.name}: status {src.get('status')!r} - the JLC source gate must pass first "
                      f"(scripts/jlc_sources.py)")
    if src.get("plan_sha256") != PLAN_SHA256:
        errors.append(f"{SOURCES_PATH.name} was written for another plan file (plan_sha256 {src.get('plan_sha256')} != "
                      f"{PLAN_SHA256}) - run scripts/jlc_sources.py again")
    lines = {ln["jlc_code"]: ln for ln in src["lines"]}
    options = {a["jlc_code"]: a for a in src["alternates_and_options"] if a.get("group") == "option"}
    used: dict[str, list[str]] = {}
    for p in parts:
        fit = fitsets.fit_of(p)
        ref = p["refdes"]
        if fit in ("user", "none"):
            p["bom"] = {"line": None, "jlc_code": None,
                        "why": "fitted by the user after delivery (not in the JLC BOM / CPL)" if fit == "user" else
                               "no part to fit: bare plated holes, DNP option or copper-only feature"}
            continue
        code = jlc_code_for(p)
        if not code:
            errors.append(f"{ref}: fit {fit} but no LCSC code rule for footprint {p['footprint']} / value {p['value']!r}")
            continue
        if fit == "jlc_optional":
            opt = options.get(code)
            if code != MCU_CODE or not opt or opt.get("i7") != "ORDERABLE":
                errors.append(f"{ref}: optional JLC part needs the orderable option {MCU_CODE} in {SOURCES_PATH.name}")
                continue
            p["bom"] = {"line": None, "jlc_code": code, "comment": "ATmega128A-AU TQFP-64", "mpn": opt["mpn"],
                        "library_type": opt["library_type"], "assembly": opt["assembly_process"],
                        "pair": "with_mcu only"}
            continue
        ln = lines.get(code)
        if ln is None and prog == "B" and ref in PROG_REFS and code in OPTION_B_LINES and code in options:
            opt = options[code]
            if opt.get("i7") != "ORDERABLE":
                errors.append(f"{ref}: option B part {code} is not orderable today ({opt.get('i7')})")
                continue
            num, comment = OPTION_B_LINES[code]
            p["bom"] = {"line": num, "jlc_code": code, "comment": comment, "mpn": opt["mpn"],
                        "library_type": opt["library_type"], "assembly": opt["assembly_process"],
                        "pair": "option B (D6)"}
            continue
        if ln is None:
            errors.append(f"{ref}: LCSC code {code} has no line in {SOURCES_PATH.name}")
            continue
        if ln.get("i7") != "ORDERABLE":
            errors.append(f"{ref}: line {ln['line']} {code} is not orderable: {ln.get('i7')} {ln.get('i7_reasons')}")
        if ln["footprint"] != p["footprint"]:
            errors.append(f"{ref}: footprint {p['footprint']} != {ln['footprint']} of line {ln['line']} ({code})")
        if ln["assembly_process"] not in ("SMT", "THT"):
            errors.append(f"{ref}: line {ln['line']} has assembly process {ln['assembly_process']!r}")
        p["bom"] = {"line": ln["line"], "jlc_code": code, "comment": ln["comment"], "mpn": ln["mpn"],
                    "library_type": ln["library_type"], "assembly": ln["assembly_process"]}
        used.setdefault(code, []).append(ref)
    for code, ln in lines.items():
        have = sorted(used.get(code, []), key=fitsets.natural)
        want = sorted(ln["designators"], key=fitsets.natural)
        if have != want:
            errors.append(f"BOM line {ln['line']} {code}: designators in the spec {have} != {SOURCES_PATH.name} {want}")
    if errors:
        return {}

    def rows(with_mcu: bool) -> list[dict]:
        out = []
        for g in fitsets.bom_groups(parts, with_mcu):
            out.append({"line": g["line"], "value": g["comment"], "comment": g["comment"], "footprint": g["footprint"],
                        "footprint_name": g["footprint_name"], "source": "JLCPCB assembly library (LCSC)",
                        "part_number": g["jlc_code"], "mpn": g["mpn"], "jlc_type": g["library_type"],
                        "assembly": "JLC SMT assembly" if g["assembly"] == "SMT" else
                                    "JLC THT assembly (hand soldering by JLC, manualWeld)",
                        "process": g["assembly"], "refs": g["designators"], "dnp_refs": [], "qty": g["qty"]})
        return out

    main, mcu = rows(False), rows(True)
    extra = [r for r in mcu if r["part_number"] not in {m["part_number"] for m in main}]
    return {
        "rule": "rev 3: every part with fit jlc is one line of the JLC BOM; stock, prices and fetch times are evidence "
                "in spec/parts_sources.json (scripts/jlc_sources.py, rule I7: " + PLAN["interpretation"][6] + ")",
        "parts_sources": "spec/parts_sources.json",
        "jlc_bom_columns": ["Comment", "Designator", "Footprint", "LCSC Part #"],
        "prog_headers": prog,
        "board_lines": main,
        "with_mcu_extra_lines": extra,
        "counts": {"main": fitsets.summary(parts), "with_mcu": fitsets.summary(parts, with_mcu=True)},
        "not_in_the_jlc_bom": {f: refs for f, refs in fitsets.fit_sets(parts).items() if f != "jlc"},
        "order_day_gate": PLAN["order_day_gate"],
        "alternates": [{"for": a["for"], "codes": [x["jlc_code"] for x in a["alternates"]]} for a in PLAN["alternates"]],
        "user_fitted": PLAN["user_fitted"],
        "off_board_purchase": PLAN["off_board_purchase"],
        "label": "F (plan + parts_sources.json); nothing is ordered by a tool - the user uploads and pays",
    }


# ------------------------------------------------------------------------------------------------ plan equality
_POPULATE_BY = {"jlc": "JLC", "jlc_optional": "user", "user": "user", "none": "none"}


def _member(member: str) -> tuple[str, str]:
    """'U6.VO(2)' -> ('U6', '2'); 'C45.1' -> ('C45', '1'); 'J7.MP' -> ('J7', 'MP')."""
    ref, _, rest = member.partition(".")
    m = re.search(r"\(([^()]+)\)$", rest)
    return ref, (m.group(1) if m else rest)


def pin_tables(parts: list[dict], nets: list[dict], ncl: list[dict]) -> dict[str, dict[str, str]]:
    """{refdes: {pin number: net name | 'NC' | 'UNASSIGNED'}} for every pin of every part's library symbol."""
    byref = {p["refdes"]: p for p in parts}
    pin_net: dict[tuple[str, str], str] = {}
    for n in nets:
        for m in n["members"]:
            pin_net[_member(m)] = n["name"]
    nc: set[tuple[str, str]] = set()
    for e in ncl:
        ref, _, fn = e["pin"].partition(".")
        pins = PROBE["symbols"][byref[ref]["symbol"]]["pins"]
        nums = [q[0] for q in pins if q[1] == fn] or [q[0] for q in pins if q[0] == fn]
        nc.update((ref, n) for n in nums)
    out: dict[str, dict[str, str]] = {}
    for ref, p in byref.items():
        out[ref] = {}
        for q in PROBE["symbols"][p["symbol"]]["pins"]:
            key = (ref, q[0])
            out[ref][q[0]] = pin_net.get(key) or ("NC" if key in nc else "UNASSIGNED")
    return out


def plan_check(parts: list[dict], nets: list[dict], ncl: list[dict], prog: str, errors: list[str]) -> dict:
    """Compare the built spec with spec/rev3_plan.json; undeclared differences are appended to ``errors``."""
    byref = {p["refdes"]: p for p in parts}
    tables = pin_tables(parts, nets, ncl)
    diffs: list[str] = []
    declared: list[str] = []
    pins_compared = 0

    def diff(ref: str, field: str, have, want, why: str | None = None) -> None:
        text = f"{ref}.{field}: spec {have!r} != plan {want!r}"
        if why:
            declared.append(f"{text} - {why}")
        else:
            diffs.append(text)

    optb = "option B (VTOL_PROG_HEADERS=B, plan D6 / conditional_items): header fitted by JLC outside the tie band"
    plan_refs = set()
    for c in PLAN["changes"]:
        ref, r3 = c["refdes"], c["rev3"]
        plan_refs.add(ref)
        if c["action"] == "remove":
            if ref in byref:
                diffs.append(f"{ref}: removed in the plan but present in the spec")
            if r3 is not None:
                diffs.append(f"{ref}: plan action remove with a rev3 entry")
            continue
        p = byref.get(ref)
        if p is None:
            diffs.append(f"{ref}: in the plan ({c['action']}) but not in the spec")
            continue
        b_hdr = prog == "B" and ref in PROG_REFS
        if p["value"] != r3["value"]:
            if ref == "J3" and p["value"] == r3["value"].replace("ESP-12E", "ESP-12F"):
                diff(ref, "value", p["value"], r3["value"], "text only: the plan's J3 value still names the ESP-12E, the "
                     "fitted module is the ESP-12F (plan changes[U7])")
            else:
                diff(ref, "value", p["value"], r3["value"])
        if p["symbol"] != r3["symbol"]:
            diff(ref, "symbol", p["symbol"], r3["symbol"])
        if p["footprint"] != r3["footprint"]:
            diff(ref, "footprint", p["footprint"], r3["footprint"], optb if b_hdr else None)
        if p["fit"] != r3["fit"]:
            diff(ref, "fit", p["fit"], r3["fit"], optb if b_hdr else None)
        if _POPULATE_BY[p["fit"]] != r3["populate_by"]:
            diff(ref, "populate_by", _POPULATE_BY[p["fit"]], r3["populate_by"], optb if b_hdr else None)
        bom = p.get("bom") or {}
        code = None if bom.get("pair") == "with_mcu only" else bom.get("jlc_code")
        if code != r3["jlc_code"]:
            diff(ref, "jlc_code", code, r3["jlc_code"], optb if b_hdr else None)
        if p["fit"] == "jlc" and bom.get("mpn") != r3["mpn"]:
            diff(ref, "mpn", bom.get("mpn"), r3["mpn"], optb if b_hdr else None)
        if "role" in r3 and p["role"] != r3["role"]:
            diff(ref, "role", p["role"], r3["role"])
        have, want = tables[ref], r3["pins"]
        pins_compared += len(set(have) | set(want))
        for num in sorted(set(have) | set(want), key=fitsets.natural):
            if have.get(num) != want.get(num):
                diff(ref, f"pin {num}", have.get(num), want.get(num))
    for ref in sorted(set(byref) - plan_refs, key=fitsets.natural):
        diffs.append(f"{ref}: in the spec but not in the plan")

    # counts (plan sections 3 / 6): 175 items, 174 parts, fit sets, BOM lines
    actions = dict(sorted(Counter(c["action"] for c in PLAN["changes"]).items()))
    want_actions = {"add": 13, "keep": 140, "keep-dnp": 10, "remove": 1, "replace": 11}
    if actions != want_actions:
        diffs.append(f"plan actions {actions} != {want_actions}")
    summ = fitsets.summary(parts)
    want_fit = dict(Counter(c["rev3"]["fit"] for c in PLAN["changes"] if c["rev3"]))
    want_lines = {"total": len(PLAN["bom_lines"]),
                  "smt": sum(1 for b in PLAN["bom_lines"] if b["assembly_process"] == "SMT"),
                  "tht": sum(1 for b in PLAN["bom_lines"] if b["assembly_process"] == "THT")}
    want_parts = sum(b["qty_per_board"] for b in PLAN["bom_lines"])
    have_lines = {"total": summ["bom_lines"], "smt": summ["bom_lines_smt"], "tht": summ["bom_lines_tht"]}
    count_why = optb + ": 8 more JLC-fitted parts, 2 more BOM lines" if prog == "B" else None
    if summ["parts"] != len(PLAN["changes"]) - 1:
        diffs.append(f"parts {summ['parts']} != plan {len(PLAN['changes']) - 1}")
    for name, have, want in (("fit counts", {k: v for k, v in summ["fit_counts"].items() if v}, want_fit),
                             ("JLC-fitted parts", summ["jlc_fitted_parts"], want_parts),
                             ("JLC BOM lines", have_lines, want_lines)):
        if have != want:
            diff("counts", name, have, want, count_why)
    if (want_parts, want_lines) != (157, {"total": 40, "smt": 34, "tht": 6}):
        diffs.append(f"plan BOM {want_parts} parts / {want_lines} != 157 parts / 40 lines (34 SMT + 6 THT)")

    # BOM lines one by one
    groups = {g["jlc_code"]: g for g in fitsets.bom_groups(parts)}
    for b in PLAN["bom_lines"]:
        g = groups.pop(b["jlc_code"], None)
        if g is None:
            diffs.append(f"BOM line {b['line']} {b['jlc_code']}: in the plan but not in the spec")
            continue
        for field, have, want in (("line", g["line"], b["line"]), ("comment", g["comment"], b["comment"]),
                                  ("footprint", g["footprint_name"], b["footprint"]),
                                  ("kicad footprint", g["footprint"], b["kicad_footprint"]),
                                  ("designators", g["designators"], sorted(b["designators"], key=fitsets.natural)),
                                  ("qty", g["qty"], b["qty_per_board"]), ("process", g["assembly"], b["assembly_process"]),
                                  ("mpn", g["mpn"], b["mpn"]), ("library type", g["library_type"], b["library_type"])):
            if have != want:
                diff(f"BOM line {b['line']} {b['jlc_code']}", field, have, want)
    for code, g in sorted(groups.items()):
        diff(f"BOM line {g['line']} {code}", "designators", g["designators"], None, optb if prog == "B" else None)

    # the five new nets, member by member; the J6 no-connects are gone
    new_nets = []
    netmap = {n["name"]: sorted({".".join(_member(m)) for m in n["members"]}, key=fitsets.natural) for n in nets}
    for nn in PLAN["new_nets"]:
        have, want = netmap.get(nn["name"]), sorted(nn["members"], key=fitsets.natural)
        new_nets.append({"name": nn["name"], "members": have, "equal_to_plan": have == want})
        if have != want:
            diffs.append(f"new net {nn['name']}: spec {have} != plan {want}")
    for gone in PLAN["removed_no_connects"]:
        if any(e["pin"] == gone for e in ncl):
            diffs.append(f"no-connect {gone} must be gone")
    for name in PLAN["removed_nets"]:
        if name in netmap:
            diffs.append(f"net {name} must be gone")

    errors.extend(f"plan check: {d}" for d in diffs)
    return {
        "tool": "scripts/rev3_spec.py plan_check (run by scripts/build_spec.py)",
        "plan": "spec/rev3_plan.json", "plan_sha256": PLAN_SHA256,
        "result": "PASS" if not diffs else "FAIL",
        "items_compared": len(PLAN["changes"]), "plan_actions": actions, "parts": summ["parts"],
        "pins_compared": pins_compared,
        "fit_counts": summ["fit_counts"], "jlc_fitted_parts": summ["jlc_fitted_parts"], "jlc_bom_lines": have_lines,
        "plan_counts": {"fit": want_fit, "jlc_fitted_parts": want_parts, "jlc_bom_lines": want_lines},
        "new_nets": new_nets,
        "compared_fields": ["value", "symbol", "footprint", "fit", "populate_by", "jlc_code", "mpn (JLC-fitted parts)",
                            "role (added parts)", "every pin -> net / NC", "BOM line number, comment, footprint, "
                            "designators, quantity, process, mpn, library type", "new nets", "removed no-connects"],
        "differences": diffs,
        "declared_differences": declared,
    }


# ------------------------------------------------------------------------------------------------ rev 3 texts
#: top-level sections that are history or verbatim copies of the plan: not rewritten, not scanned
HISTORY_KEYS = ("changes_from_rev1", "rev_2c_changes", "radio_decision_2026_11_05", "removed_5th_mcu",
                "battery_spot_A_assumed", "rev3_plan_extract", "checks")
#: words that describe rev 2c; outside HISTORY_KEYS they may appear only inside one of ALLOWED_PHRASES
STALE = re.compile(r"Pololu|S9V11|ESP-12E|\bJ6\b|\bJ13\b|22 pF|\b2 layers\b|2-layer|two_layer|four_layer_fallback|"
                   r"1\.34 A|1\.3 A peak|boost module|5 V module|either layer|both layers|B\.Cu mostly solid GND")
ALLOWED_PHRASES = (
    "RF_Module:ESP-12E", "ESP-12E.kicad_mod", "same 22 pins as ESP-12E", "'ESP-12E 직접 납땜'",
    "pin-compatible with the ESP-12E", "a soldered ESP-12E on 2026-11-05", "ESP-12E and 1S + boost 2026-11-05",
    "replaces the Pololu S9V11F5S6CMA module on J6", "replaces the Pololu module", "instead of the Pololu module on J6",
    "the DNP release header J13 is removed (rev 2c", "removes the unpopulated J13 header", "gives 2 x 22 pF",
    "'[정품] ESP8266 ESP-12E'", "(rev 2c: Pololu S9V11F5S6CMA plug-in module on J6)",
    "assumed equal to the ESP-12E", "rev 2b used 22 pF", "section 6 (2 x 22 pF)",
    "rev 2c removed the unpopulated J13 header",
)
_SUBS = (("J8-J13 pin 3", "J8-J11 pin 3"), ("J8-J11 (+ J13 DNP) pin 2", "J8-J11 pin 2"), ("2 x 22 pF", "2 x 27 pF"),
         ("CL 16-18 pF A", "CL 18 pF, LCSC C70562"), ("with 22 pF caps", "with 27 pF load caps"),
         ("ESP-12E", "ESP-12F"))


def _sub(text: str) -> str:
    """ESP-12E -> ESP-12F etc. in running text; the allowed phrases (library ids, quotes) are left alone."""
    keep: list[str] = []

    def hide(m: re.Match) -> str:
        keep.append(m.group(0))
        return f"\x00{len(keep) - 1}\x00"

    pat = "|".join(re.escape(a) for a in sorted(ALLOWED_PHRASES, key=len, reverse=True))
    text = re.sub(pat, hide, text)
    for old, new in _SUBS:
        text = text.replace(old, new)
    return re.sub("\x00(\\d+)\x00", lambda m: keep[int(m.group(1))], text)


def _walk_sub(node):
    if isinstance(node, str):
        return _sub(node)
    if isinstance(node, list):
        return [_walk_sub(v) for v in node]
    if isinstance(node, dict):
        return {_sub(k): _walk_sub(v) for k, v in node.items()}
    return node


def _stale(node, path: str, out: list[str]) -> None:
    if isinstance(node, str):
        text = node
        for a in ALLOWED_PHRASES:
            text = text.replace(a, "")
        for m in STALE.finditer(text):
            out.append(f"{path}: {m.group(0)!r} in {node[:160]!r}")
    elif isinstance(node, list):
        for i, v in enumerate(node):
            _stale(v, f"{path}/{i}", out)
    elif isinstance(node, dict):
        for k, v in node.items():
            _stale(k, f"{path}/<key>", out)
            _stale(v, f"{path}/{k}", out)


def apply_rev3_text(spec: dict, prog: str, errors: list[str]) -> None:
    """Rewrite the narrative sections that described rev 2c; then scan for stale words (leftovers are errors)."""
    pb = PLAN["power_block"]
    uv = pb["uvlo"]
    peak = pb["source_path"]["cases"]["peak at 3.0 V pack"]
    typ = pb["source_path"]["cases"]["typ at 3.0 V pack"]
    plan3 = "spec/rev3_plan.json"

    def put(path: tuple, new, needle: str) -> None:
        """Replace the value at ``path``; the old value (as JSON text) must contain ``needle``."""
        node = spec
        try:
            for k in path[:-1]:
                node = node[k]
            old = node[path[-1]]
        except (KeyError, IndexError, TypeError):
            errors.append(f"rev 3 text: path {'/'.join(map(str, path))} not found")
            return
        if needle not in json.dumps(old, ensure_ascii=False):
            errors.append(f"rev 3 text: {'/'.join(map(str, path))} no longer contains {needle!r} - check the rewrite")
            return
        node[path[-1]] = new(old) if callable(new) else new

    def swap(old_text: str, new_text: str):
        def f(old: str) -> str:
            if old.count(old_text) != 1:
                errors.append(f"rev 3 text: expected exactly one {old_text[:60]!r}")
            return old.replace(old_text, new_text)
        return f

    # ---- board
    put(("board", "layers"), 4, "2")
    put(("board", "copper"), "1 oz outer / 0.5 oz inner (A: JLC standard 4-layer stack-up; D16)", "1 oz")
    put(("board", "stackup_note"),
        f"4 layers ({plan3} invariants (d)): F.Cu signals + every fitted part / In1.Cu solid GND plane / In2.Cu +5V "
        "plane / B.Cu signals + GND fill; JLC standard 4-layer stack-up, no impedance control (order_options, D16)",
        "user chose")
    put(("board", "layer_plan"), {
        "decision": f"rev 3 keeps the 4-layer board that rev 2c was delivered on ({plan3} invariants (d)); the rev 2c "
                    "spec text still said 'layers: 2' - that inconsistency is not copied (plan open_risks)",
        "stackup": ["F.Cu: signals + all parts (every JLC-fitted part is on the top side)",
                    "In1.Cu: solid GND plane (no tracks)", "In2.Cu: +5V plane", "B.Cu: signals + GND fill (no parts)"],
        "board": "JLCPCB standard 4-layer stack-up, 1.6 mm, no impedance control; surface finish ENIG (D16); name the "
                 "stack-up in the order (A: not fetched as a document)",
        "rules": "track 0.2 mm, clearance 0.13 mm, via 0.3 / 0.6 mm, edge 0.3 mm, hole-to-hole 0.5 mm - the rev 2c rule "
                 f"file unchanged ({plan3} invariants (f); its hash is compared by the routing step); through-hole vias "
                 "only, no blind / buried vias",
        "keepouts": "the ESP-12F antenna keep-out applies to ALL four layers (no inner plane under the antenna)"},
        "four_layer_fallback")
    put(("board", "keepouts", 1), lambda old: {
        **old, "what": "ESP-12F antenna",
        "rule": "footprint keep-out (all copper layers, 16.24 x 7.32 mm) at the +y edge + a rule area extending 4 mm "
                "beyond both module sides along the edge: no copper on any of the four layers, no parts, no GND "
                "stitching vias (A; Espressif / Ai-Thinker ask for the antenna to overhang the board or for a "
                "copper-free area, ideally >= 15 mm clear of metal - check)"}, "ESP-12E antenna")
    put(("board", "keepouts", 4), lambda old: {
        **old,
        "rule": "no parts under: OpenLog (19 x 16, J5, A), BMP280 module (J2, A), GY-521 (21 x 16 + support pad, J1, "
                "A) - measure the bought modules (D15). The outline of the rev 2c plug-in converter module in the +x / -y corner is "
                "gone: that corner holds the boost converter block (U8, L5, U9 and their passives)",
        "source": f"rev-1 independent review 2026-11-05 MF-1, MF-9; {plan3} placement_notes"}, "S9V11F5S6CMA")
    put(("board", "edges", 7, "connectors", 6), "U8 / L5 / U9 5 V boost converter block (+x / -y corner, no connector)",
        "J6 5 V module")

    # ---- power
    put(("power", "decisions", 1),
        lambda old: old + f" (rev 3: on-board TPS61023 boost converter U8 with the TLV3012B under-voltage lock-out U9 "
                          f"instead of a plug-in module, {plan3} user_decisions D1 = A)", "5 V BOOST stage")
    put(("power", "tree", 0),
        "J14 logic battery (1S, 3.0-4.2 V) -> Q10 AO3401A reverse-polarity P-FET -> VBAT_LOGIC -> J15 switch lead -> "
        "VBAT_LOGIC_SW -> C41 / C42 (2 x 22 uF) -> L5 1.0 uH -> BOOST_SW -> U8 TPS61023 (synchronous boost, 1 MHz, "
        "output set to 5.06 V by R63 / R64) -> +5V (C43 + C57 at the IC). U8 is enabled by U9 (TLV3012B comparator "
        "with a 1.242 V reference on the R65 / R66 / R67 divider): on above 3.59 V, off below 3.13 V nominal (E)",
        "J6 pin 3 VIN")
    put(("power", "tree", 2),
        "+5V -> U6 AMS1117-3.3 -> +3V3 -> ESP-12F VCC (100 uF + 10 uF + 100 nF at the pin), BMP280 VCC, GPS VCC (JP1 "
        "default), BSS138 gates + LV pull-ups, ESP strap pull-ups", "ESP-12E VCC")
    put(("power", "tree", 4), swap("J8-J11 (+ J13 DNP) pin 2", "J8-J11 pin 2"), "J13 DNP")
    le = ("power", "load_estimate")
    put(le + ("rail_3V3_mA",), lambda old: {
        "ESP-12F (Wi-Fi associated, UART bridge: ~80 avg, A; TX bursts up to 500 mA, F plan power_block."
        "load_assumption: C82891 datasheet text)": [80, 500], **{k: v for k, v in old.items() if "ESP-12E" not in k}},
        "[80, 400]")
    put(le + ("rail_3V3_total_mA",), {"typical": 128, "peak": 548}, "448")
    put(le + ("rail_5V_mA",), lambda old: {**old, "3.3 V rail (above)": [128, 548]}, "[128, 448]")
    put(le + ("rail_5V_total_mA",), {"typical": 273, "peak": 783}, "683")
    put(le + ("boost",), {
        "converter": "U8 TPS61023DRLR (LCSC C919459) + L5 1.0 uH FXL0420-1R0-MA1 (C53426685): synchronous boost, "
                     "output 5.06 V nominal (4.84 .. 5.28 V over tolerances, E), output disconnected from the input in "
                     "shutdown (F tps61023.txt:19, :559)",
        "numbers": f"{plan3} power_block (E: datasheet calculations by the plan's verify_power.py, nothing measured); "
                   "copied in rev3_plan_extract.power_block",
        "load": pb["load_assumption"],
        "output_current_capability_A": {"at_3.0V_at_the_IC_ILIM_min": pb["iout_capability_at_IC_input_3p0V_ILIM_min"]["Iout_CL_A"],
                                        "margin_vs_peak": pb["iout_capability_at_IC_input_3p0V_ILIM_min"]["margin_vs_peak_783mA"]},
        "input_current_A": {"typical_at_3.0V": typ["Iin_A"], "peak_at_3.0V": peak["Iin_A"],
                            "peak_at_3.3V": pb["source_path"]["cases"]["peak at 3.3 V pack"]["Iin_A"],
                            "peak_at_3.7V": pb["source_path"]["cases"]["peak at 3.7 V pack"]["Iin_A"]},
        "source_path_ohm_E": pb["source_path"]["sum_ohm"],
        "voltage_at_the_IC_at_3.0V_peak_V": peak["V_at_IC_V"],
        "inductor_peak_current_A": {"at_3.0V_peak_load": peak["IL_peak_A"],
                                    "in_current_limit_typ": pb["inductor"]["IL_peak_in_current_limit_typ_A"],
                                    "Isat_typ_A": pb["inductor"]["Isat_typ_max_A"][0]}}, "S9V11F5S6CMA")
    put(le + ("ams1117_dissipation_W",), {
        "typical": 0.22, "peak_burst": 0.93,
        "formula": "(5.0 - 3.3) x I_3V3 (E); SOT-223 on its pour: ~+20 C at the typical load (E, theta_JA ~90 C/W A). "
                   "Headroom: +5V minimum 4.844 V minus the AMS1117 dropout of 1.3 V max leaves 0.244 V above 3.3 V (E, "
                   "plan open_risks); the +3V3 minimum during a Wi-Fi burst is NOT_VERIFIED"}, "0.76")
    put(le + ("q10_drop_V",), f"<= {0.085 * peak['Iin_A']:.3f} V at {peak['Iin_A']} A (85 mOhm at Vgs -2.5 V, A); the "
        f"whole source path is estimated at {pb['source_path']['sum_ohm']} Ohm (E, plan power_block.source_path)", "1.34 A")
    put(le + ("c_rate",), f"peak {peak['Iin_A']} A = {peak['Iin_A'] / 4.2:.2f} C of 4200 mAh (E)", "1.34 A")
    put(("power", "over_discharge"), {
        "risk": "a boost converter keeps running far below the safe end of a Li-Po cell: without a cut-off the cell "
                "can be pulled below it (cell damage) - and when any cut-off acts in flight ALL four chips reset (loss "
                "of control)",
        "protection_in_this_design": [
            f"HARDWARE under-voltage lock-out: U9 TLV3012B (1.242 V reference, push-pull output) drives the EN pin of "
            f"U8. Thresholds from R65 470k / R66 270k / R67 3.3M (E, 256 tolerance corners): off {uv['V_off_nominal']:.2f} V "
            f"nominal ({uv['V_off_worst'][0]:.3f} .. {uv['V_off_worst'][1]:.3f} V), on {uv['V_on_nominal']:.2f} V nominal "
            f"({uv['V_on_worst'][0]:.3f} .. {uv['V_on_worst'][1]:.3f} V). Fixed by the resistors - nothing to adjust. It is "
            f"the LAST line, not a flight-time alarm",
            "the pack's PCM (A, assumed from the 'P' suffix: cut-off ~2.5-3.0 V) - MEASURE it with a bench load before "
            "relying on it",
            "logic-battery monitor on FCU PF3 POPULATED (47k / 47k, 2.10 V at 4.2 V) - firmware does not read it yet "
            "(follow-up: telemetry + refuse to ARM below ~3.6 V / land below ~3.4 V, A thresholds)",
            "J15 switch after Q10: with the switch off nothing on the board drains the pack (converter, comparator and "
            "dividers are after the switch)",
            "procedure (until the firmware alarm exists): measure the pack before each flight, >= 3.8 V (E: >= ~50 % "
            "charge)"],
        "after_cut_off": f"the board still draws {pb['drain_after_cutoff_uA']['new_block_total_typ']} uA typ / "
                         f"{pb['drain_after_cutoff_uA']['new_block_total_max']} uA max in the new block plus 34 .. 62 uA "
                         "through the R15 / R16 divider (E): the user must switch off - the cut-off is not a storage "
                         "switch",
        "hysteresis_vs_rebound": f"hysteresis {uv['hysteresis_worst'][0]:.3f} V minimum against a rebound of "
                                 f"{uv['rebound_V_at_path_resistance']['peak_load']:.3f} V at peak load with the estimated "
                                 f"{uv['rebound_V_at_path_resistance']['ohm_E']} Ohm source path: margin "
                                 f"{uv['hysteresis_min_minus_rebound_peak_V'] * 1000:.0f} mV (E). Above "
                                 f"{uv['path_resistance_accept_limit_ohm']} Ohm the supply can cycle on / off at the end "
                                 "of discharge under peak load - measure the source resistance on the first board",
        "sag_check": "the cut-off acts on the LOADED voltage (sense filter 16.3 ms): with a sustained peak load it can "
                     f"trip when the resting pack voltage is below "
                     f"{uv['pack_rest_voltage_below_which_sustained_load_trips_worst_case_V']['peak_load']:.2f} V (worst-case "
                     f"threshold; {uv['pack_rest_voltage_below_which_sustained_load_trips_worst_case_V']['typ_load']:.2f} V "
                     "at typical load) (E); the pre-flight check (>= 3.8 V) keeps ~0.2 V above that",
        "switch_on": f"switch-on threshold up to {uv['V_on_worst'][1]:.3f} V: a pack resting below that may not start "
                     "the board (E)",
        "label": f"E ({plan3} power_block.uvlo; calculated, nothing built or measured)"}, "S9V11F5S6CMA")
    put(("power", "phantom_power_through_6S_sense", "estimate"),
        lambda old: swap("Back-feed into the Pololu VOUT while it is unpowered: not specified on the vendor page (A: "
                         "sub-mA through its output stage is harmless) - follow the procedure",
                         "Back-feed into the VOUT pin of U8 while its input is at 0 V: not specified in the datasheet "
                         "(A: sub-mA is harmless) - follow the procedure")(
            swap("the 5 V module (synchronous buck-boost) may pass some of it to its input side (A)",
                 "the boost converter U8 is off then (EN held low by R68) and disconnects its output from the input "
                 "side in shutdown (F tps61023.txt:19, :559; with VIN at 0 V: A)")(old)), "Pololu")

    # ---- radio / support / layout texts
    put(("esp12e", "part"),
        "ESP-12F (Ai-Thinker ESP8266MOD: ESP8266EX + 4 MB SPI flash + PCB antenna, LCSC C82891), soldered by JLC "
        "(castellated pads); pin-compatible with the ESP-12E the user named on 2026-11-05", "ESP-12E")
    put(("esp12e", "antenna", "keepout"),
        "footprint keep-out (all copper layers) + rule area 4 mm wider on both sides, no copper on any of the four "
        "layers, no parts (A)", "either layer")
    put(("layout_rules", "rules", 3),
        "GND: In1.Cu is a solid GND plane without tracks, B.Cu carries signals + a GND fill, stitching vias (none in "
        "the antenna keep-out), no thin GND tracks, nlink SPI over unbroken GND, GND_PWR trunk along the board edge",
        "B.Cu mostly solid GND")
    put(("layout_rules", "rules", 5),
        "boost path (" + plan3 + " routing_notes): VBAT_LOGIC_RAW / VBAT_LOGIC / VBAT_LOGIC_SW from J14 through Q10 and "
        "J15 to L5 as a pour or >= 2.0 mm wide (target <= 0.035 Ohm, about 1.75 A average at the 3.0 V peak, E); "
        "BOOST_SW on F.Cu only, no via, as short as the two pads allow; +5V from the C57 / C43 pads into In2 with >= 3 "
        "vias; U8 pin 4 and the capacitor grounds into In1 with >= 3 vias, In1 solid under the block; BOOST_FB sensed at "
        "the output capacitor, away from BOOST_SW / L5; no signal trace under L5 on F.Cu", "J6 pin 3")
    put(("layout_rules", "rules", 6),
        "ESP-12F: VCC caps at pin 8; antenna keep-out free of copper on all four layers and of parts; the 1k series "
        "resistor next to the ESP RXD0 pin", "both layers")
    put(("user_parts", "radio"),
        "ESP-12F (ESP8266MOD, LCSC C82891) soldered on the carrier by JLC - the user chose a soldered ESP-12E on "
        "2026-11-05; rev 3 fits the pin-compatible ESP-12F because it is the JLC library part (plan I3). It replaced "
        "the PLM100 LoRa module of rev 1", "ESP-12E")
    put(("conflicts", 0), swap("ESP-12E and 1S + boost 2026-11-05)",
                               "ESP-12E and 1S + boost 2026-11-05; rev 3 fits the pin-compatible ESP-12F)"), "ESP-12E and 1S")

    # ---- assumptions (same order as rev 2c; entries that named the plug-in module / 2 layers are rewritten)
    asm = ("assumptions",)
    put(asm + (1,),
        "5 V converter U8 TPS61023DRLR + L5 + U9 TLV3012B: pin tables from the datasheet text extracts (sources "
        "tps61023.txt / tlv3011.txt); Vout 5.06 V nominal, 1.8 A output capability at 3.0 V at the IC (ILIM min), "
        "cut-off off 3.13 V / on 3.59 V nominal - all E (plan power_block: calculated, nothing measured)", "S9V11F5S6CMA")
    put(asm + (3,),
        "ESP-12F: VCC 3.0-3.6 V, ~80 mA average (A) / bursts up to 500 mA (F plan power_block.load_assumption), I/O "
        "3.3 V only (not 5 V tolerant), straps EN / RST / GPIO0 / GPIO2 high, GPIO15 low, boot log 74880 baud; RF "
        "behaviour assumed equal to the ESP-12E in the same keep-out (A, plan open_risks)", "ESP-12E: VCC")
    put(asm + (11,),
        "parts: every JLC-fitted part has an orderable line in spec/parts_sources.json (rule I7, scripts/"
        "jlc_sources.py); pad numbering of the THT headers / sockets and the CPL rotation of polarised parts against "
        "the delivered parts are NOT_VERIFIED ('Confirm Parts Placement' in the order)", "match the KiCad footprints")
    put(asm + (12,), swap("for the 1.34 A boost-input peak",
                          "for the 1.75 A boost-input current at the 3.0 V peak (E, plan power_block) - check the "
                          "switch's rating"), "1.34 A")
    put(asm + (15,),
        "4 layers (F.Cu / In1 GND / In2 +5V / B.Cu), JLC standard stack-up; corner radius 1 mm; tie band x -3..+3 mm "
        "free of fitted parts", "2 layers")

    # ---- user decisions: the plan's D1..D17 first, then the still-open rev 2c items
    carried = [d for d in spec["user_decisions"]
               if not any(w in d for w in ("5 V module", "2 layers first", "parts sources:"))]
    if len(spec["user_decisions"]) - len(carried) != 4:
        errors.append("rev 3 text: expected 4 superseded rev 2c user decisions (5 V module x 2, 2 layers, parts sources)")
    spec["user_decisions"] = list(PLAN["user_decisions"])
    spec["user_decisions_carried_from_rev2c"] = carried
    spec["open_risks"] = list(PLAN["open_risks"])
    spec["rev3_plan_extract"] = {
        "label": f"F: copied verbatim from {plan3} (sha256 {PLAN_SHA256}); texts may name rev 2c parts",
        "interpretation": PLAN["interpretation"], "invariants": PLAN["invariants"], "power_block": pb,
        "placement_notes": PLAN["placement_notes"], "routing_notes": PLAN["routing_notes"],
        "conditional_items": PLAN["conditional_items"], "user_decision_details": PLAN["user_decision_details"],
        "checks_to_add": PLAN["checks_to_add"]}

    # ---- remaining wording (ESP-12E -> ESP-12F, 22 pF -> 27 pF, J13) outside the history sections
    for key in list(spec):
        if key not in HISTORY_KEYS and key not in ("schema", "plan", "summary"):
            spec[key] = _walk_sub(spec[key])
    left: list[str] = []
    for key, val in spec.items():
        if key not in HISTORY_KEYS:
            _stale(val, key, left)
    spec["checks"]["stale_text_scan"] = {
        "what": "words that describe rev 2c (plug-in 5 V module, ESP-12E, J6, J13, 22 pF, 2 layers, 1.34 A) searched in "
                "every text outside the history sections",
        "pattern": STALE.pattern, "not_scanned": list(HISTORY_KEYS), "allowed_phrases": list(ALLOWED_PHRASES),
        "leftovers": left, "result": "PASS" if not left else "FAIL"}
    errors.extend(f"stale rev 2c text: {x}" for x in left)


# ------------------------------------------------------------------------------------------------ Korean summary
_D_KO = (
    "D1 Pololu 모듈을 대신할 5 V 변환기: 기본 A (TPS61023 승압 + TLV3012B 저전압 차단), 권장 A",
    "D2 로직 배터리(1S) 차단 전압: 기본 유지 (꺼짐 약 3.13 V, 켜짐 약 3.59 V), 권장 유지",
    "D3 요청 때문이 아니라 재고·도면 근거로 바꾼 부품번호 3건: 기본 유지, 권장 유지",
    "D4 ATmega128A-AU 4개를 누가 붙이는가: 기본 사용자 납땜, 권장 JLC 실장 (with_mcu 파일 쌍)",
    "D5 OpenLog 장착 방식: 기본 소켓 J5에 꽂기 (센서 모듈처럼 사용자가 장착), 권장 동일",
    "D6 프로그래밍 헤더 (J3, J19~J22, J24~J26): 기본 A (케이블타이 띠 안의 빈 도금 구멍), 자리가 확인되면 권장 B (띠 밖에 JLC 실장)",
    "D7 스위치·부저 커넥터 J15 / J17 (JST-PH): 기본 THT, 권장 THT",
    "D8 레벨시프터 FET (보드당 8개): 기본 C40912, 권장 C40912",
    "D9 JLC 주문 종류: 기본 Standard, 권장 Standard",
    "D10 툴링 홀: 기본 설계에 넣지 않음 (JLC가 붙이는 공정 레일에 생김)",
    "D11 수량: 기본 PCB 5장 / 실장 2장, 권장 먼저 2장 실장",
    "D12 모듈 소켓 J2 / J5 (1x6 직각): 기본 C22436147 + 주문일 재고 확인, 안 되면 C55629861, 그다음 C54876735",
    "D13 배터리 전압 분압 R15 / R16 (47k / 47k): 기본 유지",
    "D14 습기에 민감한 부품(ESP-12F, LED) 베이킹 요청: 기본 예 (주문 메모)",
    "D15 렌더와 맞춤 검사에 쓰는 모듈 치수: 기본 가정값 (실측 전까지 모든 그림·검사에 A 표시), 권장 실측",
    "D16 주문 화면의 PCB 옵션: 기본 권장값 (ENIG, 녹색 마스크, 흰색 실크, JLC 기본 4층 적층, 비아 덮기)",
    "D17 실장하지 않는 여분 기판: 기본 수락",
)
_USER_FIT_KO = (
    ("U1~U4", "ATmega128A-AU (TQFP-64) 4개", "기본: 사용자가 납땜 (칩당 64핀, 모두 256곳). JLC에 맡기려면 with_mcu BOM / CPL 쌍으로 주문 (LCSC C6510)"),
    ("J1", "GY-521 MPU-6050 모듈", "모듈에 딸린 1x8 핀을 기판의 도금 구멍 8개에 납땜 (모듈은 약 2.5 mm 받침 위에 눕힘 - 핀 스트립 플라스틱 높이, 가정 A). 기판에는 구멍만 있음"),
    ("J2", "BMP280 모듈 TYE-BMP29", "직각 6핀 헤더를 JLC가 실장한 소켓 J2에 꽂음 (받은 모듈의 헤더가 일자형이면 직각 핀을 먼저 납땜 - 헤더 종류 NOT_VERIFIED)"),
    ("J7", "NEO-7M GPS 모듈 SZH-NT07", "기체 상단 캡에 두고 4선 JST-GH 리드를 J7에 꽂음"),
    ("J5", "SparkFun OpenLog DEV-13712", "OpenLog에 직각 1x6 핀을 납땜한 뒤 소켓 J5에 꽂음 (맞는지 NOT_VERIFIED)"),
)
_NOT_VERIFIED_KO = (
    "만들거나 측정한 것은 없습니다. 전원부 수치는 모두 데이터시트 계산값(E)입니다.",
    "JLC가 이 기판을 받아 주는지는 사용자가 파일을 올리고 DFM 결과와 견적을 본 뒤에야 알 수 있습니다 (결제 전 확인).",
    "THT 6줄을 JLC가 납땜해 주는지는 견적에 표시될 때까지 NOT_VERIFIED입니다.",
    "이 문서는 사양 단계의 내용만 담습니다. 배치·배선 뒤의 결과(전원부 자리, 배터리 경로 폭 검사, 가장자리 거리, 모듈 간섭)는 README_ko.md 와 out/reports/ 의 보고서에 있습니다.",
    "U8은 라이브러리에 전용 심볼이 없어 일반 6핀 심볼을 씁니다. ERC는 U8 핀에 대해 아무 근거도 주지 않으며, 핀-네트 표 검사(rev3_checks.py u8-u9)가 그 역할을 합니다.",
    "차단 히스테리시스 여유는 최대 부하에서 28 mV(E)입니다. 전원 경로 저항이 0.29 Ohm을 넘으면 방전 끝에서 켜짐/꺼짐을 반복할 수 있습니다 (첫 기판에서 측정).",
    "켜짐 문턱이 최대 3.73 V입니다. 쉬는 전압이 그보다 낮은 팩은 보드를 켜지 못할 수 있습니다.",
    "극성 부품과 IC의 CPL 회전은 NOT_VERIFIED입니다. 주문 때 'Confirm Parts Placement'를 켜고, 전원을 넣기 전에 탄탈 방향을 눈으로 확인하십시오.",
    "재고는 움직입니다. spec/parts_sources.json 기록({day}) 기준으로 재고가 얇은 줄: C22436147 (소켓 J2 / J5). 주문일에는 scripts/jlc_gate.py 를 다시 돌려 모든 줄을 새로 확인합니다.",
    "센서 모듈·OpenLog의 치수와 핀 순서는 가정(A)입니다. 받은 모듈의 실크를 확인하십시오.",
)


def write_spec_ko(spec: dict, path: Path, board_dir: Path) -> None:
    """spec/SPEC_ko.md: plain-Korean summary of the rev 3 spec, generated (no hand edits, no timestamps)."""
    if len(_D_KO) != len(PLAN["user_decisions"]) or any(
            not d.startswith(f"D{i} ") or not k.startswith(f"D{i} ")
            for i, (d, k) in enumerate(zip(PLAN["user_decisions"], _D_KO), 1)):
        raise SystemExit("rev3_spec.write_spec_ko: the Korean decision list no longer matches the plan's D1..D17")
    root = board_dir.as_posix()
    s = spec["summary"]
    bs = spec["bom_sources"]
    sets = fitsets.fit_sets(spec["parts"])
    uv = PLAN["power_block"]["uvlo"]
    pb = PLAN["power_block"]
    prog = spec["prog_headers"]["option"]
    # snapshot day of the JLC records the spec was built from (make_all.py re-evaluates the cache with this day)
    day = json.loads((board_dir / "spec" / "parts_sources.json").read_text(encoding="utf-8"))["today"]
    L: list[str] = []
    A = L.append
    A("# VTOL 캐리어 PCB rev 3 - 전기 사양 요약")
    A("")
    A(f"이 문서는 `{root}/scripts/build_spec.py` 가 `{root}/spec/carrier_spec.json` 과 함께 자동으로 만든 것입니다. "
      "손으로 고치지 말고 스크립트를 다시 돌리십시오.")
    A(f"설계의 기준은 계획서 `{root}/spec/REV3_PLAN.md` (기계가 읽는 사본 `{root}/spec/rev3_plan.json`, "
      f"계획 날짜 {PLAN['plan_date']}) 이며, 사양이 계획과 부품 하나하나 같은지 스크립트가 비교합니다 "
      f"(비교 {spec['checks']['plan_rev3']['items_compared']}항목, 핀 {spec['checks']['plan_rev3']['pins_compared']}개, "
      f"차이 {len(spec['checks']['plan_rev3']['differences'])}건).")
    A("")
    A("표시: F = 파일·데이터시트에서 읽음, M = 여기서 도구로 측정, E = 추정(계산), A = 가정. 근거가 없으면 NOT_VERIFIED.")
    A("이 문서의 수치는 사양을 만드는 스크립트의 계산·대조 결과입니다. ERC / DRC 결과는 kicad-cli 보고서"
      f"(`{root}/out/reports/`)에만 있습니다.")
    A("")
    A("## 1. rev 2c에서 달라진 것")
    A("")
    A("- 5 V 전원: 꽂는 Pololu 모듈(J6)을 없애고 기판 위 승압 회로로 바꿨습니다. U8 TPS61023DRLR + L5 1.0 uH + "
      "분압 R63 / R64 + C57 / C58, 그리고 저전압 차단용 비교기 U9 TLV3012BIDBVR + R65~R68 + C59 / C60 (새 부품 13개, 새 네트 5개: "
      "BOOST_SW, BOOST_FB, BOOST_EN, UVLO_SENSE, UVLO_REF).")
    A("- 무선 모듈: ESP-12E 대신 핀이 같은 ESP-12F (JLC 라이브러리 부품 C82891). 풋프린트는 KiCad의 RF_Module:ESP-12E 를 그대로 씁니다 "
      "(라이브러리에 ESP-12F 풋프린트가 따로 없음).")
    A("- MCU와 센서 모듈을 뺀 모든 실장 부품을 JLCPCB 조립 라이브러리의 주문 가능한 부품으로 정했습니다 "
      f"(재고 근거: `{root}/spec/parts_sources.json`).")
    A("- 부품마다 `fit` (누가 붙이는가) 값을 가집니다. JLC BOM / CPL은 이 값에서만 만들어집니다.")
    A("- MCU 4개의 핀 256개가 가는 네트, 기판 외곽 76 x 70 mm, 커넥터·MCU·크리스털·모듈 소켓·ESP 위치는 rev 2c 그대로입니다. "
      f"rev 2c 자료는 `{root}/archive/rev2c_pololu_1s/` 에 보관했습니다.")
    A("")
    A("## 2. 누가 무엇을 붙이는가")
    A("")
    A(f"부품 자리 {s['parts']}개 (네트 {s['nets']}개).")
    A("")
    A("| fit | 뜻 | 개수 | 부품 |")
    A("|---|---|---|---|")
    mean = {"jlc": "JLC가 실장 (BOM / CPL에 들어감)", "jlc_optional": "기본은 사용자 납땜, with_mcu 파일 쌍에서만 JLC 실장",
            "user": "사용자가 납땜", "none": "부품 없음 (빈 도금 구멍 / 미실장 / 구리 패턴만)"}
    for f in fitsets.FITS:
        refs = sets[f]
        shown = ", ".join(refs) if len(refs) <= 20 else f"{', '.join(refs[:6])} ... (목록은 carrier_spec.json 의 parts[].fit)"
        A(f"| {f} | {mean[f]} | {len(refs)} | {shown} |")
    A("")
    A(f"JLC 실장 부품 {s['jlc_fitted_parts']}개, BOM {s['jlc_bom_lines']['total']}줄 "
      f"(SMT {s['jlc_bom_lines']['smt']}줄 + THT {s['jlc_bom_lines']['tht']}줄). "
      f"MCU까지 JLC에 맡기는 with_mcu 쌍은 {s['with_mcu_pair']['jlc_fitted_parts']}개 / {s['with_mcu_pair']['jlc_bom_lines']}줄입니다.")
    A("")
    A("## 3. 5 V 전원부 (모두 E: 데이터시트 계산, 측정 아님)")
    A("")
    A(f"- 출력 전압: 정격 {pb['vout_V']['nominal_V']:.2f} V (공차 범위 {pb['vout_V']['min_V']:.2f} ~ {pb['vout_V']['max_V_pwm']:.2f} V).")
    A(f"- 부하 가정: +5V 평소 273 mA / 최대 783 mA. IC 입력 3.0 V에서 낼 수 있는 출력 전류 "
      f"{pb['iout_capability_at_IC_input_3p0V_ILIM_min']['Iout_CL_A']} A (최대 부하의 "
      f"{pb['iout_capability_at_IC_input_3p0V_ILIM_min']['margin_vs_peak_783mA']}배).")
    A(f"- 저전압 차단: 꺼짐 {uv['V_off_nominal']:.2f} V ({uv['V_off_worst'][0]:.3f} ~ {uv['V_off_worst'][1]:.3f} V), "
      f"켜짐 {uv['V_on_nominal']:.2f} V ({uv['V_on_worst'][0]:.3f} ~ {uv['V_on_worst'][1]:.3f} V). 저항값으로 고정되어 조정할 것이 없습니다.")
    A(f"- 차단 뒤에도 약 {pb['drain_after_cutoff_uA']['new_block_total_typ']} uA(새 회로) + 34~62 uA(R15 / R16 분압)가 흐릅니다. "
      "보관할 때는 반드시 스위치를 끄십시오.")
    A("- U8, U9 핀 번호는 데이터시트 표에서 옮겼습니다: "
      + "; ".join(f"{r} {DATASHEET_PINS[r]['part']}: " + ", ".join(f"{n} {name}" for n, name in DATASHEET_PINS[r]['pins'].items())
                  for r in ("U8", "U9")) + ".")
    A("")
    A("## 4. JLC BOM 줄")
    A("")
    A("| 줄 | Comment | Designator | Footprint | LCSC Part # | 공정 | 수량 |")
    A("|---|---|---|---|---|---|---|")
    for r in bs["board_lines"]:
        A(f"| {r['line']} | {r['comment']} | {', '.join(r['refs'])} | {r['footprint'].split(':', 1)[1]} | {r['part_number']} | "
          f"{r['process']} | {r['qty']} |")
    A("")
    for r in bs["with_mcu_extra_lines"]:
        A(f"with_mcu 쌍에만 추가되는 줄: {r['comment']} | {', '.join(r['refs'])} | {r['footprint'].split(':', 1)[1]} | {r['part_number']} | "
          f"{r['process']} | {r['qty']}")
    A("")
    A("Footprint 칸은 KiCad 풋프린트 이름입니다 (주문용 BOM 파일의 Footprint 칸과 같은 값). "
      f"주문용 파일은 내보내기 단계(`{root}/scripts/checks_exports.py`)가 이 사양의 `fit` 값으로 만든 "
      f"`{root}/out/jlcpcb_bom.csv` / `{root}/out/jlcpcb_cpl.csv` (MCU 포함 쌍: `*_with_mcu.csv`) 이고, 위 표는 그 파일의 근거입니다. "
      f"주문 방법은 `{root}/out/ORDER_ko.md` 에 있습니다.")
    A("")
    A("## 5. 프로그래밍 헤더 (결정 D6)")
    A("")
    A(f"현재 설정: **{prog}** (환경변수 `VTOL_PROG_HEADERS`, 기본 A).")
    A("- A: J3, J19~J22 (2x3), J24~J26 (1x2) 은 rev 2c와 같은 자리의 빈 도금 구멍입니다. 프로그래밍하려면 사용자가 헤더를 납땜하거나 "
      "포고핀 어댑터를 씁니다 (포고핀 접촉은 NOT_VERIFIED).")
    A("- B: 같은 부품번호를 케이블타이 띠 밖에 JLC가 실장하는 SMD 헤더로 둡니다 (2x3: C32713283, 1x2: C41417359; BOM 42줄, JLC 실장 165개). "
      "사용자의 승인이 있어야 적용합니다. 띠 밖에 자리가 있는지 배치 단계에서 찾은 결과는 "
      f"`{root}/out/reports/programming_access.json` 에 있습니다 (사양 문서는 배치 결과를 읽지 않습니다; 요약은 `{root}/README_ko.md`).")
    A("")
    A("## 6. 사용자가 붙이거나 꽂는 것")
    A("")
    A("| 자리 | 부품 | 방법 |")
    A("|---|---|---|")
    for ref, what, how in _USER_FIT_KO:
        A(f"| {ref} | {what} | {how} |")
    A("")
    A("기판 밖에서 따로 사는 것(배터리, 스위치와 리드, GPS 리드, 부저, 모듈, MCU 등)의 목록은 계획서의 off_board_purchase 항목"
      f"(`{root}/spec/rev3_plan.json`)에 있고, 구매 목록 `{root}/out/off_board_purchase.csv` 와 주문 안내서 `{root}/out/ORDER_ko.md` 6장에 "
      "표로 있습니다. 이 품목들의 재고·가격은 NOT_VERIFIED입니다.")
    A("")
    A("## 7. 사용자 결정 (기본값으로 진행 중)")
    A("")
    for d in _D_KO:
        A(f"- {d}")
    A("")
    A("## 8. 확인되지 않은 것")
    A("")
    for t in _NOT_VERIFIED_KO:
        A("- " + t.replace("{day}", day))
    A("")
    A("## 9. 파일과 다시 만드는 방법")
    A("")
    A(f"- 사양: `{root}/spec/carrier_spec.json`")
    A(f"- 계획: `{root}/spec/REV3_PLAN.md`, `{root}/spec/rev3_plan.json`")
    A(f"- JLC 부품 근거: `{root}/spec/parts_sources.json`")
    A(f"- 라이브러리 확인: `{root}/spec/library_probe.json`")
    A(f"- rev 2c 보관본: `{root}/archive/rev2c_pololu_1s/`")
    A("")
    A(f"전체(사양 → 회로도 → 배치 → 기록된 배선 재생 → 검사 → 주문 파일 → 그림 → 문서)는 작업 폴더 `{root}/scripts` 에서 "
      "`B:/Claude/.venv/Scripts/python.exe -X utf8 -B make_all.py` 한 줄로 다시 만듭니다 (단계 목록: `make_all.py --list`). "
      "사양 부분만 다시 만들 때의 순서:")
    A("")
    A("1. `probe_library.py` - KiCad 라이브러리에서 심볼·풋프린트·핀을 읽어 기록")
    A(f"2. `jlc_sources.py --today {day}` - 저장된 JLC 기록으로 부품 근거를 다시 평가 (네트워크 없음; 주문일 재고 확인은 `jlc_gate.py`)")
    A("3. `build_spec.py` - 이 사양과 이 문서를 생성")
    A("4. `build_schematic.py` - 회로도 생성")
    A("5. `rev3_checks.py all` - kicad-cli ERC·네트리스트 내보내기와 네트리스트 기준 검사")
    A("")
    path.write_text("\n".join(L), encoding="utf-8", newline="\n")
