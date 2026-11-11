"""Netlist-level checks of the rev 3 carrier board (standard library only).

Invariant this module enforces: every PASS in its reports is backed by a file read in the same run - the netlist
that kicad-cli exported from the schematic, spec/carrier_spec.json, spec/rev3_plan.json, spec/library_probe.json,
the archived rev 2c netlist and the datasheet text extracts. A missing input makes the sub-check NOT_VERIFIED,
never PASS. ERC results come only from kicad-cli (sub-command ``export``). Nothing here edits the design; the only
files written are the reports, the ERC report / netlist of ``export`` and the rebuilt (byte-identical) spec and
schematic of ``determinism``.

Sub-commands (each writes out/reports/rev3_<name>.json; exit 0 = PASS, 1 = FAIL, 3 = NOT_VERIFIED):

  export            kicad-cli ERC (all severities) + netlist export  -> out/reports/erc.json, out/vtol_carrier.net
  netlist-vs-spec   every pin of every part: netlist net == spec net / declared no-connect
  mcu-pinmap        the 256 pins of U1..U4 == the rev 2c pin map
  u8-u9             pad -> net of the two new ICs == datasheet pin table x intended function
  fit               fit sets and JLC BOM lines == plan; netlist components == spec parts
  nets              the five new nets == plan; nothing of rev 2c lost except J6
  determinism       spec + schematic built twice: byte-identical
  export-files      JLC BOM / CPL pairs and gerber zips of checks_exports.py: headers, fit sets, one line per code,
                    CPL == rotation table, silk marks of the orientation-deciding parts, zips differ only in F.Paste
  export-determinism  BOM / CPL / zips written twice into temp folders + rotation table rebuilt: byte-identical
  all               determinism, export, then the rest; summary in out/reports/rev3_checks.json
                    (export-files / export-determinism are run on their own, after checks_exports.py)

The export step re-uses ``fit_report`` and ``check_fit_files`` (BOM / CPL designators against the fit sets), the PCB
step ``expected_pad_nets`` (U8 / U9 pad -> net table for the pads of the .kicad_pcb).

Run: ``B:/Claude/.venv/Scripts/python.exe -X utf8 -B rev3_checks.py all`` with cwd = this folder (any Python >= 3.10
works for everything except ``determinism``, whose build scripts need the ai_eda package).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from collections import Counter
from pathlib import Path

import fitsets
import rev3_spec as R3
from fitsets import natural

ROOT = Path(__file__).resolve().parent.parent          # .../pcb/vtol_carrier
SCRIPTS = ROOT / "scripts"
SPEC = ROOT / "spec" / "carrier_spec.json"
SPEC_KO = ROOT / "spec" / "SPEC_ko.md"
PLAN = ROOT / "spec" / "rev3_plan.json"
PROBE = ROOT / "spec" / "library_probe.json"
KICAD = ROOT / "kicad"
SCH = KICAD / "vtol_carrier.kicad_sch"
MANIFEST = KICAD / "schematic_manifest.json"
OUT = ROOT / "out"
REP = OUT / "reports"
NETLIST = OUT / "vtol_carrier.net"
ERC_JSON = REP / "erc.json"
REV2C = ROOT / "archive" / "rev2c_pololu_1s"
REV2C_NETLIST = REV2C / "out" / "vtol_carrier.net"
REV2C_ERC = REV2C / "out" / "reports" / "erc.json"
REV2C_PINMAP = Path(R3.SCR) / "scout" / "rev2c_mcu_pinmap.json"
KICAD_CLI_DEFAULT = "C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/bin/kicad-cli.exe"

MCUS = ("U1", "U2", "U3", "U4")
PASS, FAIL, NOT_VERIFIED = "PASS", "FAIL", "NOT_VERIFIED"

#: datasheet text extracts the U8 / U9 pin tables were transcribed from (file, expected hash, cited line range)
DATASHEET_FILES = {
    "U8": {"file": Path(R3.SCR) / "datasheets" / "tps61023" / "tps61023.txt",
           "sha256": "db5a2ea386e175f6062f84b450919e809d82d583006790b1e0b299994a8f8cad", "first": 148, "last": 160,
           "document": "TI TPS61023 datasheet SLVSF14B, section 5: Figure 5-1 (DRL package, 6-pin SOT563, top view) "
                       "and the Pin Functions table below it"},
    "U9": {"file": Path(R3.SCR) / "verify" / "electrical_work" / "tlv3011.txt",
           "sha256": "bc930405eef950466c3522ef976e71974eb38b6c2e38bf8aaebf32fc22bc2d05", "first": 154, "last": 163,
           "document": "TI TLV3011 / TLV3012 datasheet SBOS300C, Table 5-1 Pin Functions (DCK / DBV package)"},
}
#: design intent, pin function -> net (spec/rev3_plan.json power_block and new_nets)
FUNCTION_NET = {
    "U8": {"FB": "BOOST_FB", "EN": "BOOST_EN", "VIN": "VBAT_LOGIC_SW", "GND": "GND", "SW": "BOOST_SW",
           "VOUT": "+5V"},
    "U9": {"OUT": "BOOST_EN", "V-": "GND", "IN+": "UVLO_SENSE", "IN-": "UVLO_REF", "REF": "UVLO_REF",
           "V+": "VBAT_LOGIC_SW"},
}
#: substitute symbol of U9: (symbol pin name, electrical type) -> datasheet pin name of the TLV3012
MCP65R41_AS_TLV3012 = {("", "output"): "OUT", ("V-", "power_in"): "V-", ("+", "input"): "IN+",
                       ("-", "input"): "IN-", ("Vref", "power_out"): "REF", ("V+", "power_in"): "V+"}
#: refs the plan lists as "in neither the BOM nor the CPL" for option A (rev3_plan.json checks_to_add, fit-set line);
#: U7 (ESP-12F) and the 12 through-hole connectors joined the list with the plan amendments of 2026-11-12: the user
#: buys and solders them (JLC fits SMT parts only)
THT_BY_USER = ("J2", "J5", "J8", "J9", "J10", "J11", "J12", "J14", "J15", "J16", "J17", "J18")
NOT_JLC_OPTION_A = ("U1", "U2", "U3", "U4", "U7", "J1", "J3", "J19", "J20", "J21", "J22", "J24", "J25", "J26", "Q3",
                    "R47", "JP1", "NT1") + THT_BY_USER


# ------------------------------------------------------------------------------------------------- small helpers
def posix(path: Path | str) -> str:
    return str(path).replace("\\", "/")


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str | None:
    return sha256_bytes(path.read_bytes()) if path.is_file() else None


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_report(name: str, data: dict) -> dict:
    REP.mkdir(parents=True, exist_ok=True)
    path = REP / f"rev3_{name}.json"
    data = {"report": posix(path), **data}
    path.write_bytes((json.dumps(data, indent=1, ensure_ascii=False) + "\n").encode("utf-8"))
    return data


def overall(results: list[str]) -> str:
    if FAIL in results:
        return FAIL
    return NOT_VERIFIED if NOT_VERIFIED in results else PASS


# --------------------------------------------------------------------------------------- s-expression / netlist
def parse_sexpr(text: str) -> list:
    """Nested lists of strings; quoted strings and bare atoms are both plain ``str``."""
    stack: list[list] = [[]]
    i, n = 0, len(text)
    while i < n:
        ch = text[i]
        if ch in " \t\r\n":
            i += 1
        elif ch == "(":
            stack.append([])
            i += 1
        elif ch == ")":
            done = stack.pop()
            if not stack:
                raise ValueError("unbalanced ')'")
            stack[-1].append(done)
            i += 1
        elif ch == '"':
            j, buf = i + 1, []
            while text[j] != '"':
                if text[j] == "\\":
                    j += 1
                    buf.append({"n": "\n", "t": "\t", "r": "\r"}.get(text[j], text[j]))
                else:
                    buf.append(text[j])
                j += 1
            stack[-1].append("".join(buf))
            i = j + 1
        else:
            j = i
            while j < n and text[j] not in " \t\r\n()":
                j += 1
            stack[-1].append(text[i:j])
            i = j
    if len(stack) != 1 or len(stack[0]) != 1:
        raise ValueError("unbalanced s-expression")
    return stack[0][0]


def children(node: list, tag: str) -> list[list]:
    return [c for c in node[1:] if isinstance(c, list) and c and c[0] == tag]


def child(node: list, tag: str) -> list | None:
    found = children(node, tag)
    return found[0] if found else None


def val(node: list | None, tag: str) -> str | None:
    c = child(node, tag) if node else None
    return c[1] if c and len(c) > 1 and isinstance(c[1], str) else None


class Netlist:
    """A KiCad s-expression netlist (``kicad-cli sch export netlist --format kicadsexpr``)."""

    def __init__(self, path: Path):
        self.path = Path(path)
        raw = self.path.read_bytes()
        self.sha256 = sha256_bytes(raw)
        self.sha256_without_date_line = normalised_netlist(self.path)
        tree = parse_sexpr(raw.decode("utf-8"))
        if tree[0] != "export":
            raise ValueError(f"{path}: not a KiCad netlist")
        design = child(tree, "design")
        self.design = {k: val(design, k) for k in ("source", "date", "tool")}
        self.components: dict[str, dict] = {}
        for c in children(child(tree, "components"), "comp"):
            props = {val(p, "name"): val(p, "value") or "" for p in children(c, "property")}
            lib = child(c, "libsource")
            ref = val(c, "ref")
            if ref in self.components:
                raise ValueError(f"{path}: component {ref} twice")
            self.components[ref] = {"value": val(c, "value"), "footprint": val(c, "footprint"),
                                    "symbol": f"{val(lib, 'lib')}:{val(lib, 'part')}", "dnp": "dnp" in props,
                                    "sheet": props.get("Sheetname")}
        self.nets: dict[str, list[tuple[str, str]]] = {}
        self.net_class: dict[str, str | None] = {}
        self.pin_type: dict[tuple[str, str], str | None] = {}
        for n in children(child(tree, "nets"), "net"):
            name = val(n, "name")
            if name in self.nets:
                raise ValueError(f"{path}: net {name} twice")
            nodes = []
            for x in children(n, "node"):
                key = (val(x, "ref"), val(x, "pin"))
                nodes.append(key)
                self.pin_type[key] = val(x, "pintype")
            self.nets[name] = sorted(nodes, key=lambda k: (natural(k[0]), natural(k[1])))
            self.net_class[name] = val(n, "class")

    @staticmethod
    def is_unconnected(name: str) -> bool:
        return name.startswith("unconnected-(")

    def named(self) -> dict[str, list[tuple[str, str]]]:
        return {k: v for k, v in self.nets.items() if not self.is_unconnected(k)}

    def members(self, name: str) -> list[str]:
        return [f"{r}.{p}" for r, p in self.nets.get(name, [])]

    def pin_table(self) -> dict[str, dict[str, str]]:
        """{ref: {pin number: net name | 'NC'}}; 'NC' = a single-pin net that KiCad named 'unconnected-(...)'."""
        out: dict[str, dict[str, str]] = {}
        for name, nodes in self.nets.items():
            nc = self.is_unconnected(name)
            if nc and len(nodes) != 1:
                raise ValueError(f"{self.path}: {name} has {len(nodes)} pins")
            for ref, pin in nodes:
                if pin in out.setdefault(ref, {}):
                    raise ValueError(f"{self.path}: pin {ref}.{pin} in two nets")
                out[ref][pin] = "NC" if nc else name
        return out

    def evidence(self) -> dict:
        # esp-user-fix step (finding V4): a later build step exports the netlist again, so the file on disk carries
        # a later '(date ...)' line and another raw sha256 than the one read here; the hash without that line is the
        # one that identifies the content
        return {"file": posix(self.path), "sha256": self.sha256,
                "sha256_is": "of the file as it was when this check read it (it carries the export time; a later "
                             "build step exports it again)",
                "sha256_without_date_line": self.sha256_without_date_line, "tool": self.design["tool"],
                "export_date_in_file": self.design["date"], "components": len(self.components),
                "named_nets": len(self.named()), "unconnected_single_pin_nets": len(self.nets) - len(self.named())}


def load_netlist(path: Path | None = None) -> Netlist | None:
    """The netlist at ``path`` (default: the module's NETLIST, read at call time), None when the file is missing."""
    path = Path(path) if path is not None else NETLIST
    return Netlist(path) if path.is_file() else None


def no_netlist(name: str) -> dict:
    return write_report(name, {"result": NOT_VERIFIED,
                               "reason": f"netlist {posix(NETLIST)} not found - run 'rev3_checks.py export' first"})


# ------------------------------------------------------------------------------------------------------- export
def find_kicad_cli() -> str | None:
    for cand in (os.environ.get("KICAD_CLI"), shutil.which("kicad-cli"), KICAD_CLI_DEFAULT):
        if cand and Path(cand).is_file():
            return posix(cand)
    return None


def run(args: list[str], cwd: Path | None = None, env: dict | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(args, cwd=cwd, env=env, capture_output=True, text=True, encoding="utf-8", errors="replace")


def schematic_files() -> list[Path]:
    """Root schematic + the sub-sheets named in the schematic manifest."""
    names = [k for k in load_json(MANIFEST)["files"] if k.endswith(".kicad_sch")]
    return [KICAD / n for n in sorted(names)]


def check_export() -> dict:
    """kicad-cli ERC with all severities + the netlist export every other check reads."""
    cli = find_kicad_cli()
    if not cli:
        return write_report("erc", {"result": NOT_VERIFIED, "reason": "kicad-cli not found (KICAD_CLI, PATH, "
                                                                      f"{KICAD_CLI_DEFAULT})"})
    version = run([cli, "version"]).stdout.strip()
    before = {p.name: sha256_file(p) for p in schematic_files()}
    REP.mkdir(parents=True, exist_ok=True)
    erc_cmd = [cli, "sch", "erc", "--format", "json", "--severity-all", "--units", "mm", "-o", posix(ERC_JSON),
               posix(SCH)]
    net_cmd = [cli, "sch", "export", "netlist", "--format", "kicadsexpr", "-o", posix(NETLIST), posix(SCH)]
    r1 = run(erc_cmd, cwd=ROOT)
    r2 = run(net_cmd, cwd=ROOT)
    problems = []
    if r1.returncode != 0 or not ERC_JSON.is_file():
        problems.append(f"kicad-cli sch erc exit {r1.returncode}: {r1.stderr.strip()[:300]}")
    if r2.returncode != 0 or not NETLIST.is_file():
        problems.append(f"kicad-cli sch export netlist exit {r2.returncode}: {r2.stderr.strip()[:300]}")
    if problems:
        return write_report("erc", {"result": FAIL, "problems": problems, "kicad_cli": cli, "version": version})
    d = load_json(ERC_JSON)
    counts: Counter = Counter()
    listed = []
    for sh in d["sheets"]:
        for v in sh["violations"]:
            counts[f"{v['severity']}:{v['type']}"] += 1
            listed.append({"sheet": sh["path"], "severity": v["severity"], "type": v["type"],
                           "description": v.get("description"),
                           "items": [i.get("description") for i in v.get("items", [])]})
    errors = sum(n for k, n in counts.items() if k.startswith("error"))
    warnings = sum(n for k, n in counts.items() if k.startswith("warning"))
    ignored = sorted(c["key"] for c in d.get("ignored_checks", []))
    ignored_2c = sorted(c["key"] for c in load_json(REV2C_ERC).get("ignored_checks", [])) if REV2C_ERC.is_file() \
        else None
    if ignored_2c is None:
        problems.append("rev 2c ERC report not found: ignored-check list not compared")
    elif ignored != ignored_2c:
        problems.append(f"ignored ERC checks differ from rev 2c: {ignored} / {ignored_2c}")
    if errors:
        problems.append(f"{errors} ERC error(s)")
    if before != {p.name: sha256_file(p) for p in schematic_files()}:
        problems.append("a schematic file changed while kicad-cli ran")
    nl = Netlist(NETLIST)
    return write_report("erc", {
        "result": FAIL if problems else PASS, "problems": problems,
        "tool": "kicad-cli sch erc --format json --severity-all --units mm", "kicad_cli": cli,
        "tool_version": version, "kicad_version_in_report": d.get("kicad_version"),
        "erc_report": posix(ERC_JSON), "erc_report_date": d.get("date"),
        "included_severities": d.get("included_severities"), "sheets_checked": [s["path"] for s in d["sheets"]],
        "errors": errors, "warnings": warnings, "counts": dict(sorted(counts.items())), "violations": listed,
        "ignored_checks": ignored, "ignored_checks_rev2c": ignored_2c,
        "ignored_checks_note": "the generated .kicad_pro has no 'erc' section (no severity is set by the project), "
                               "so these are the checks kicad-cli itself reports as ignored; the list is compared "
                               "with the archived rev 2c ERC report - nothing was switched off for rev 3",
        "kicad_pro_sets_erc_severities": "erc" in load_json(KICAD / "vtol_carrier.kicad_pro"),
        "u8_note": "ERC gives NO evidence for U8: its symbol Connector_Generic:Conn_02x03_Counter_Clockwise has "
                   "six passive pins, so a wrong connection on U8 cannot raise an ERC violation. Sub-command "
                   "u8-u9 (pad -> net table against the datasheet) replaces it.",
        "schematic_sha256": before, "netlist": nl.evidence(),
        "commands": [" ".join(erc_cmd), " ".join(net_cmd)]})


# ---------------------------------------------------------------------------------------------- netlist vs spec
def check_netlist_vs_spec() -> dict:
    nl = load_netlist()
    if nl is None:
        return no_netlist("netlist_vs_spec")
    spec = load_json(SPEC)
    want = R3.pin_tables(spec["parts"], spec["nets"], spec["no_connect"])
    got = nl.pin_table()
    diffs = []
    for ref in sorted(set(want) | set(got), key=natural):
        w, g = want.get(ref, {}), got.get(ref, {})
        for pin in sorted(set(w) | set(g), key=natural):
            if w.get(pin) != g.get(pin):
                diffs.append({"pin": f"{ref}.{pin}", "spec": w.get(pin), "netlist": g.get(pin)})
    byref = {p["refdes"]: p for p in spec["parts"]}
    comp_diffs = []
    for ref in sorted(set(byref) | set(nl.components), key=natural):
        p, c = byref.get(ref), nl.components.get(ref)
        if p is None or c is None:
            comp_diffs.append({"ref": ref, "spec": p is not None, "netlist": c is not None})
            continue
        for key, a, b in (("value", p["value"], c["value"]), ("footprint", p["footprint"], c["footprint"]),
                          ("symbol", p["symbol"], c["symbol"]),
                          ("dnp", not fitsets.populated(p), c["dnp"])):
            if a != b:
                comp_diffs.append({"ref": ref, "field": key, "spec": a, "netlist": b})
    spec_names = {n["name"] for n in spec["nets"]}
    named = set(nl.named())
    pins = [v for t in want.values() for v in t.values()]
    return write_report("netlist_vs_spec", {
        "result": PASS if not diffs and not comp_diffs and spec_names == named else FAIL,
        "tool": "kicad-cli netlist export + scripts/rev3_checks.py netlist-vs-spec",
        "netlist": nl.evidence(), "spec": posix(SPEC), "spec_sha256": sha256_file(SPEC),
        "parts": len(byref), "pins_compared": len(pins), "pins_connected": sum(1 for v in pins if v != "NC"),
        "pins_no_connect": sum(1 for v in pins if v == "NC"), "pin_differences": diffs,
        "component_fields_compared": ["value", "footprint", "symbol", "dnp == not populated"],
        "component_differences": comp_diffs, "spec_nets": len(spec_names), "netlist_named_nets": len(named),
        "nets_only_in_spec": sorted(spec_names - named), "nets_only_in_netlist": sorted(named - spec_names),
        "dnp_parts_in_netlist": sorted((r for r, c in nl.components.items() if c["dnp"]), key=natural)})


# --------------------------------------------------------------------------------------------------- MCU pin map
def mcu_pins(table: dict[str, dict[str, str]]) -> dict[str, dict[str, str]]:
    return {u: dict(sorted(table.get(u, {}).items(), key=lambda kv: natural(kv[0]))) for u in MCUS}


def check_mcu_pinmap() -> dict:
    nl = load_netlist()
    if nl is None:
        return no_netlist("mcu_pinmap")
    got = mcu_pins(nl.pin_table())
    problems = []
    ref_source = None
    reference = None
    if REV2C_PINMAP.is_file():
        reference = load_json(REV2C_PINMAP)
        ref_source = {"file": posix(REV2C_PINMAP), "sha256": sha256_file(REV2C_PINMAP)}
    old = load_netlist(REV2C_NETLIST)
    cross = None
    if old is not None:
        from_archive = mcu_pins(old.pin_table())
        if reference is None:
            reference = from_archive
            ref_source = {"file": posix(REV2C_NETLIST), "sha256": old.sha256,
                          "note": "pin map derived here from the archived rev 2c netlist (scout file not found)"}
        else:
            bad = [f"{u}.{p}" for u in MCUS for p in sorted(set(reference.get(u, {})) | set(from_archive[u]),
                                                            key=natural)
                   if reference.get(u, {}).get(p) != from_archive[u].get(p)]
            cross = {"archived_rev2c_netlist": posix(REV2C_NETLIST), "sha256": old.sha256,
                     "differences_reference_vs_archived_netlist": bad}
            if bad:
                problems.append(f"reference pin map differs from the archived rev 2c netlist on {len(bad)} pins")
    if reference is None:
        return write_report("mcu_pinmap", {"result": NOT_VERIFIED, "reason": "no rev 2c reference: neither "
                                           f"{posix(REV2C_PINMAP)} nor {posix(REV2C_NETLIST)} exists"})
    diffs = []
    compared = 0
    per_chip = {}
    for u in MCUS:
        want, have = reference.get(u, {}), got[u]
        if sorted(have, key=natural) != [str(i) for i in range(1, 65)]:
            problems.append(f"{u}: the netlist lists {len(have)} pins, not pins 1..64")
        for pin in sorted(set(want) | set(have), key=natural):
            compared += 1
            if want.get(pin) != have.get(pin):
                diffs.append({"pin": f"{u}.{pin}", "rev2c": want.get(pin), "rev3": have.get(pin)})
        per_chip[u] = {"pins": len(have), "connected": sum(1 for v in have.values() if v != "NC"),
                       "no_connect": sum(1 for v in have.values() if v == "NC")}
    if compared != 256:
        problems.append(f"{compared} pins compared, expected 256")
    return write_report("mcu_pinmap", {
        "result": PASS if not diffs and not problems else FAIL,
        "tool": "kicad-cli netlist export + scripts/rev3_checks.py mcu-pinmap", "netlist": nl.evidence(),
        "reference": ref_source, "reference_cross_check": cross, "pins_compared": compared,
        "differences": diffs, "per_chip": per_chip, "problems": problems})


# ------------------------------------------------------------------------------------------------------ U8 / U9
def expected_pad_nets(ref: str) -> dict[str, str]:
    """{pad number: net} of U8 / U9 = datasheet pin table (pad -> function) x design intent (function -> net).

    Also the table the PCB step compares the pads of the .kicad_pcb with."""
    return {pad: FUNCTION_NET[ref][name] for pad, name in R3.DATASHEET_PINS[ref]["pins"].items()}


def read_datasheet_rows(ref: str) -> dict:
    """Re-read the cited lines of the datasheet text extract: {pad: {name, line}} (file hash checked)."""
    info = DATASHEET_FILES[ref]
    out = {"file": posix(info["file"]), "document": info["document"],
           "cited_lines": f"{info['first']}-{info['last']}", "expected_sha256": info["sha256"]}
    if not info["file"].is_file():
        return {**out, "status": NOT_VERIFIED, "reason": "datasheet text extract not found on this machine"}
    raw = info["file"].read_bytes()
    out["sha256"] = sha256_bytes(raw)
    if out["sha256"] != info["sha256"]:
        return {**out, "status": FAIL, "reason": "datasheet text extract has a different hash"}
    lines = raw.decode("utf-8", errors="replace").split("\n")
    rows = {}
    for no in range(info["first"], info["last"] + 1):
        text = lines[no - 1].rstrip("\r").replace("\u2013", "-").replace("\u2212", "-")
        m = re.match(r"^([1-6]) (\S+) (I|O|PWR|-) ", text)
        if m:
            rows[m.group(1)] = {"name": m.group(2), "line": no}
    transcribed = R3.DATASHEET_PINS[ref]["pins"]
    same = {k: v["name"] for k, v in rows.items()} == transcribed
    return {**out, "rows_read": rows, "transcribed_table": transcribed, "status": PASS if same else FAIL,
            "reason": None if same else "rows read from the cited lines differ from the transcribed table"}


def footprint_pads(lib_id: str, probe: dict) -> dict:
    """Pad centres of the library footprint (read from the .kicad_mod on disk) and their numbering direction."""
    entry = probe["footprints"].get(lib_id)
    if not entry:
        return {"status": FAIL, "reason": f"{lib_id} not in spec/library_probe.json"}
    path = Path(entry["path"])
    out = {"footprint": lib_id, "library_file": posix(path), "pads_in_probe": entry["pads"]}
    if not path.is_file():
        return {**out, "status": NOT_VERIFIED, "reason": "library file not found on this machine"}
    tree = parse_sexpr(path.read_text(encoding="utf-8"))
    pads = {}
    for p in children(tree, "pad"):
        at = child(p, "at")
        pads[p[1]] = [float(at[1]), float(at[2])]
    left = sorted((n for n, (x, _) in pads.items() if x < 0), key=lambda n: pads[n][1])
    right = sorted((n for n, (x, _) in pads.items() if x > 0), key=lambda n: -pads[n][1])
    ccw = left == ["1", "2", "3"] and right == ["4", "5", "6"]
    return {**out, "pad_centres_mm": dict(sorted(pads.items())), "library_file_sha256": sha256_file(path),
            "numbering": "pads 1-2-3 on the -x side from top to bottom, 4-5-6 on the +x side from bottom to top "
                         "(counter-clockwise seen from the top)" if ccw else "NOT the usual counter-clockwise order",
            "status": PASS if ccw and sorted(pads) == ["1", "2", "3", "4", "5", "6"] else FAIL}


def check_u8_u9() -> dict:
    nl = load_netlist()
    if nl is None:
        return no_netlist("u8_u9_pads")
    plan, probe, spec = load_json(PLAN), load_json(PROBE), load_json(SPEC)
    table = nl.pin_table()
    byref = {p["refdes"]: p for p in spec["parts"]}
    plan_pins = {c["refdes"]: c["rev3"]["pins"] for c in plan["changes"] if c["refdes"] in ("U8", "U9")}
    parts = {}
    statuses = []
    for ref in ("U8", "U9"):
        want = expected_pad_nets(ref)
        have = dict(sorted(table.get(ref, {}).items()))
        names = R3.DATASHEET_PINS[ref]["pins"]
        sym_pins = probe["symbols"][byref[ref]["symbol"]]["pins"]
        rows = [{"pad": pad, "datasheet_pin": names[pad], "expected_net": want[pad], "netlist_net": have.get(pad),
                 "plan_net": plan_pins[ref].get(pad), "netlist_pin_type": nl.pin_type.get((ref, pad)),
                 "equal": want[pad] == have.get(pad) == plan_pins[ref].get(pad)} for pad in sorted(want)]
        ds = read_datasheet_rows(ref)
        fp = footprint_pads(byref[ref]["footprint"], probe)
        checks = {"netlist_equals_table": have == want, "plan_equals_table": plan_pins[ref] == want,
                  "datasheet_rows_reread": ds["status"], "footprint_pads": fp["status"]}
        entry = {"part": R3.DATASHEET_PINS[ref]["part"], "source": R3.DATASHEET_PINS[ref]["source"],
                 "symbol": byref[ref]["symbol"], "footprint": byref[ref]["footprint"], "pad_net_table": rows,
                 "datasheet": ds, "footprint_library": fp,
                 "symbol_pins": [{"number": q[0], "name": q[1], "type": q[2]} for q in sym_pins]}
        if ref == "U8":
            passive = all(q[2] == "passive" for q in sym_pins) and \
                all(nl.pin_type.get((ref, pad)) == "passive" for pad in want)
            entry["erc_evidence"] = ("NONE: generic connector symbol, all six pins passive - ERC cannot see a wrong "
                                     "connection on U8; this table is the only netlist-level check of U8"
                                     if passive else "symbol pins are not all passive (unexpected)")
            checks["symbol_is_generic_passive"] = passive
        else:
            mapped = {q[0]: MCP65R41_AS_TLV3012.get((q[1], q[2])) for q in sym_pins}
            checks["substitute_symbol_pin_functions_equal_datasheet"] = mapped == names
            entry["substitute_symbol"] = {"symbol_pin_function_as_tlv3012": mapped, "datasheet": names,
                                          "note": "Comparator:MCP65R41 is a substitute symbol (no TLV3012 symbol "
                                                  "on disk); ERC sees U9 with the pin types of the MCP65R41, which "
                                                  "match the TLV3012B pin functions pad by pad"}
        entry["checks"] = checks
        ok = all(v is True or v == PASS for v in checks.values())
        missing = any(v == NOT_VERIFIED for v in checks.values())
        bad = any(v is False or v == FAIL for v in checks.values())
        entry["result"] = FAIL if bad else (NOT_VERIFIED if missing else (PASS if ok else FAIL))
        statuses.append(entry["result"])
        parts[ref] = entry
    return write_report("u8_u9_pads", {
        "result": overall(statuses), "tool": "kicad-cli netlist export + scripts/rev3_checks.py u8-u9",
        "netlist": nl.evidence(),
        "statement": "ERC gives no evidence for U8 (generic connector symbol with passive pins). The pad -> net "
                     "table below was transcribed from the datasheet pin tables (source lines cited, re-read by "
                     "this script from the text extracts) and compared with the exported netlist and with the plan.",
        "function_to_net_source": "spec/rev3_plan.json power_block / new_nets (design intent)",
        "not_checked_here": [
            "pads of the .kicad_pcb (the PCB step repeats this table on the board: expected_pad_nets)",
            "second-reader review of the table against the datasheet PDF (plan checks_to_add): NOT_VERIFIED",
            "the package drawings are graphics: the pad-1 corner / counter-clockwise order of the datasheet "
            "figures was not read by a tool, only the pad numbering of the KiCad library footprints was"],
        "parts": parts})


# ----------------------------------------------------------------------------------------------------- fit sets
def fit_report(spec: dict, with_mcu: bool = False) -> dict:
    """Who fits what, derived only from spec ``fit`` / ``bom`` (fitsets.py): the export step writes BOM and CPL
    from ``bom_lines`` and checks its files with ``check_fit_files``."""
    parts = spec["parts"]
    lines = fitsets.bom_groups(parts, with_mcu)
    return {"with_mcu": with_mcu, "prog_headers": spec["prog_headers"]["option"],
            "fit_sets": fitsets.fit_sets(parts), "summary": fitsets.summary(parts, with_mcu), "bom_lines": lines,
            "jlc_designators": sorted((r for g in lines for r in g["designators"]), key=natural)}


def check_fit_files(spec: dict, bom_designators: list[str], cpl_designators: list[str],
                    with_mcu: bool = False) -> dict:
    """set(BOM designators) == set(CPL designators) == JLC-fitted parts of the chosen file pair, no duplicates."""
    want = set(fit_report(spec, with_mcu)["jlc_designators"])
    out = {"with_mcu": with_mcu, "expected": len(want)}
    for name, got in (("bom", list(bom_designators)), ("cpl", list(cpl_designators))):
        out[f"{name}_count"] = len(got)
        out[f"{name}_missing"] = sorted(want - set(got), key=natural)
        out[f"{name}_extra"] = sorted(set(got) - want, key=natural)
        out[f"{name}_duplicates"] = sorted((r for r, n in Counter(got).items() if n > 1), key=natural)
    out["result"] = PASS if not any(out[f"{n}_{k}"] for n in ("bom", "cpl")
                                    for k in ("missing", "extra", "duplicates")) else FAIL
    return out


def check_fit() -> dict:
    spec, plan = load_json(SPEC), load_json(PLAN)
    option = spec["prog_headers"]["option"]
    main, mcu = fit_report(spec, False), fit_report(spec, True)
    problems = []
    # per-part fit against the plan
    plan_fit = {c["refdes"]: c["rev3"]["fit"] for c in plan["changes"] if c.get("rev3")}
    if option == "B":
        plan_fit.update({r: "jlc" for r in R3.PROG_REFS})
    spec_fit = {p["refdes"]: p["fit"] for p in spec["parts"]}
    fit_diffs = [{"ref": r, "plan": plan_fit.get(r), "spec": spec_fit.get(r)}
                 for r in sorted(set(plan_fit) | set(spec_fit), key=natural) if plan_fit.get(r) != spec_fit.get(r)]
    # the plan's explicit "in neither file" list
    not_jlc = sorted((r for r, f in spec_fit.items() if f != "jlc"), key=natural)
    want_not_jlc = sorted((r for r in NOT_JLC_OPTION_A if option == "A" or r not in R3.PROG_REFS), key=natural)
    if not_jlc != want_not_jlc:
        problems.append(f"parts not fitted by JLC {not_jlc} != plan list {want_not_jlc}")
    # BOM lines against the plan
    plan_lines = {b["jlc_code"]: b for b in plan["bom_lines"]}
    line_diffs = []
    for g in main["bom_lines"]:
        b = plan_lines.get(g["jlc_code"])
        if b is None:
            if option == "B" and g["jlc_code"] in R3.OPTION_B_LINES:
                if not set(g["designators"]) <= set(R3.PROG_REFS):
                    line_diffs.append({"jlc_code": g["jlc_code"], "field": "designators (option B)",
                                       "spec": g["designators"]})
                continue
            line_diffs.append({"jlc_code": g["jlc_code"], "field": "line", "plan": None, "spec": g["line"]})
            continue
        for key, a, c in (("line", b["line"], g["line"]), ("comment", b["comment"], g["comment"]),
                          ("footprint", b["footprint"], g["footprint_name"]),
                          ("designators", sorted(b["designators"], key=natural), g["designators"]),
                          ("qty", b["qty_per_board"], g["qty"]), ("assembly", b["assembly_process"], g["assembly"]),
                          ("mpn", b["mpn"], g["mpn"]), ("library_type", b["library_type"], g["library_type"])):
            if a != c:
                line_diffs.append({"jlc_code": g["jlc_code"], "field": key, "plan": a, "spec": c})
    for code in sorted(set(plan_lines) - {g["jlc_code"] for g in main["bom_lines"]}, key=natural):
        line_diffs.append({"jlc_code": code, "field": "line", "plan": plan_lines[code]["line"], "spec": None})
    # the *_with_mcu pair adds exactly the four MCUs on one more line
    extra_refs = sorted(set(mcu["jlc_designators"]) - set(main["jlc_designators"]), key=natural)
    extra_lines = [g for g in mcu["bom_lines"] if g["jlc_code"] not in {m["jlc_code"] for m in main["bom_lines"]}]
    if extra_refs != list(MCUS) or [g["jlc_code"] for g in extra_lines] != [R3.MCU_CODE]:
        problems.append(f"with_mcu pair adds {extra_refs} on lines {[g['jlc_code'] for g in extra_lines]}, "
                        f"expected {list(MCUS)} on {R3.MCU_CODE}")
    # counts
    plan_counts = dict(Counter(plan_fit.values()))
    counts = main["summary"]
    n_plan_lines = len(plan_lines) + (len(R3.OPTION_B_LINES) if option == "B" else 0)
    if counts["fit_counts"] != {f: plan_counts.get(f, 0) for f in fitsets.FITS}:
        problems.append(f"fit counts {counts['fit_counts']} != plan {plan_counts}")
    if counts["bom_lines"] != n_plan_lines:
        problems.append(f"{counts['bom_lines']} BOM lines != plan {n_plan_lines}")
    if counts["jlc_fitted_parts"] != len(main["jlc_designators"]) or \
            counts["jlc_fitted_parts"] != counts["fit_counts"]["jlc"]:
        problems.append("JLC-fitted part count is not the size of the fit set 'jlc'")
    # fit x populate
    cross = Counter(f"{p['fit']}/{'populated' if fitsets.populated(p) else 'not populated'}" for p in spec["parts"])
    for p in spec["parts"]:
        if p["fit"] in ("jlc", "jlc_optional", "user") and not fitsets.populated(p):
            problems.append(f"{p['refdes']}: fit {p['fit']} but populate is not true")
    # netlist: same parts, DNP flag = not populated
    nl = load_netlist()
    netlist = None
    if nl is not None:
        refs = {p["refdes"] for p in spec["parts"]}
        dnp_want = sorted((p["refdes"] for p in spec["parts"] if not fitsets.populated(p)), key=natural)
        dnp_got = sorted((r for r, c in nl.components.items() if c["dnp"]), key=natural)
        netlist = {**nl.evidence(), "only_in_spec": sorted(refs - set(nl.components), key=natural),
                   "only_in_netlist": sorted(set(nl.components) - refs, key=natural),
                   "dnp_in_netlist": dnp_got, "not_populated_in_spec": dnp_want}
        if netlist["only_in_spec"] or netlist["only_in_netlist"] or dnp_got != dnp_want:
            problems.append("netlist components / DNP flags differ from the spec")
    # self-test of the function the export step calls
    good = check_fit_files(spec, main["jlc_designators"], main["jlc_designators"])
    bad1 = check_fit_files(spec, main["jlc_designators"][1:], main["jlc_designators"])
    bad2 = check_fit_files(spec, main["jlc_designators"], main["jlc_designators"] + ["U1"])
    bad3 = check_fit_files(spec, main["jlc_designators"] + main["jlc_designators"][:1], main["jlc_designators"])
    good_mcu = check_fit_files(spec, mcu["jlc_designators"], mcu["jlc_designators"], with_mcu=True)
    selftest = {"equal sets": good["result"], "one designator missing in the BOM": bad1["result"],
                "U1 added to the CPL of the main pair": bad2["result"], "one designator twice": bad3["result"],
                "with_mcu pair, equal sets": good_mcu["result"]}
    if list(selftest.values()) != [PASS, FAIL, FAIL, FAIL, PASS]:
        problems.append(f"check_fit_files self-test: {selftest}")
    result = FAIL if problems or fit_diffs or line_diffs else (PASS if nl is not None else NOT_VERIFIED)
    return write_report("fit_sets", {
        "result": result, "tool": "scripts/rev3_checks.py fit (fitsets.py on spec/carrier_spec.json)",
        "spec": posix(SPEC), "spec_sha256": sha256_file(SPEC), "plan_sha256": sha256_file(PLAN),
        "prog_headers": option, "problems": problems, "fit_differences_vs_plan": fit_diffs,
        "bom_line_differences_vs_plan": line_diffs, "fit_x_populate": dict(sorted(cross.items())),
        "main_pair": main, "with_mcu_pair": {k: mcu[k] for k in ("summary", "jlc_designators")},
        "with_mcu_extra": {"designators": extra_refs, "lines": extra_lines},
        "not_fitted_by_jlc": not_jlc, "netlist": netlist or "NOT_VERIFIED: no netlist",
        "check_fit_files_selftest": selftest,
        "note": "BOM / CPL files do not exist yet (export step); this report is the bookkeeping they must equal"})


# --------------------------------------------------------------------------------------------------------- nets
def check_nets() -> dict:
    nl = load_netlist()
    if nl is None:
        return no_netlist("nets")
    plan = load_json(PLAN)
    problems = []
    new = [{"name": n["name"], "plan": sorted(n["members"], key=natural), "netlist": nl.members(n["name"])}
           for n in plan["new_nets"]]
    for n in new:
        n["netlist"] = sorted(n["netlist"], key=natural)
        n["equal"] = n["plan"] == n["netlist"]
        n["class"] = nl.net_class.get(n["name"])
        if not n["equal"]:
            problems.append(f"new net {n['name']} differs from the plan")
    old = load_netlist(REV2C_NETLIST)
    if old is None:
        return write_report("nets", {"result": NOT_VERIFIED if not problems else FAIL, "new_nets": new,
                                     "problems": problems,
                                     "reason": f"archived rev 2c netlist {posix(REV2C_NETLIST)} not found"})
    added_refs = sorted((c["refdes"] for c in plan["changes"] if c["action"] == "add"), key=natural)
    removed_refs = sorted((c["refdes"] for c in plan["changes"] if c["action"] == "remove"), key=natural)
    a, b = old.named(), nl.named()
    lost, gained = sorted(set(a) - set(b)), sorted(set(b) - set(a))
    if lost:
        problems.append(f"rev 2c nets lost: {lost}")
    if gained != sorted(n["name"] for n in plan["new_nets"]):
        problems.append(f"nets added {gained} != plan new_nets")
    if sorted(lost) != sorted(plan.get("removed_nets", [])):
        problems.append(f"nets removed {lost} != plan removed_nets {plan.get('removed_nets')}")
    changed = []
    for name in sorted(set(a) & set(b)):
        gone = sorted(set(a[name]) - set(b[name]), key=lambda k: (natural(k[0]), natural(k[1])))
        came = sorted(set(b[name]) - set(a[name]), key=lambda k: (natural(k[0]), natural(k[1])))
        if not gone and not came:
            continue
        bad_gone = [f"{r}.{p}" for r, p in gone if r not in removed_refs]
        bad_came = [f"{r}.{p}" for r, p in came if r not in added_refs]
        changed.append({"net": name, "removed_members": [f"{r}.{p}" for r, p in gone],
                        "added_members": [f"{r}.{p}" for r, p in came],
                        "unexpected": bad_gone + bad_came})
        if bad_gone or bad_came:
            problems.append(f"net {name}: unexpected member change {bad_gone + bad_came}")
    for name in gained:
        outside = [f"{r}.{p}" for r, p in b[name] if r not in added_refs]
        if outside:
            problems.append(f"new net {name} has members of existing parts: {outside}")
    ta, tb = old.pin_table(), nl.pin_table()
    nc_a = {f"{r}.{p}" for r, t in ta.items() for p, v in t.items() if v == "NC"}
    nc_b = {f"{r}.{p}" for r, t in tb.items() for p, v in t.items() if v == "NC"}
    nc_removed, nc_added = sorted(nc_a - nc_b, key=natural), sorted(nc_b - nc_a, key=natural)
    if nc_removed != sorted(plan["removed_no_connects"], key=natural) or nc_added:
        problems.append(f"no-connect changes: removed {nc_removed}, added {nc_added}; plan removes "
                        f"{plan['removed_no_connects']}")
    comp_removed = sorted(set(old.components) - set(nl.components), key=natural)
    comp_added = sorted(set(nl.components) - set(old.components), key=natural)
    if comp_removed != removed_refs or comp_added != added_refs:
        problems.append(f"components removed {comp_removed} / added {comp_added} != plan {removed_refs} / "
                        f"{added_refs}")
    class_changes = [{"net": n, "rev2c": old.net_class.get(n), "rev3": nl.net_class.get(n)}
                     for n in sorted(set(a) & set(b)) if old.net_class.get(n) != nl.net_class.get(n)]
    if class_changes:
        problems.append(f"net class changed on {len(class_changes)} rev 2c nets")
    # every pin of every part that exists in both revisions keeps its net
    pin_changes = []
    for ref in sorted(set(ta) & set(tb), key=natural):
        for pin in sorted(set(ta[ref]) | set(tb[ref]), key=natural):
            if ta[ref].get(pin) != tb[ref].get(pin):
                pin_changes.append({"pin": f"{ref}.{pin}", "rev2c": ta[ref].get(pin), "rev3": tb[ref].get(pin)})
    return write_report("nets", {
        "result": FAIL if problems else PASS, "tool": "kicad-cli netlist export + scripts/rev3_checks.py nets",
        "netlist": nl.evidence(), "rev2c_netlist": old.evidence(), "problems": problems, "new_nets": new,
        "named_nets_rev2c": len(a), "named_nets_rev3": len(b), "nets_lost": lost, "nets_added": gained,
        "components_removed": comp_removed, "components_added": comp_added, "nets_with_changed_members": changed,
        "no_connects_rev2c": len(nc_a), "no_connects_rev3": len(nc_b), "no_connects_removed": nc_removed,
        "no_connects_added": nc_added, "net_class_changes_on_rev2c_nets": class_changes,
        "pins_of_kept_parts_with_another_net": pin_changes,
        "note": "pins_of_kept_parts_with_another_net lists every pin of a part present in both revisions whose "
                "net differs (replaced parts with another pin count would appear here); it is information, the "
                "member check above is the pass / fail criterion"})


# -------------------------------------------------------------------------------------------------- determinism
def determinism_files() -> list[Path]:
    return [SPEC, SPEC_KO, KICAD / "ir" / "vtol_carrier.ir.json", MANIFEST, KICAD / "vtol_carrier.kicad_pro",
            *schematic_files()]


def hashes(paths: list[Path]) -> dict[str, str | None]:
    return {posix(p.relative_to(ROOT)): sha256_file(p) for p in paths}


def normalised_netlist(path: Path) -> str:
    """sha256 of a netlist without the export date line of its (design ...) header."""
    text = path.read_text(encoding="utf-8")
    return sha256_bytes(re.sub(r'(?m)^\s*\(date "[^"]*"\)\s*\n', "", text, count=1).encode("utf-8"))


def check_determinism() -> dict:
    """Build spec + schematic twice with the interpreter running this script; all three states must be equal:
    the files on disk before, after build 1 and after build 2."""
    option = load_json(SPEC)["prog_headers"]["option"]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", VTOL_PROG_HEADERS=option)
    env.pop("VTOL_SPEC_DIR", None)
    states = [hashes(determinism_files())]
    commands, problems = [], []
    for n in (1, 2):
        for script in ("build_spec.py", "build_schematic.py"):
            cmd = [sys.executable, "-X", "utf8", "-B", script]
            r = run(cmd, cwd=SCRIPTS, env=env)
            commands.append(f"build {n}: {posix(sys.executable)} -X utf8 -B {script} (cwd {posix(SCRIPTS)}, "
                            f"VTOL_PROG_HEADERS={option}) -> exit {r.returncode}")
            if r.returncode != 0:
                problems.append(f"build {n} {script} exit {r.returncode}: {(r.stderr or r.stdout).strip()[-400:]}")
        states.append(hashes(determinism_files()))
    differing = sorted(k for k in set().union(*states) if len({s.get(k) for s in states}) != 1)
    if differing:
        problems.append(f"{len(differing)} file(s) not byte-identical: {differing}")
    if any(v is None for s in states for v in s.values()):
        problems.append("a file of the set is missing")
    cli = find_kicad_cli()
    netlist = {"status": NOT_VERIFIED,
               "reason": "not run: the file comparison above failed" if problems else "kicad-cli not found"}
    if cli and not problems:
        with tempfile.TemporaryDirectory() as tmp:
            got = []
            for n in (1, 2):
                target = Path(tmp) / f"export{n}.net"
                r = run([cli, "sch", "export", "netlist", "--format", "kicadsexpr", "-o", posix(target), posix(SCH)],
                        cwd=ROOT)
                got.append(normalised_netlist(target) if r.returncode == 0 and target.is_file() else None)
        on_disk = normalised_netlist(NETLIST) if NETLIST.is_file() else None
        same = got[0] is not None and got[0] == got[1] and (on_disk is None or on_disk == got[0])
        netlist = {"status": PASS if same else FAIL, "export_1": got[0], "export_2": got[1],
                   "out_vtol_carrier_net": on_disk,
                   "note": "sha256 of the netlist text without the '(date ...)' line of its design header; the raw "
                           "netlist carries the export time and is evidence, not a deterministic artifact"}
        if not same:
            problems.append("netlist (date line removed) differs between two exports or from out/vtol_carrier.net")
    return write_report("determinism", {
        "result": FAIL if problems else PASS, "tool": "scripts/rev3_checks.py determinism",
        "scope": "spec (carrier_spec.json, SPEC_ko.md), IR, schematic sheets, .kicad_pro, schematic manifest. The "
                 ".kicad_pcb, BOM, CPL and gerbers do not exist for rev 3 yet (later steps).",
        "prog_headers": option, "commands": commands, "problems": problems, "files": len(states[0]),
        "sha256_on_disk_before": states[0], "sha256_after_build_1": states[1], "sha256_after_build_2": states[2],
        "byte_identical": not differing, "netlist_without_date_line": netlist})


# ------------------------------------------------------------------------------------------- JLC order files
BOM_HEADER = ["Comment", "Designator", "Footprint", "JLCPCB Part #\uff08optional\uff09"]
CPL_HEADER = ["Designator", "Mid X", "Mid Y", "Layer", "Rotation"]
#: symbols whose orientation decides the function: each such JLC-fitted part needs its silk mark (routing_rules.json)
POLAR_SYMBOL = re.compile(r"^(Device:C_Polarized|Device:LED|Device:D_Schottky|Transistor_|Regulator_|RF_Module:|"
                          r"Comparator:|MCU_|Connector_Generic:Conn_02x03_Counter_Clockwise|"
                          r"Connector_Generic_MountingPin:)")
KICAD_FP_DIR = Path("C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/share/kicad/footprints")


def read_csv(path: Path) -> tuple[list[str], list[list[str]]]:
    import csv as _csv
    raw = path.read_bytes()
    rows = list(_csv.reader(raw.decode("utf-8-sig").splitlines()))
    return rows[0], rows[1:]


def library_silk_items(lib_id: str) -> int:
    lib, name = lib_id.split(":", 1)
    f = KICAD_FP_DIR / f"{lib}.pretty" / f"{name}.kicad_mod"
    text = f.read_text(encoding="utf-8")
    return len(re.findall(r'\((?:fp_line|fp_poly|fp_circle|fp_arc|fp_rect)[^()]*(?:\([^()]*(?:\([^()]*\)[^()]*)*\)[^()]*)*?\(layer "F\.SilkS"\)', text))


def check_export_files() -> dict:
    spec = load_json(SPEC)
    plan = load_json(PLAN)
    problems, pairs = [], {}
    table_path = OUT / "cpl_rotation_table.csv"
    if not table_path.is_file():
        return write_report("export_files", {"result": NOT_VERIFIED, "reason": "out/cpl_rotation_table.csv missing"})
    th, trows = read_csv(table_path)
    table = {r[0]: dict(zip(th, r)) for r in trows}
    for with_mcu, suffix in ((False, ""), (True, "_with_mcu")):
        bom_p, cpl_p = OUT / f"jlcpcb_bom{suffix}.csv", OUT / f"jlcpcb_cpl{suffix}.csv"
        if not bom_p.is_file() or not cpl_p.is_file():
            problems.append(f"missing {bom_p.name} / {cpl_p.name}")
            continue
        bh, brows = read_csv(bom_p)
        ch, crows = read_csv(cpl_p)
        p: dict = {"bom": posix(bom_p), "cpl": posix(cpl_p), "bom_sha256": sha256_file(bom_p),
                   "cpl_sha256": sha256_file(cpl_p), "bom_lines": len(brows), "cpl_rows": len(crows),
                   "bom_header_exact": bh == BOM_HEADER, "cpl_header_exact": ch == CPL_HEADER,
                   "utf8_bom_prefix": bom_p.read_bytes()[:3] == b"\xef\xbb\xbf"}
        if bh != BOM_HEADER or ch != CPL_HEADER:
            problems.append(f"{suffix or 'main'}: header {bh} / {ch}")
        bom_refs = [r for row in brows for r in row[1].split(",")]
        if any(" " in row[1] for row in brows):
            problems.append(f"{suffix or 'main'}: space in a designator cell")
        codes = Counter(row[3] for row in brows)
        p["duplicate_codes"] = [c for c, n in codes.items() if n > 1]
        p["codes_not_C_number"] = [c for c in codes if not re.fullmatch(r"C\d+", c)]
        fit = check_fit_files(spec, bom_refs, [r[0] for r in crows], with_mcu)
        p["fit_set_check"] = fit
        main_refs = set(fit_report(spec, False)["jlc_designators"])
        if not with_mcu:
            p["forbidden_refs_present"] = sorted(set(bom_refs + [r[0] for r in crows]) & set(NOT_JLC_OPTION_A),
                                                 key=natural)
        else:
            p["added_against_main"] = sorted(set(bom_refs) - main_refs, key=natural)
            if p["added_against_main"] != list(MCUS):
                problems.append(f"with_mcu adds {p['added_against_main']}, expected {list(MCUS)}")
            # user-fitted and unfitted refs are in neither pair (2026-11-12: U7 must not come back with the MCUs)
            p["forbidden_refs_present"] = sorted(set(bom_refs + [r[0] for r in crows])
                                                 & (set(NOT_JLC_OPTION_A) - set(MCUS)), key=natural)
            if p["forbidden_refs_present"]:
                problems.append(f"with_mcu pair contains {p['forbidden_refs_present']}")
        want_lines = len(plan["bom_lines"]) + (1 if with_mcu else 0)
        if len(brows) != want_lines:
            problems.append(f"{suffix or 'main'}: {len(brows)} BOM lines, plan gives {want_lines}")
        bad_rows = []
        for r in crows:
            ok = (re.fullmatch(r"-?\d+\.\d{4}mm", r[1]) and re.fullmatch(r"-?\d+\.\d{4}mm", r[2]) and r[3] == "T"
                  and re.fullmatch(r"\d{1,3}", r[4]) and int(r[4]) < 360)
            t = table.get(r[0])
            same = t is not None and abs(float(r[1][:-2]) - float(t["CPL Mid X (mm)"])) < 5e-5 and \
                abs(float(r[2][:-2]) - float(t["CPL Mid Y (mm)"])) < 5e-5 and \
                (int(r[4]) - float(t["CPL rotation"])) % 360 == 0
            if not ok or not same:
                bad_rows.append(r)
        p["cpl_rows_bad_format_or_not_equal_to_table"] = bad_rows
        p["orientation_critical_not_verified"] = sorted(
            (r for r in fit_report(spec, with_mcu)["jlc_designators"]
             if table[r]["kind"] != "non-polar 2-pad" and table[r]["status"] != "EE_FIT"), key=natural)
        for key in ("duplicate_codes", "codes_not_C_number", "cpl_rows_bad_format_or_not_equal_to_table"):
            if p[key]:
                problems.append(f"{suffix or 'main'}: {key} {p[key][:5]}")
        if fit["result"] != PASS:
            problems.append(f"{suffix or 'main'}: fit-set check FAIL")
        if not with_mcu and p["forbidden_refs_present"]:
            problems.append(f"main pair contains {p['forbidden_refs_present']}")
        pairs["with_mcu" if with_mcu else "main"] = p
    # polarity / pin-1 silk marks of every orientation-deciding JLC-fitted part
    rr_path = REP / "routing_rules.json"
    rr = load_json(rr_path)
    marks = {m["ref"]: m for m in rr["checks"]["silkscreen"]["marks"]}
    parts = {q["refdes"]: q for q in spec["parts"]}
    mcu_refs = fit_report(spec, True)["jlc_designators"]
    # parts the user solders need their pin-1 / polarity mark just as much (2026-11-12: U7 ESP-12F)
    user_refs = sorted((q["refdes"] for q in spec["parts"] if q["fit"] == "user"), key=natural)
    need = sorted((r for r in [*mcu_refs, *user_refs] if POLAR_SYMBOL.match(parts[r]["symbol"])), key=natural)
    silk = {"board_sha256_of_report": rr["board"]["sha256"], "board_sha256_now": sha256_file(KICAD / "vtol_carrier.kicad_pcb"),
            "required": need, "required_user_fitted": [r for r in need if r in user_refs],
            "missing_or_not_ok": [r for r in need if not marks.get(r, {}).get("ok")]}
    # user-soldered parts (2026-11-12: the 12 THT connectors) need the library silkscreen outline as much as the
    # JLC-fitted ones: it is the only orientation reference the user has
    other = sorted((set(mcu_refs) | set(user_refs)) - set(need), key=natural)
    lib_marks = {}
    for r in other:
        if table.get(r, {}).get("kind") == "non-polar 2-pad":
            continue
        lib_marks[r] = library_silk_items(parts[r]["footprint"])
    silk["other_orientation_critical_library_silk_items"] = lib_marks
    silk["other_without_library_silk"] = [r for r, n in lib_marks.items() if n == 0]
    if silk["missing_or_not_ok"] or silk["board_sha256_of_report"] != silk["board_sha256_now"]:
        problems.append(f"silk marks missing / not ok: {silk['missing_or_not_ok']} (or report of another board)")
    if silk["other_without_library_silk"]:
        problems.append(f"no library silkscreen at all: {silk['other_without_library_silk']}")
    # paste apertures: every footprint JLC does not fit in that order has none (report of the export step);
    # 2026-11-12: that includes the user-soldered ESP-12F U7 in BOTH sets
    gc = load_json(REP / "gerber_check.json")
    paste = {}
    for name, with_mcu in (("main", False), ("with_mcu", True)):
        want = sorted((q["refdes"] for q in spec["parts"] if not fitsets.jlc_fitted(q, with_mcu)), key=natural)
        v = (gc.get("paste_variant") or {}).get(name) or {}
        smd_no_paste = sorted(v.get("pads_changed_per_ref", {}), key=natural)
        left = v.get("pads_with_paste_after")
        paste[name] = {"refs_without_paste_in_the_report": v.get("refs"), "refs_not_fitted_by_jlc": want,
                       "refs_that_lost_paste_pads": v.get("pads_changed_per_ref"), "pads_with_paste_left": left,
                       "gerber_flashes": (gc.get("sets") or {}).get(name, {}).get("f_paste_flashes")}
        if v.get("refs") != want:
            problems.append(f"{name}: paste variant refs {v.get('refs')} != parts not fitted by JLC {want}")
        if left is None or left != paste[name]["gerber_flashes"]:
            problems.append(f"{name}: {left} pads keep paste on the variant board, the gerber has "
                            f"{paste[name]['gerber_flashes']} flashes")
        if v.get("paste_pads_left_on_parts_not_fitted_by_jlc") != 0:
            problems.append(f"{name}: paste pads left on parts JLC does not fit: "
                            f"{v.get('paste_pads_left_on_parts_not_fitted_by_jlc')}")
        if not set(smd_no_paste) <= set(want):
            problems.append(f"{name}: paste removed on JLC-fitted parts {sorted(set(smd_no_paste) - set(want))}")
    # the two gerber zips: same members, only F.Paste differs, by exactly the U1..U4 pads
    import zipfile as _zip
    zips = {}
    za, zb = OUT / "vtol_carrier_gerber.zip", OUT / "vtol_carrier_gerber_with_mcu.zip"
    if za.is_file() and zb.is_file():
        with _zip.ZipFile(za) as a, _zip.ZipFile(zb) as b:
            na, nb = sorted(a.namelist()), sorted(b.namelist())
            diff = [n for n in na if n in nb and a.read(n) != b.read(n)]
            fa = len(re.findall(rb"D03\*", a.read("vtol_carrier-F_Paste.gtp")))
            fb = len(re.findall(rb"D03\*", b.read("vtol_carrier-F_Paste.gtp")))
            layers = [n for n in na if n.endswith((".gtl", ".g1", ".g2", ".gbl"))]
        mcu_pads = 64 * len(MCUS)
        zips = {"members_equal": na == nb, "members": na, "differing_members": diff, "f_paste_flashes": [fa, fb],
                "copper_files": layers, "expected_difference": mcu_pads}
        if na != nb or diff != ["vtol_carrier-F_Paste.gtp"] or fb - fa != mcu_pads or len(layers) != 4:
            problems.append(f"gerber zips: {zips['differing_members']} differ, paste flashes {fa} / {fb}, "
                            f"copper files {layers}")
    else:
        problems.append("gerber zips missing")
    result = FAIL if problems else PASS
    return write_report("export_files", {
        "result": result, "tool": "scripts/rev3_checks.py export-files", "spec_sha256": sha256_file(SPEC),
        "problems": problems, "pairs": pairs, "silk_marks": silk, "paste_apertures": paste, "gerber_zips": zips,
        "not_verified": ["JLC's upload parser: header strings, UTF-8 BOM, unit suffix and sign of Y are in the "
                         "format of JLC's sample files (the order page accepted the files of the build before the "
                         "amendments of 2026-11-12, spec/jlc_order_page_20261112.json), but only JLC's upload preview "
                         "of THESE files shows every line matched",
                         "rotation rows listed under orientation_critical_not_verified (and the assumption that JLC's "
                         "0 deg is the EasyEDA footprint) - JLC's 'Confirm Parts Placement'"]})


def check_export_determinism() -> dict:
    """BOM / CPL / both gerber zips written twice into temporary folders (checks_exports.py --files-only) and the
    rotation table rebuilt: all byte-identical to the files in out/."""
    files = ["jlcpcb_bom.csv", "jlcpcb_cpl.csv", "jlcpcb_bom_with_mcu.csv", "jlcpcb_cpl_with_mcu.csv",
             "vtol_carrier_gerber.zip", "vtol_carrier_gerber_with_mcu.zip"]
    base = {f: sha256_file(OUT / f) for f in files}
    runs, problems = [], []
    kpy = "C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/bin/python.exe"
    rot = [OUT / "cpl_rotation_table.csv", OUT / "reports" / "cpl_rotation.json", OUT / "cpl_rotation_check.png"]
    rot_before = {posix(p.relative_to(ROOT)): sha256_file(p) for p in rot}
    r = run([kpy, "-B", "cpl_rotation.py"], cwd=SCRIPTS, env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"))
    rot_after = {posix(p.relative_to(ROOT)): sha256_file(p) for p in rot}
    if r.returncode != 0 or rot_before != rot_after:
        problems.append(f"cpl_rotation.py exit {r.returncode}, outputs changed: "
                        f"{[k for k in rot_before if rot_before[k] != rot_after[k]]}")
    for n in (1, 2):
        with tempfile.TemporaryDirectory() as tmp:
            env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", CARRIER_EXPORT_DIR=tmp)
            r = run([sys.executable, "-X", "utf8", "-B", "checks_exports.py", "--files-only"], cwd=SCRIPTS, env=env)
            got = {f: sha256_file(Path(tmp) / f) for f in files}
            runs.append({"run": n, "exit": r.returncode, "sha256": got})
            if r.returncode != 0:
                problems.append(f"run {n} exit {r.returncode}: {(r.stderr or r.stdout)[-300:]}")
            diff = [f for f in files if got[f] != base[f]]
            if diff:
                problems.append(f"run {n}: not byte-identical to out/: {diff}")
    return write_report("export_determinism", {
        "result": FAIL if problems or any(v is None for v in base.values()) else PASS,
        "tool": "scripts/rev3_checks.py export-determinism", "files": files, "sha256_out": base, "runs": runs,
        "cpl_rotation_outputs": {"before": rot_before, "after": rot_after}, "problems": problems,
        "note": "the gerber / drill / job files carry KiCad's export time; checks_exports.py writes the fixed "
                "RELEASE_STAMP into those fields (comment / attribute lines only), so the zips are comparable"})


# ---------------------------------------------------------------------------------------------------------- CLI
CHECKS = {"export": check_export, "netlist-vs-spec": check_netlist_vs_spec, "mcu-pinmap": check_mcu_pinmap,
          "u8-u9": check_u8_u9, "fit": check_fit, "nets": check_nets, "determinism": check_determinism,
          "export-files": check_export_files, "export-determinism": check_export_determinism}
ALL_ORDER = ("determinism", "export", "netlist-vs-spec", "mcu-pinmap", "u8-u9", "fit", "nets")


def brief(name: str, rep: dict) -> dict:
    keys = {"export": ("tool_version", "errors", "warnings", "sheets_checked", "ignored_checks"),
            "netlist-vs-spec": ("parts", "pins_compared", "pins_connected", "pins_no_connect", "spec_nets",
                                "netlist_named_nets"),
            "mcu-pinmap": ("pins_compared",), "u8-u9": (), "fit": ("prog_headers", "not_fitted_by_jlc"),
            "nets": ("named_nets_rev2c", "named_nets_rev3", "nets_lost", "nets_added", "components_removed",
                     "components_added", "no_connects_removed", "no_connects_added"),
            "determinism": ("files", "byte_identical"), "export-files": ("gerber_zips",),
            "export-determinism": ("sha256_out",)}[name]
    out = {"result": rep["result"], "report": rep["report"], **{k: rep.get(k) for k in keys}}
    if name == "netlist-vs-spec":
        out["differences"] = len(rep.get("pin_differences", [])) + len(rep.get("component_differences", []))
    if name == "mcu-pinmap":
        out["differences"] = len(rep.get("differences", []))
    if name == "u8-u9" and "parts" in rep:
        out["parts"] = {r: p["result"] for r, p in rep["parts"].items()}
    if name == "fit" and "main_pair" in rep:
        out["main_pair"] = rep["main_pair"]["summary"]
        out["with_mcu_pair"] = rep["with_mcu_pair"]["summary"]
    for key in ("problems", "reason"):
        if rep.get(key):
            out[key] = rep[key]
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("check", choices=[*CHECKS, "all"])
    args = ap.parse_args()
    names = ALL_ORDER if args.check == "all" else (args.check,)
    summary = {}
    for name in names:
        summary[name] = brief(name, CHECKS[name]())
    result = overall([s["result"] for s in summary.values()])
    if args.check == "all":
        det = load_json(REP / "rev3_determinism.json")
        erc = load_json(REP / "rev3_erc.json")
        same = erc.get("schematic_sha256") is not None and all(
            det["sha256_after_build_2"].get(f"kicad/{k}") == v for k, v in erc["schematic_sha256"].items())
        summary = write_report("checks", {
            "result": result if same else FAIL,
            "erc_and_netlist_made_from_the_files_of_the_determinism_run": same,
            "interpreter": posix(sys.executable), "python": sys.version.split()[0],
            "spec_sha256": sha256_file(SPEC), "plan_sha256": sha256_file(PLAN),
            "netlist_sha256": sha256_file(NETLIST), "checks": summary})
        result = summary["result"]
    print(json.dumps(summary, indent=1, ensure_ascii=False))
    return {PASS: 0, FAIL: 1, NOT_VERIFIED: 3}[result]


if __name__ == "__main__":
    sys.exit(main())
