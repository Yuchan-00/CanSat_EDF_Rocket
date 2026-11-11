"""JLC CPL rotation / position table for every JLC-fitted part (KiCad python: pcbnew + PIL).

Outputs: out/cpl_rotation_table.csv (read by checks_exports.py - a CPL row without a table row is an error),
out/reports/cpl_rotation.json (details) and out/cpl_rotation_check.png (board top view: each orientation-critical part
drawn as the CPL would place JLC's footprint, pin 1 / + / cathode marked, for comparison with JLC's 'Confirm Parts
Placement' preview).

Invariant this module enforces: a rotation correction is never typed per part. It is measured geometrically:
the KiCad footprint pads (read from the routed board with pcbnew) are matched, pad by pad and by pin FUNCTION, to the
pads of the EasyEDA footprint of the same LCSC code (JLC's own library record, cached read-only under
<SCR>/jlc_cache/easyeda_<code>.json). theta = the multiple of 90 deg (counter-clockwise seen from the top) that turns
the KiCad footprint onto the EasyEDA one; the CPL rotation is then  KiCad rotation - theta  (mod 360) and the CPL
position is the board position of the EasyEDA footprint origin (= the centre of its pad-centre bounding box, measured
on every cached record). Assumption A (stated in every row): JLC's 0 deg is the EasyEDA footprint at 0 deg - only
JLC's placement preview / production file can confirm it, so no row is ever called VERIFIED here.

Parts without an EasyEDA record (EasyEDA answered http 403 after 16 requests on 2026-11-11 - not retried in a loop):
rotation = KiCad rotation + the offset of the third-party Fabrication-Toolkit table (JLC's recommended plugin,
F scout/jlc_rules.md:81-101) when a pattern matches, else + 0; position = KiCad pad-centre bounding-box centre (A,
by analogy with the measured records). Such rows are NOT_VERIFIED.

Polarity cross-checks (2-pad polarised parts): the EasyEDA '+' glyph (two crossing bars on the silk / document layer)
must sit next to the EasyEDA pad that maps to KiCad pad 1 (KiCad Device:C_Polarized: pin 1 = '+'); for diodes / LEDs
the EasyEDA symbol pin names give the cathode pad. A failed or missing cross-check makes the row NOT_VERIFIED.

Run:  cd <board>/scripts && C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/bin/python.exe -B cpl_rotation.py
"""
from __future__ import annotations

import csv
import hashlib
import io
import json
import math
import re
import sys
from pathlib import Path

import pcbnew
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fitsets  # noqa: E402
from rev3_checks import FUNCTION_NET  # noqa: E402  (U8 / U9 datasheet pin function -> intended net)

ROOT = Path(__file__).resolve().parent.parent
BOARD = ROOT / "kicad" / "vtol_carrier.kicad_pcb"
SPEC = ROOT / "spec" / "carrier_spec.json"
OUT_CSV = ROOT / "out" / "cpl_rotation_table.csv"
OUT_JSON = ROOT / "out" / "reports" / "cpl_rotation.json"
OUT_PNG = ROOT / "out" / "cpl_rotation_check.png"
SCR = Path("C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e"
           "/scratchpad/pcb_rev3")
EE_CACHE = SCR / "jlc_cache"
UNIT_MM = 0.254          # EasyEDA canvas unit = 10 mil
FIT_TOL = 0.35           # mm, worst pad-centre error accepted for a theta
BX0, BY0, BX1, BY1 = 62.0, 65.0, 138.0, 135.0

#: EasyEDA pad number -> KiCad pad number where both libraries number the same FUNCTION differently. Every entry is
#: re-checked against the pin names of both symbols (EasyEDA record / spec/library_probe.json) at run time.
_ESP = {str(i): str(i) for i in range(1, 9)}
_ESP.update({str(i): str(i + 6) for i in range(9, 17)})     # EE 9..16 (GND..TXD0) = KiCad 15..22
_ESP.update({str(i): str(i - 8) for i in range(17, 23)})    # EE 17..22 (CS0..SCLK) = KiCad 9..14
PIN_MAP = {
    "C2286": {"1": "2", "2": "1"},                           # EE 1=A 2=K ; KiCad Device:LED 1=K 2=A
    "C6186": {"4": "2"},                                     # EE 4 = tab (VOUT) ; KiCad tab is a second pad "2"
    "C82891": _ESP,
    "C161692": {"5": "MP", "6": "MP"},                       # EE 5 / 6 = mounting pads ; KiCad "MP"
}
#: pin-name aliases used only to compare the two symbols' names (EasyEDA name -> KiCad name)
ALIASES = {"VOUT": "VO", "VIN": "VI", "-": "K", "+": "A", "OSC1": "1", "OSC2": "3", "RXD0": "RXD", "TXD0": "TXD"}
ALIAS_BY_CODE = {"C70562": {"GND": "G"}}
#: third-party fallback table (Fabrication-Toolkit transformations.csv, F scout/jlc_rules.md:85-101)
FT_TABLE = [(r"^CP_EIA-", 180), (r"^CP_Elec_", 180), (r"^C_Elec_", 180), (r"^D_SOT-23", 180), (r"^JST_GH_SM", 180),
            (r"^JST_PH_S", 180), (r"^SOT-143", 180), (r"^SOT-223", 180), (r"^SOT-23", 180), (r"^SOT-353", 180),
            (r"^SOT-363", 180), (r"^SOT-89", 180), (r"^TSOT-23", 180), (r"^TQFP-", 270), (r"^LQFP-", 270),
            (r"^SOIC-", 270), (r"^QFN-", 90), (r"^SC-74-6", 180)]

POLAR_CAP = {"Device:C_Polarized"}
DIODES = {"Device:LED", "Device:D_Schottky", "Device:D"}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rot_ccw(x: float, y: float, deg: float) -> tuple[float, float]:
    """Counter-clockwise as seen from the top, coordinates with Y down (KiCad board frame and EasyEDA canvas)."""
    a = math.radians(deg)
    return x * math.cos(a) + y * math.sin(a), -x * math.sin(a) + y * math.cos(a)


def name_tokens(name: str, code: str) -> set[str]:
    """'GPIO3/RXD' -> {'GPIO3', 'RXD'}; 'PE0(RXD0/PDI)' -> {'PE0', 'RXD', 'PDI'}; aliases applied."""
    n = re.sub(r"~\{|\}|#", "", name or "").strip().upper()
    out = set()
    for t in re.split(r"[()/]", n):
        t = t.strip()
        if t:
            t = ALIAS_BY_CODE.get(code, {}).get(t, t)
            out.add(ALIASES.get(t, t))
    return out


def norm_name(name: str, code: str) -> str:
    toks = sorted(name_tokens(re.split(r"[(/]", name or "")[0], code))
    return toks[0] if toks else ""


def board_box(fp, layer) -> tuple[float, float, float, float] | None:
    boxes = []
    for g in fp.GraphicalItems():
        if g.GetLayer() == layer and g.GetClass() != "PCB_TEXT":
            bb = g.GetBoundingBox()
            boxes.append((pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()), pcbnew.ToMM(bb.GetRight()),
                          pcbnew.ToMM(bb.GetBottom())))
    if not boxes:
        return None
    return (min(b[0] for b in boxes), min(b[1] for b in boxes), max(b[2] for b in boxes), max(b[3] for b in boxes))


def matthewlai_offset(fp_name: str) -> tuple[float | None, str | None]:
    """spec/jlc_cpl_rotations_db.csv (rev 2c table, github matthewlai/JLCKicadTools, A) - reported, not applied."""
    with (ROOT / "spec" / "jlc_cpl_rotations_db.csv").open(encoding="utf-8") as f:
        rd = csv.reader(f, skipinitialspace=True)
        next(rd)
        for r in rd:
            if r and r[0] and re.search(r[0], fp_name):
                return float(r[1]) % 360, r[0]
    return None, None


# --------------------------------------------------------------------------------------------- EasyEDA records
def ee_record(code: str) -> dict | None:
    f = EE_CACHE / f"easyeda_{code}.json"
    if not f.is_file():
        return None
    rec = json.loads(f.read_text(encoding="utf-8"))
    res = (rec.get("response") or {}).get("result") if isinstance(rec.get("response"), dict) else None
    if rec.get("http_status") != 200 or not isinstance(res, dict) or not res.get("packageDetail"):
        return None
    ds = res["packageDetail"]["dataStr"]
    pads, silk, outline = [], [], []
    for sh in ds.get("shape") or []:
        if not isinstance(sh, str):
            continue
        t = sh.split("~")
        if t[0] == "PAD":
            pads.append({"n": t[8], "x": float(t[2]) * UNIT_MM, "y": float(t[3]) * UNIT_MM,
                         "w": float(t[4]) * UNIT_MM, "h": float(t[5]) * UNIT_MM})
        elif t[0] == "SOLIDREGION" and t[1] in ("3", "12", "99"):
            nums = [float(v) for v in re.findall(r"-?\d+\.?\d*", t[3])]
            xs, ys = nums[0::2], nums[1::2]
            if xs and ys:
                item = ("rect", min(xs) * UNIT_MM, min(ys) * UNIT_MM, max(xs) * UNIT_MM, max(ys) * UNIT_MM)
                if t[1] == "99":
                    outline.append(item[1:])
                else:
                    silk.append(item)
        elif t[0] == "TRACK" and t[2] in ("3", "12"):
            nums = [float(v) for v in t[4].split()]
            for i in range(0, len(nums) - 3, 2):
                silk.append(("seg", nums[i] * UNIT_MM, nums[i + 1] * UNIT_MM, nums[i + 2] * UNIT_MM,
                             nums[i + 3] * UNIT_MM))
    pins = []
    for sh in (res.get("dataStr") or {}).get("shape") or []:
        if isinstance(sh, str) and sh.startswith("P~"):
            segs = sh.split("^^")
            head = segs[0].split("~")
            name = segs[3].split("~")[4] if len(segs) > 3 and len(segs[3].split("~")) > 4 else ""
            pins.append((head[3], name))
    head = ds.get("head") or {}
    return {"file": f.as_posix(), "sha256": sha256(f), "fetched_at": rec.get("fetched_at"), "url": rec.get("url"),
            "package": res["packageDetail"].get("title"), "pads": pads, "silk": silk, "pins": pins,
            "outline": (min(o[0] for o in outline), min(o[1] for o in outline), max(o[2] for o in outline),
                        max(o[3] for o in outline)) if outline else None,
            "origin": (float(head.get("x", 0)) * UNIT_MM, float(head.get("y", 0)) * UNIT_MM)}


def plus_glyphs(silk: list) -> list[tuple[float, float]]:
    """Centres of '+' signs: a horizontal and a vertical bar (rectangles or short segments) crossing near their
    middles. Returns EasyEDA canvas mm coordinates."""
    hs, vs = [], []
    for s in silk:
        if s[0] == "rect":
            _, x0, y0, x1, y1 = s
            w, h = x1 - x0, y1 - y0
            if w > 2 * h and w < 2.5:
                hs.append(((x0 + x1) / 2, (y0 + y1) / 2, w))
            elif h > 2 * w and h < 2.5:
                vs.append(((x0 + x1) / 2, (y0 + y1) / 2, h))
        else:
            _, x0, y0, x1, y1 = s
            ln = math.hypot(x1 - x0, y1 - y0)
            if ln < 2.5 and abs(y1 - y0) < 0.02:
                hs.append(((x0 + x1) / 2, y0, ln))
            elif ln < 2.5 and abs(x1 - x0) < 0.02:
                vs.append((x0, (y0 + y1) / 2, ln))
    out = []
    for hx, hy, hl in hs:
        for vx, vy, vl in vs:
            if abs(hx - vx) < 0.25 * hl + 0.05 and abs(hy - vy) < 0.25 * vl + 0.05:
                out.append(((hx + vx) / 2, (hy + vy) / 2))
    return out


# -------------------------------------------------------------------------------------------------- geometry
def centre(pts: list[tuple[float, float]]) -> tuple[float, float]:
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2


def fit_theta(kpads: list[dict], epads: list[dict]) -> list[dict]:
    """kpads / epads already centred and numbered by KiCad pad number."""
    res = []
    for th in (0, 90, 180, 270):
        worst, n = 0.0, 0
        for kp in kpads:
            cands = [e for e in epads if e["n"] == kp["n"]]
            if not cands:
                continue
            x, y = rot_ccw(kp["x"], kp["y"], th)
            worst = max(worst, min(math.hypot(x - e["x"], y - e["y"]) for e in cands))
            n += 1
        res.append({"theta_ccw": th, "worst_pad_centre_error_mm": round(worst, 3), "pads_matched": n})
    return res


def ft_offset(fp_name: str) -> tuple[int, str | None]:
    for pat, off in FT_TABLE:
        if re.search(pat, fp_name):
            return off, pat
    return 0, None


# ------------------------------------------------------------------------------------------------------- main
def main() -> int:
    spec = json.loads(SPEC.read_text(encoding="utf-8"))
    probe = json.loads((ROOT / "spec" / "library_probe.json").read_text(encoding="utf-8"))
    parts = {p["refdes"]: p for p in spec["parts"]}
    lines = fitsets.bom_groups(spec["parts"], with_mcu=True)
    code_of = {r: g["jlc_code"] for g in lines for r in g["designators"]}
    main_refs = {r for g in fitsets.bom_groups(spec["parts"], with_mcu=False) for r in g["designators"]}
    board = pcbnew.LoadBoard(str(BOARD))
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    ee_cache: dict[str, dict | None] = {}
    rows, details, problems = [], {}, []
    for ref in sorted(code_of, key=fitsets.natural):
        p, fp, code = parts[ref], fps[ref], code_of[ref]
        fp_name = p["footprint"].split(":", 1)[1]
        if fp.GetFPID().GetUniStringLibItemName() != fp_name:
            problems.append(f"{ref}: board footprint {fp.GetFPID().GetUniStringLibItemName()} != spec {fp_name}")
        if fp.GetLayer() != pcbnew.F_Cu:
            problems.append(f"{ref}: not on the top side")
        phi = fp.GetOrientationDegrees() % 360
        px, py = pcbnew.ToMM(fp.GetPosition().x), pcbnew.ToMM(fp.GetPosition().y)
        kp = []
        for pad in fp.Pads():
            num = pad.GetNumber()
            if not num:
                continue
            lx, ly = pcbnew.ToMM(pad.GetFPRelativePosition().x), pcbnew.ToMM(pad.GetFPRelativePosition().y)
            bx, by = rot_ccw(lx, ly, phi)
            gx, gy = pcbnew.ToMM(pad.GetPosition().x), pcbnew.ToMM(pad.GetPosition().y)
            if math.hypot(px + bx - gx, py + by - gy) > 0.002:
                problems.append(f"{ref}.{num}: local pad transform does not reproduce the board pad position")
            kp.append({"n": num, "x": lx, "y": ly, "gx": gx, "gy": gy})
        sym = p["symbol"]
        kpins = {a[0]: a[1] for a in probe["symbols"][sym]["pins"]}
        polar = sym in POLAR_CAP or sym in DIODES
        n_pads = len({k["n"] for k in kp})
        orient_critical = polar or n_pads > 2 or sym.startswith("Connector")
        if code not in ee_cache:
            ee_cache[code] = ee_record(code)
        ee = ee_cache[code]
        row = {"designator": ref, "jlc_code": code, "footprint": fp_name, "pair": "main" if ref in main_refs
               else "with_mcu only", "kind": ("polarised" if polar else "orientation-critical" if orient_critical
                                              else "non-polar 2-pad"),
               "kicad_rotation": round(phi, 3), "kicad_origin_mm": [round(px, 4), round(py, 4)]}
        notes: list[str] = []
        status = "NOT_VERIFIED"
        if ee:
            pm = PIN_MAP.get(code, {})
            # pin-function check: EasyEDA symbol pin names vs KiCad symbol pin names, through the map
            name_rows, name_bad = [], []
            pad_net = {pd.GetNumber(): pd.GetNetname() for pd in fp.Pads()}
            for en, ename in ee["pins"]:
                kn = pm.get(en, en)
                kname = kpins.get(kn) if kn != "MP" else "MountPin"
                a, b = name_tokens(ename, code), name_tokens(kname or "", code)
                informative = (bool(a) and a != {en} and bool(b) and b != {kn} and "MOUNTPIN" not in b
                               and not any(re.fullmatch(r"PIN_\d+", t) for t in b) and ref not in FUNCTION_NET)
                how = "compared" if informative else "not informative"
                if informative and not (a & b):
                    name_bad.append(f"EE {en} {ename!r} -> KiCad {kn} {kname!r}")
                if ref in FUNCTION_NET:     # generic / substitute symbol: the datasheet function must land on its net
                    want = FUNCTION_NET[ref].get(ename.strip().upper())
                    how = f"function {ename} -> net {pad_net.get(kn)} (intended {want})"
                    informative = True
                    if want is None or pad_net.get(kn) != want:
                        name_bad.append(f"EE {en} {ename!r} -> KiCad pad {kn} on net {pad_net.get(kn)}, intended {want}")
                name_rows.append([en, ename, kn, kname, how])
            compared = sum(1 for r in name_rows if r[4] != "not informative")
            row["pin_function_rows"] = name_rows
            epads_all = [dict(e, n=pm.get(e["n"], e["n"])) for e in ee["pads"]]
            common = {k["n"] for k in kp} & {e["n"] for e in epads_all}
            kc = centre([(k["x"], k["y"]) for k in kp if k["n"] in common])
            ec = centre([(e["x"], e["y"]) for e in epads_all if e["n"] in common])
            kcen = [{"n": k["n"], "x": k["x"] - kc[0], "y": k["y"] - kc[1]} for k in kp]
            ecen = [{"n": e["n"], "x": e["x"] - ec[0], "y": e["y"] - ec[1]} for e in epads_all]
            fits = fit_theta(kcen, ecen)
            ok = [f for f in fits if f["pads_matched"] == len(kp) and f["worst_pad_centre_error_mm"] <= FIT_TOL]
            if not polar and n_pads == 2:
                ok = ok[:1] or [f for f in fits if f["worst_pad_centre_error_mm"] <= FIT_TOL][:1]
            theta = ok[0]["theta_ccw"] if len(ok) == 1 else None
            if len(ok) > 1 and not orient_critical:
                theta = ok[0]["theta_ccw"]
            row.update({"easyeda_package": ee["package"], "easyeda_record": ee["file"],
                        "easyeda_fetched_at": ee["fetched_at"], "theta_fit": fits,
                        "pin_names_compared": compared, "pin_name_mismatches": name_bad})
            if name_bad:
                notes.append("pin-name mismatch: " + "; ".join(name_bad[:4]))
            if theta is None:
                notes.append(f"no unique 90-deg fit within {FIT_TOL} mm: " + json.dumps(fits))
            else:
                kcb = rot_ccw(kc[0], kc[1], phi)
                kcb = (px + kcb[0], py + kcb[1])
                fab = board_box(fp, pcbnew.F_Fab)

                def body_cmp(th: int) -> tuple[float, float, float] | None:
                    if not (ee["outline"] and fab):
                        return None
                    o = ee["outline"]
                    ex, ey = rot_ccw((o[0] + o[2]) / 2 - ec[0], (o[1] + o[3]) / 2 - ec[1], (phi - th) % 360)
                    kx, ky = (fab[0] + fab[2]) / 2 - kcb[0], (fab[1] + fab[3]) / 2 - kcb[1]
                    me, mk = math.hypot(ex, ey), math.hypot(kx, ky)
                    if me <= 0.3 or mk <= 0.3:
                        return me, mk, -1.0
                    return me, mk, math.degrees(math.acos(max(-1.0, min(1.0, (ex * kx + ey * ky) / (me * mk)))))

                # single-row connectors have identical pins: the pin NUMBERING of two libraries may be mirrored
                # (measured: the Hanxia 1x2 / 1x3 headers and the Molex SPOX record). The physical criterion is the
                # body: if the number fit puts JLC's body on the other side of the pads than the KiCad F.Fab body,
                # turn by 180 deg when the pad SET still fits - then the body lands where the board expects it.
                bc = body_cmp(theta)
                if sym.startswith("Connector_Generic:Conn_01x") and bc and bc[2] > 45:
                    alt = (theta + 180) % 360
                    worst = 0.0
                    for k in kcen:
                        x, y = rot_ccw(k["x"], k["y"], alt)
                        worst = max(worst, min(math.hypot(x - e["x"], y - e["y"]) for e in ecen))
                    bc2 = body_cmp(alt)
                    if worst <= FIT_TOL and bc2 and 0 <= bc2[2] <= 45:
                        notes.append(f"pin numbering of the EasyEDA record is mirrored against KiCad (number fit "
                                     f"{theta} deg puts the body on the wrong side); body-matched {alt} deg used, "
                                     f"pad set fits within {worst:.3f} mm - identical pins, so the numbering does not "
                                     "matter, the body side does")
                        row["numbering_mirrored"] = True
                        theta = alt
                cpl = (phi - theta) % 360
                ox, oy = rot_ccw(ee["origin"][0] - ec[0], ee["origin"][1] - ec[1], cpl)
                mid = (kcb[0] + ox, kcb[1] + oy)
                row.update({"theta_ccw": theta, "correction_deg": (-theta) % 360, "cpl_rotation": round(cpl, 3),
                            "mid_board_mm": [round(mid[0], 4), round(mid[1], 4)],
                            "source": "EasyEDA footprint record (JLC library) " + ee["file"].split("/")[-1]
                                      + f" fetched {ee['fetched_at']}; pad fit {[f for f in fits if f['theta_ccw'] == theta][0]['worst_pad_centre_error_mm']} mm"})
                # EasyEDA footprint in board frame (for the drawing and the pin-1 / mark positions)
                row["ee_pads_board"] = []
                for e in epads_all:
                    x, y = rot_ccw(e["x"] - ec[0], e["y"] - ec[1], cpl)
                    row["ee_pads_board"].append({"n": e["n"], "x": round(kcb[0] + x, 3), "y": round(kcb[1] + y, 3)})
                if ee["outline"]:
                    o = ee["outline"]
                    row["ee_outline_board"] = []
                    for cx_, cy_ in ((o[0], o[1]), (o[2], o[1]), (o[2], o[3]), (o[0], o[3])):
                        x, y = rot_ccw(cx_ - ec[0], cy_ - ec[1], cpl)
                        row["ee_outline_board"].append([round(kcb[0] + x, 3), round(kcb[1] + y, 3)])
                if sym in POLAR_CAP:
                    for gx_, gy_ in plus_glyphs(ee["silk"])[:1]:
                        x, y = rot_ccw(gx_ - ec[0], gy_ - ec[1], cpl)
                        row["ee_plus_board"] = [round(kcb[0] + x, 3), round(kcb[1] + y, 3)]
                status = "EE_FIT"
                # body side: EasyEDA component outline (layer 99) vs KiCad F.Fab box, both relative to the pads
                bc = body_cmp(theta)
                if bc is None:
                    row["body_side_check"] = "no EasyEDA outline or no KiCad F.Fab graphics"
                elif bc[2] >= 0:
                    row["body_side_check"] = (f"body offset from the pad centre: EasyEDA outline {bc[0]:.2f} mm, "
                                              f"KiCad F.Fab {bc[1]:.2f} mm, angle between {bc[2]:.0f} deg")
                    if bc[2] > 45:
                        status = "NOT_VERIFIED"
                        notes.append("EasyEDA body outline lies on another side of the pads than the KiCad F.Fab "
                                     "body: JLC would place the body turned")
                else:
                    row["body_side_check"] = (f"body centred on the pads (EasyEDA {bc[0]:.2f} mm, KiCad {bc[1]:.2f} "
                                              "mm, <= 0.3 mm): no side to compare")
                # third-party tables (reported; a disagreement with the JLC-recommended Fabrication-Toolkit table
                # makes the row NOT_VERIFIED: one of the two is wrong and only JLC's preview can tell)
                ft, ftp = ft_offset(fp_name)
                ml, mlp = matthewlai_offset(fp_name)
                corr = (-theta) % 360
                row["third_party_tables"] = {"fabrication_toolkit": [ftp, ft] if ftp else None,
                                             "matthewlai_rev2c": [mlp, ml] if mlp else None}
                same = (lambda a, b: (a - b) % 360 == 0) if (orient_critical or polar) else \
                    (lambda a, b: (a - b) % 180 == 0)
                if ftp and not same(ft, corr):
                    status = "NOT_VERIFIED"
                    notes.append(f"CONFLICT: EasyEDA fit gives correction {corr:g} deg, Fabrication-Toolkit table "
                                 f"{ftp} gives {ft:g} deg")
                if mlp and not same(ml, corr):
                    notes.append(f"rev 2c table {mlp} gives {ml:g} deg (not used)")
                # polarity cross-checks
                if sym in POLAR_CAP:
                    plus = plus_glyphs(ee["silk"])
                    if plus:
                        gx, gy = plus[0]
                        near = min(epads_all, key=lambda e: math.hypot(e["x"] - gx, e["y"] - gy))
                        row["polarity_check"] = (f"EasyEDA '+' glyph at the EasyEDA pad mapped to KiCad pad "
                                                 f"{near['n']} (KiCad Device:C_Polarized pin 1 = '+')")
                        if near["n"] != "1":
                            status = "NOT_VERIFIED"
                            notes.append("EasyEDA '+' glyph is NOT next to KiCad pad 1 - reversed polarity")
                    else:
                        status = "NOT_VERIFIED"
                        row["polarity_check"] = "no '+' glyph found in the EasyEDA footprint"
                elif sym in DIODES:
                    kname = {kn: kpins.get(kn) for kn in kpins}
                    cath_ee = [en for en, nm in ee["pins"] if norm_name(nm, code) == "K"]
                    cath_k = [kn for kn, nm in kname.items() if nm == "K"]
                    mapped = [pm.get(en, en) for en in cath_ee]
                    row["polarity_check"] = (f"cathode: EasyEDA symbol pin {cath_ee} -> KiCad pad {mapped}; KiCad "
                                             f"symbol cathode pad {cath_k}")
                    if not cath_ee or mapped != cath_k:
                        status = "NOT_VERIFIED"
                        notes.append("cathode pad of the EasyEDA record does not map to the KiCad cathode pad")
                if name_bad:
                    status = "NOT_VERIFIED"
        if "cpl_rotation" not in row:
            off, pat = ft_offset(fp_name)
            cpl = (phi + off) % 360
            kc = centre([(k["gx"], k["gy"]) for k in kp])
            row.update({"theta_ccw": None, "correction_deg": off, "cpl_rotation": round(cpl, 3),
                        "mid_board_mm": [round(kc[0], 4), round(kc[1], 4)],
                        "source": (f"no EasyEDA record ({'not cached' if not ee else 'no fit'}); Fabrication-Toolkit "
                                   f"pattern {pat} +{off} (third-party table, A)" if pat else
                                   f"no EasyEDA record ({'not cached' if not ee else 'no fit'}); no table pattern: "
                                   "KiCad rotation unchanged (A)")})
            status = "NOT_VERIFIED"
        # KiCad pad 1 in the board frame (for the drawing)
        p1 = [k for k in kp if k["n"] == "1"]
        row["kicad_pad1_board"] = [round(p1[0]["gx"], 3), round(p1[0]["gy"], 3)] if p1 else None
        row["kicad_pads_board"] = [{"n": k["n"], "x": round(k["gx"], 3), "y": round(k["gy"], 3)} for k in kp]
        if row.get("ee_pads_board") and p1:
            e1 = [e for e in row["ee_pads_board"] if e["n"] == "1"]
            row["pad1_offset_mm"] = round(math.hypot(e1[0]["x"] - p1[0]["gx"], e1[0]["y"] - p1[0]["gy"]), 3) if e1 else None
        row["mid_offset_from_kicad_origin_mm"] = round(math.hypot(row["mid_board_mm"][0] - px,
                                                                  row["mid_board_mm"][1] - py), 3)
        if not row["kind"].startswith("non-polar") or status != "EE_FIT":
            pass
        row["status"] = status
        row["notes"] = notes
        rows.append(row)
        details[ref] = row
    # ---------------------------------------------------------------- CSV
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(["Designator", "JLC code", "pair", "KiCad footprint", "kind", "KiCad rotation", "theta EasyEDA",
                "correction applied (deg)", "CPL rotation", "KiCad origin X (mm)", "KiCad origin Y (mm)",
                "CPL Mid X (mm)", "CPL Mid Y (mm)", "Mid - origin (mm)", "status", "source", "polarity check",
                "notes"])
    for r in rows:
        w.writerow([r["designator"], r["jlc_code"], r["pair"], r["footprint"], r["kind"], f"{r['kicad_rotation']:g}",
                    "" if r["theta_ccw"] is None else r["theta_ccw"], f"{r['correction_deg']:g}",
                    f"{r['cpl_rotation']:g}", f"{r['kicad_origin_mm'][0]:.4f}", f"{-r['kicad_origin_mm'][1]:.4f}",
                    f"{r['mid_board_mm'][0]:.4f}", f"{-r['mid_board_mm'][1]:.4f}",
                    f"{r['mid_offset_from_kicad_origin_mm']:.3f}", r["status"], r["source"],
                    r.get("polarity_check", ""), " | ".join(r["notes"])])
    OUT_CSV.write_bytes(buf.getvalue().encode("utf-8"))
    # ---------------------------------------------------------------- PNG
    draw_png(rows)
    crit = [r for r in rows if r["kind"] != "non-polar 2-pad"]
    summary = {
        "rows": len(rows), "main_pair_rows": sum(1 for r in rows if r["pair"] == "main"),
        "orientation_critical_rows": len(crit),
        "orientation_critical_ee_fit": sorted((r["designator"] for r in crit if r["status"] == "EE_FIT"),
                                              key=fitsets.natural),
        "orientation_critical_not_verified": sorted((r["designator"] for r in crit if r["status"] != "EE_FIT"),
                                                    key=fitsets.natural),
        "non_polar_ee_fit": sum(1 for r in rows if r["kind"] == "non-polar 2-pad" and r["status"] == "EE_FIT"),
        "non_polar_not_verified": sorted((r["designator"] for r in rows if r["kind"] == "non-polar 2-pad"
                                          and r["status"] != "EE_FIT"), key=fitsets.natural),
        "rows_with_mid_offset_over_0.05mm": sorted((f"{r['designator']} {r['mid_offset_from_kicad_origin_mm']}"
                                                    for r in rows if r["mid_offset_from_kicad_origin_mm"] > 0.05),
                                                   key=fitsets.natural),
    }
    doc = {"tool": "scripts/cpl_rotation.py (pcbnew " + pcbnew.GetBuildVersion() + ")",
           "board": {"file": BOARD.as_posix(), "sha256": sha256(BOARD)}, "spec_sha256": sha256(SPEC),
           "assumption_A": "JLC's 0 deg of a part = its EasyEDA footprint at 0 deg and JLC places the EasyEDA "
                           "footprint origin at Mid X / Mid Y; rotation positive = counter-clockwise seen from the top "
                           "(F jlc_rules.md:66). Confirmed only by JLC's placement preview / production file.",
           "status_meaning": {"EE_FIT": "theta measured from JLC's EasyEDA footprint record (every pad matched by pin "
                                        "function within 0.35 mm, polarity cross-check passed); still NOT confirmed "
                                        "by JLC's preview", "NOT_VERIFIED": "no record / no unique fit / cross-check "
                                        "failed: the rotation is a guess - check this part in the preview"},
           "problems": problems, "summary": summary, "rows": details}
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JSON.write_text(json.dumps(doc, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"problems": problems, "summary": summary}, indent=1, ensure_ascii=False))
    for r in crit:
        print(f"  {r['designator']:<5} {r['jlc_code']:<10} kicad {r['kicad_rotation']:>6g} theta {r['theta_ccw']!s:>4} "
              f"cpl {r['cpl_rotation']:>6g} {r['status']:<12} {r.get('polarity_check', '')[:70]} {' | '.join(r['notes'])[:120]}")
    return 1 if problems else 0


def draw_png(rows: list[dict]) -> None:
    s = 16.0
    m = 40
    W, H = int((BX1 - BX0) * s) + 2 * m, int((BY1 - BY0) * s) + 2 * m + 150
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("arial.ttf", 12)
        big = ImageFont.truetype("arial.ttf", 16)
    except OSError:
        font = big = ImageFont.load_default()

    def P(x: float, y: float) -> tuple[float, float]:
        return (m + (x - BX0) * s, m + (y - BY0) * s)

    d.rectangle([P(BX0, BY0), P(BX1, BY1)], outline=(30, 120, 30), width=3)
    for r in rows:
        crit = r["kind"] != "non-polar 2-pad"
        col = (150, 150, 150) if not crit else ((30, 70, 200) if r["status"] == "EE_FIT" else (230, 120, 0))
        pads = r.get("ee_pads_board") or r["kicad_pads_board"]
        xs, ys = [q["x"] for q in pads], [q["y"] for q in pads]
        if crit and r.get("ee_outline_board"):
            d.polygon([P(*c) for c in r["ee_outline_board"]], outline=col, fill=(225, 232, 250) if
                      r["status"] == "EE_FIT" else (252, 232, 210))
        else:
            d.rectangle([P(min(xs) - 0.4, min(ys) - 0.4), P(max(xs) + 0.4, max(ys) + 0.4)], outline=col, width=1)
        if r.get("ee_plus_board"):
            x, y = P(*r["ee_plus_board"])
            d.text((x - 4, y - 8), "+", fill=(200, 0, 0), font=big)
        for q in pads:
            x, y = P(q["x"], q["y"])
            rr = 2.5 if crit else 1.5
            fill = (220, 0, 0) if q["n"] == "1" and crit else None
            d.ellipse([x - rr, y - rr, x + rr, y + rr], outline=col, fill=fill)
        if crit and r.get("kicad_pad1_board"):
            x, y = P(*r["kicad_pad1_board"])
            d.ellipse([x - 5, y - 5, x + 5, y + 5], outline=(0, 160, 0), width=2)
        if crit:
            cx, cy = P(*r["mid_board_mm"])
            # rotation arrow: JLC 0 deg -> +x; CPL rotation counter-clockwise (screen y down)
            a = math.radians(r["cpl_rotation"])
            d.line([cx, cy, cx + 14 * math.cos(a), cy - 14 * math.sin(a)], fill=col, width=2)
            lab = f"{r['designator']} {r['cpl_rotation']:g}"
            if r["status"] != "EE_FIT":
                lab += " ?"
            d.text((cx + 4, cy + 4), lab, fill=col, font=font)
    y0 = m + (BY1 - BY0) * s + 14
    nv = [r["designator"] for r in rows if r["kind"] != "non-polar 2-pad" and r["status"] != "EE_FIT"]
    mir = [r["designator"] for r in rows if r.get("numbering_mirrored")]
    d.text((m, y0), "CPL check (top view, KiCad board frame, Y down). Shaded box = body outline of JLC's EasyEDA "
                    "footprint as the CPL places it; blue = rotation measured from that footprint (A: JLC 0 deg = "
                    "EasyEDA 0 deg)", fill=(30, 70, 200), font=font)
    d.text((m, y0 + 20), "orange '?' = NOT_VERIFIED (no unique fit, or the EasyEDA fit conflicts with the third-party "
                         "rotation table: " + ", ".join(nv) + ") - check these in JLC's 'Confirm Parts Placement' "
                         "preview", fill=(230, 120, 0), font=font)
    d.text((m, y0 + 40), "red dot = EasyEDA pad 1 as the CPL places it; red '+' = EasyEDA '+' mark of the polarised "
                         "capacitors; green ring = KiCad pad 1 on the board. Red inside green = same pin 1.",
           fill=(160, 0, 0), font=font)
    # since 2026-11-12 the single-row THT connectors (the parts with mirrored numbering) are not placed by JLC
    d.text((m, y0 + 60), (", ".join(mir) + ": red dot at the far end on purpose - the EasyEDA pin numbering is "
                          "mirrored, the body side was matched instead (identical pins).") if mir else
           "no placed part has mirrored pin numbering (the single-row THT connectors are soldered by the user).",
           fill=(160, 0, 0), font=font)
    d.text((m, y0 + 80), "line from the part centre = direction of the CPL rotation (0 deg points to +x, counter-"
                         "clockwise positive); grey = non-polar 2-pad parts", fill=(80, 80, 80), font=font)
    d.text((m, y0 + 100), "U1..U4 appear only in the *_with_mcu pair. Nothing here is VERIFIED: JLC's preview / "
                          "production file decides.", fill=(80, 80, 80), font=font)
    buf = io.BytesIO()
    img.save(buf, format="PNG", optimize=False)
    OUT_PNG.write_bytes(buf.getvalue())


if __name__ == "__main__":
    sys.exit(main())
