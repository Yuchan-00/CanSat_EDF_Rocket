"""Route the board with Freerouting (run with KiCad's own python: it needs ``pcbnew``).

Stages (recorded in kicad/work/route_log.json). Each stage exports a Specctra DSN of the current board
(pcbnew.ExportSpecctraDSN), runs ``java -jar freerouting-2.5.0.jar -de <dsn> -do <ses> -mp <passes> -mt <threads>
--gui.enabled=false`` (headless), imports the session (pcbnew.ImportSpecctraSES replaces the tracks of every net in
the session) and LOCKS everything routed so far:
  servo       SERVO_5V / GND_PWR alone, only the 1.2 / 0.6 mm via padstack in the DSN
  power_high  VBAT_LOGIC* alone, only the 0.8 / 0.4 mm via
  power       +5V / +3V3 / GPS_VCC / *_AVCC alone, only the 0.6 / 0.3 mm via
  signals     every other net except GND (GND pads stay obstacles)
  gnd         GND, with the B.Cu GND pour kept in the DSN as a plane: Freerouting fans F.Cu GND pads out to it
  complete    every net (B.Cu GND plane kept), only when connections are still open
Why (scratch experiments, M): routing every net at once left 113-166 connections open (44 +5V, 14 +3V3 of 77 after
the pour fill); with the power nets routed alone first they connect. Nets not routed in a stage are split into
single-pin nets (their pads keep a net and stay clearance obstacles; removed nets become "netless" pins Freerouting
routes over); fixed wiring of split nets (the locked pre-routes) becomes keep-outs. One via padstack per power stage:
with all three, servo nets got 0.6 mm-class vias that KiCad gave the 0.6 mm servo drill (0.05 mm rings).
After the last stage: vias shrunk back from the DSN inflation (VIA_INFLATE_MM keeps hole edges >= 0.5 mm apart,
review F6), neck-downs below 0.127 mm widened, power / servo neck-downs widened back toward their class width where
the clearance allows (rev 2c, rev-2b review PF-M6), dangling vias removed, zones filled, GND stitching vias added
(never in or on a pad, rev 2c, review PF-M3), filled again, silkscreen labels placed (silk.py), dangling track stubs
that kicad-cli DRC reports removed (rev 2c, review PF-m5); saved as kicad/vtol_carrier.kicad_pcb.

4 layers (default, common.LAYERS): In1.Cu = GND plane, In2.Cu = +5V plane (CARRIER_IN2=plane); fanout.py first puts a
via + stub at every GND / +5V SMD pad; tracks only on F.Cu / B.Cu (per-class use_layer in the DSN); the GND stage keeps
both planes in the DSN; board-edge keepouts give the 0.3 mm copper-to-edge rule.

Freerouting is not deterministic (multi-threaded). Every stage session is kept (kicad/work/vtol_carrier_<stage>.ses)
with the stage order in kicad/work/ses_replay.json; ``--reuse-ses`` replays them (same fan-out, imports, via shrink,
locking, finishing) on a freshly generated unrouted board - that path is deterministic (scripts/determinism.py).
rev 2c "polish" stage (``--reuse-ses --polish [passes] [threads]``): the recorded stages are replayed, then every
track / via EXCEPT the fixed copper (build_pcb pre-routes + fanout.py stubs / vias, matched by geometry) is unlocked
and one more Freerouting run gets all nets with the existing routes as rip-up-able wiring (planes kept as in the
complete stage). Locked stages cannot give way for the last few connections (M: rev 2c live runs ended at 2 open,
both in the FCU top area); the polish session is appended to ses_replay.json, so the replay stays deterministic.
Usage: python route.py [passes] [threads] | python route.py --reuse-ses [--polish passes threads]
"""

from __future__ import annotations

import hashlib
import json
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pcbnew

ROOT = Path(__file__).resolve().parent.parent
#: CARRIER_KICAD_DIR: run on a scratch copy (experiments); default = the design folder
import os  # noqa: E402

KICAD = Path(os.environ["CARRIER_KICAD_DIR"]) if os.environ.get("CARRIER_KICAD_DIR") else ROOT / "kicad"
WORK = KICAD / "work"
#: Freerouting 2.5.0 jar: environment override FREEROUTING_JAR, else the copy already on this PC
#: (sha256 f6f51bb02245e8e717f9359bd260cc9c5c0b1bc0acc8b7cb2cd5b8ffeb5de3c7, 64,599,107 bytes, measured 2026-11-10;
#: the hash of the jar actually used is written to kicad/work/route_log.json). Never downloaded by this script.
JAR = Path(os.environ.get("FREEROUTING_JAR") or
           r"C:/Users/yc012/AppData/Local/Temp/claude/B--Claude/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/tools_dl/freerouting/freerouting-2.5.0.jar")
BOARD = KICAD / "vtol_carrier.kicad_pcb"
#: kicad-cli used for the DRC-driven clean-up steps: environment KICAD_CLI, else the KiCad 10 install of this PC
KICAD_CLI = Path(os.environ.get("KICAD_CLI") or r"C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/bin/kicad-cli.exe")
MIN_W = pcbnew.FromMM(0.127)
VIA_INFLATE_MM = 0.1
#: rev 3: nets that build_pcb.power_preroutes wires completely (battery path J14 -> Q10 -> J15 -> L5 with every tap,
#: and the switch node). They are checked for completeness on the unrouted board and then never handed to
#: Freerouting as nets: in every DSN they are split into single-pin nets and their copper becomes keep-outs
#: (keep_nets), and no session wire of theirs is imported. M: as routable nets the 1.0 mm / 0.8 mm-via PowerHigh
#: class left 5 tap connections open (attempt a1); as fixed wiring of a complete net Freerouting still counted
#: "VBAT_LOGIC_SW (8 unrouted connections)" (wide tracks / T-junctions) and kept trying in every pass (attempt a2).
PREROUTED_NETS = ("VBAT_LOGIC_RAW", "VBAT_LOGIC", "VBAT_LOGIC_SW", "BOOST_SW")
#: rev 3 ECO seed: the recorded rev 2c stage sessions (archive). Nets whose pads are identical on the rev 2c and the
#: rev 3 unrouted board start with their rev 2c routing wherever kicad-cli DRC finds no conflict with rev 3 copper.
SEED_DIR = Path(os.environ["CARRIER_SEED_DIR"]) if os.environ.get("CARRIER_SEED_DIR") else \
    ROOT / "archive" / "rev2c_pololu_1s" / "kicad" / "work"
#: geometry keys of the copper that was locked before any routing (pre-routes + fan-out): never deleted by a clean-up
FIXED_KEYS: set = set()
STITCH_PITCH_MM = 2.5


def sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def seed_ids(tag: str) -> None:
    """rev 3: deterministic KIIDs. pcbnew gives every new track / via / text a random KIID and saves some items in
    KIID order, so two replays of the same sessions used to differ in bytes (rev 2c compared them without uuids).
    The KIID generator is re-seeded from ``tag`` before every step that creates items (fan-out, each session import,
    the clean-up / silkscreen), in the live run and in the replay alike."""
    pcbnew.KIID.SeedGenerator(int(hashlib.sha256(tag.encode("utf-8")).hexdigest()[:8], 16))


def export_dsn(board, dsn: Path, keep_bplane: bool = False, allow_in2: bool = False) -> int:
    """2 layers: DSN without the GND planes (F.Cu always, B.Cu unless ``keep_bplane``): with both planes Freerouting
    treats every GND pad as connected through the pour and never routes GND, which leaves pour islands after the fill.
    4 layers: the outer GND pours are always dropped; In1.Cu (GND) and In2.Cu (+5V) are "power" layers in the DSN
    (Freerouting never routes tracks on them) and their planes stay only when ``keep_bplane`` (plane stage: the GND
    and +5V pads get vias to the planes)."""
    if not pcbnew.ExportSpecctraDSN(board, str(dsn)):
        raise SystemExit("ExportSpecctraDSN failed")
    text = dsn.read_text(encoding="utf-8")
    if board.GetCopperLayerCount() == 4:
        # outer GND pours and the In2.Cu +5V pour are filled after routing, never planes in the DSN; the In1.Cu GND
        # plane stays only in the plane stage (GND pads get vias to it)
        stripped = re.sub(r"\n\s*\(plane GND \(polygon [FB]\.Cu[^)]*\)\)", "", text)
        if IN2_MODE == "mixed" or not keep_bplane:
            stripped = re.sub(r'\n\s*\(plane ("[^"]*"|\S+) \(polygon In2\.Cu[^)]*\)\)', "", stripped)
        if not keep_bplane:
            stripped = re.sub(r'\n\s*\(plane ("[^"]*"|\S+) \(polygon In1\.Cu[^)]*\)\)', "", stripped)
        # never a track on In1.Cu (solid GND). In2.Cu: power tracks + the +5V pour, and signal tracks where F.Cu / B.Cu
        # are full (M: with In2 closed to signals 123 connections stayed open in the 4-layer signals stage; layout
        # deviation from the spec's "In2 = power only", README_ko.md). The GND plane stage keeps In2 closed.
        # Done with a per-class "(use_layer ...)" in every net class circuit: a keepout polygon on In1 / In2 (Freerouting
        # has no wire-only keepout) also blocked every via (M: 0 vias in three 4-layer tries).
        lays = "F.Cu In2.Cu B.Cu" if (allow_in2 and IN2_MODE == "mixed") else "F.Cu B.Cu"
        stripped, n = re.subn(r"\(circuit(\s)", lambda m: f"(circuit{m.group(1)}  (use_layer {lays}){m.group(1)}", stripped)
        if n == 0:
            raise SystemExit("4-layer DSN: no net class circuit to restrict the routing layers")
    else:
        layers = "F" if keep_bplane else "[FB]"
        stripped = re.sub(r"\n\s*\(plane GND \(polygon " + layers + r"\.Cu[^)]*\)\)", "", text)

    def inflate(m: re.Match) -> str:  # via padstacks only: (padstack "Via[0-1]_600:300_um" (shape (circle F.Cu 600)) ...
        block = m.group(0)
        return re.sub(r"\(circle ([\w.]+\.Cu) (\d+)\)",
                      lambda c: f"(circle {c.group(1)} {int(c.group(2)) + int(round(VIA_INFLATE_MM * 1000))})", block)

    stripped = re.sub(r'\(padstack "Via\[0-\d\]_\d+:\d+_um"(?:\s*\(shape \(circle [\w.]+\.Cu \d+\)\))+', inflate, stripped)
    # board-edge copper clearance 0.3 mm (.kicad_pro min_copper_edge_clearance): Freerouting only keeps its 0.13 mm
    # clearance to the boundary (M: 8 copper_edge_clearance errors in the 2-layer try) -> 0.2 mm keepout strips along
    # the edges (+ 0.13 mm clearance = 0.33 mm) and 1.3 mm squares in the rounded corners, on every copper layer
    cu = ["F.Cu", "B.Cu"] if board.GetCopperLayerCount() == 2 else ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
    rects = [(62000, 65000, 138000, 65200), (62000, 134800, 138000, 135000), (62000, 65000, 62200, 135000),
             (137800, 65000, 138000, 135000), (62000, 65000, 63300, 66300), (136700, 65000, 138000, 66300),
             (62000, 133700, 63300, 135000), (136700, 133700, 138000, 135000)]
    kos = ""
    for lay in cu:
        for x0, y0, x1, y1 in rects:
            kos += (f'\n    (keepout "" (polygon {lay} 0  {x0} {-y0}  {x1} {-y0}  {x1} {-y1}  {x0} {-y1}  {x0} {-y0}))')
    k = re.search(r'\n\s*\(via "Via\[0-\d\]_', stripped).start()
    stripped = stripped[:k] + kos + stripped[k:]
    dsn.write_text(stripped, encoding="utf-8", newline="\n")
    return text.count("(plane GND") - stripped.count("(plane GND")


#: nets routed first, one net class per sub-stage (common.py net classes) with ONLY that class's via padstack in
#: the DSN: given all three, Freerouting put 0.6 mm-class vias on servo nets and KiCad gave them the 0.6 mm servo
#: drill -> 0.05 mm annular rings (scratch DRC, M)
POWER_STAGES = [
    ("servo", {"SERVO_5V", "GND_PWR"}, "Via[0-1]_1200:600_um"),
    ("power_high", {"VBAT_LOGIC", "VBAT_LOGIC_RAW", "VBAT_LOGIC_SW"}, "Via[0-1]_800:400_um"),
    ("power", {"+5V", "+3V3", "GPS_VCC", "*_AVCC"}, "Via[0-1]_600:300_um"),
]


def power_stages(layers: int) -> list[tuple[str, set[str], str]]:
    """Via padstack names carry the layer span (Via[0-1] on 2 layers, Via[0-3] on 4). On 4 layers the power stages
    may also use In2.Cu (export_dsn allow_in2); +5V is routed as tracks there and the In2.Cu +5V pour is filled on
    top after routing."""
    out = []
    for tag, nets, via in POWER_STAGES:
        if layers == 4:
            via = via.replace("[0-1]", "[0-3]")
            if IN2_MODE == "plane":
                nets = nets - {"+5V"}  # +5V: vias to the In2.Cu plane in the plane stage
        out.append((tag, nets, via))
    return out


#: 4-layer In2.Cu use: "plane" = the spec stackup (In2 = +5V plane; +5V pads get vias to it like GND pads to In1; no
#: tracks on In2) or "mixed" (In2 = power + signal tracks under a +5V pour). CARRIER_IN2 selects it (experiments).
IN2_MODE = os.environ.get("CARRIER_IN2", "plane")


def stage_nets(board, pattern: set[str]) -> set[str]:
    names = {str(n) for n in board.GetNetsByName().keys()}
    return {n for n in names if n in pattern or ("*_AVCC" in pattern and n.endswith("_AVCC"))}


def power_first_nets(board) -> set[str]:
    out: set[str] = set()
    for _tag, pat, _via in power_stages(board.GetCopperLayerCount()):
        out |= stage_nets(board, pat)
    return out


def only_via(text: str, padstack: str) -> str:
    """Structure via list reduced to one padstack."""
    return re.sub(r'\(via "Via\[0-\d\]_[^)]*\)', f'(via "{padstack}")', text, count=1)


def _block_end(text: str, start: int) -> int:
    depth, i = 0, start
    while True:
        c = text[i]
        if c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return i + 1
        elif c == '"':
            i = text.index('"', i + 1)
        i += 1


def _children(block: str, head: str):
    out, i = [], block.find("(", 1)
    if i < 0:
        return out  # rev 3: an empty block, e.g. the (network_out) of a stage whose nets were all seeded
    while True:
        j = block.find("(", i)
        if j < 0:
            return out
        e = _block_end(block, j)
        m = re.match(r"\(" + head + r' ("[^"]*"|[^\s()]+)', block[j:e])
        out.append((j, e, m.group(1).strip('"') if m else None, block[j:e]))
        i = e


def _class_header(body: str) -> tuple[list[str], int]:
    """Tokens of '(class NAME net net ...' up to the first UNQUOTED '(' (net names such as
    "unconnected-(U7-ADC-Pad2)" contain parentheses), and that position."""
    toks, i, n = [], len("(class "), len(body)
    while i < n:
        c = body[i]
        if c.isspace():
            i += 1
        elif c == '"':
            j = body.index('"', i + 1)
            toks.append(body[i + 1:j])
            i = j + 1
        elif c == "(":
            return toks, i
        else:
            j = i
            while j < n and not body[j].isspace() and body[j] != "(":
                j += 1
            toks.append(body[i:j])
            i = j
    return toks, n


def _net_names_quoted(name: str) -> str:
    return f'"{name}"' if any(c in name for c in ' ()"') else name


def keep_nets(text: str, keep: set[str]) -> str:
    """DSN in which only the ``keep`` nets need routing: every other net is split into single-pin nets
    ("NAME__k"), so its pads keep a net and stay clearance obstacles but nothing joins them. (Removing the nets
    instead makes their pins "netless", and Freerouting then routes over them - 180 KiCad clearance errors, M.)
    Fixed wiring of split nets (the locked pre-routes) becomes structure keep-outs, widened by 2 x 0.13 mm clearance:
    left as wiring of a split net, Freerouting ignored it (a +3V3 via shorted a crystal pre-route, scratch DRC, M)."""
    a = text.index("(network")
    b = _block_end(text, a)
    net = text[a:b]
    split: dict[str, list[str]] = {}
    pieces, p = [], 0
    for s0, e, name, body in _children(net, "net"):
        if body.startswith("(net ") and name not in keep:
            m = re.search(r"\(pins([^)]*)\)", body)
            pins = m.group(1).split() if m else []
            parts = [f"{name}__{k}" for k in range(max(1, len(pins)))]
            split[name] = parts
            new = "\n    ".join(f"(net {_net_names_quoted(n)}\n      (pins {pin})\n    )" for n, pin in zip(parts, pins or [""]))
            pieces.append(net[p:s0] + new)
            p = e
        elif body.startswith("(class "):
            toks, hdr_end = _class_header(body)
            members = []
            for t in toks[1:]:
                members += [_net_names_quoted(x) for x in split.get(t, [t])]
            lines = [" ".join(members[k:k + 8]) for k in range(0, len(members), 8)]
            pieces.append(net[p:s0] + "(class " + toks[0] + " " + "\n        ".join(lines) + "\n      " + body[hdr_end:])
            p = e
    pieces.append(net[p:])
    text = text[:a] + "".join(pieces) + text[b:]
    w = text.find("(wiring")
    if w >= 0:
        we = _block_end(text, w)
        wiring = text[w:we]

        keepouts, pieces, p = [], [], 0
        for s0, e, _n, body in _children(wiring, "wire"):  # every child: (wire ...) and (via ...)
            m = re.search(r'\(net ("[^"]*"|[^)\s]+)\)', body)
            if not m or m.group(1).strip('"') not in split:
                continue
            pm = re.search(r"\(path (\S+) ([\d.]+)\s+([^()]*)\)", body)
            if body.startswith("(wire") and pm:
                wdt = float(pm.group(2)) + 2 * CLEAR_UM
                pts = pm.group(3).split()
                keepouts.append(f'(keepout "" (path {pm.group(1)} {wdt:g} {" ".join(pts)}))')
                # Freerouting ends a path keep-out flat at its end points, the KiCad track has round ends: a GLINK_G2N
                # via sat 0.0069 mm from a 2.4 mm trunk end (attempt a4, kicad-cli DRC, M) -> a circle at every vertex
                for k in range(0, len(pts) - 1, 2):
                    keepouts.append(f'(keepout "" (circle {pm.group(1)} {wdt:g} {pts[k]} {pts[k + 1]}))')
            elif body.startswith("(via"):
                vm = re.match(r'\(via "Via\[0-\d\]_(\d+):\d+_um"\s+(\S+)\s+(\S+)', body)
                dia = float(vm.group(1)) + 2 * CLEAR_UM
                for lay in ("F.Cu", "B.Cu"):
                    keepouts.append(f'(keepout "" (circle {lay} {dia:g} {vm.group(2)} {vm.group(3)}))')
            pieces.append(wiring[p:s0])
            p = e
        pieces.append(wiring[p:])
        text = text[:w] + "".join(pieces) + text[we:]
        if keepouts:
            k = re.search(r'\(via "Via\[0-\d\]_', text).start()
            text = text[:k] + "\n    ".join(keepouts) + "\n    " + text[k:]
    return text


#: DSN units are um; clearance used to widen keep-outs made from fixed wiring
CLEAR_UM = 130


def keep_ses_nets(text: str, keep: set[str]) -> str:
    """Session with only the routes of the ``keep`` nets (drops the fixed wiring echoed back for split nets)."""
    a = text.index("(network_out")
    b = _block_end(text, a)
    net = text[a:b]
    pieces, p = [], 0
    for s0, e, name, body in _children(net, "net"):
        if body.startswith("(net ") and name not in keep:
            pieces.append(net[p:s0])
            p = e
    pieces.append(net[p:])
    return text[:a] + "".join(pieces) + text[b:]


def freerouting(dsn: Path, ses: Path, passes: int, threads: str, tag: str) -> dict:
    if ses.exists():
        ses.unlink()
    cmd = ["java", "-Dfile.encoding=UTF-8", "-jar", str(JAR), "-de", str(dsn), "-do", str(ses), "-mp", str(passes),
           "-mt", threads, "--gui.enabled=false"] + os.environ.get("CARRIER_FR_ARGS", "").split()
    t0 = time.time()
    r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", cwd=str(WORK))
    (WORK / f"freerouting_{tag}_stdout.log").write_text(r.stdout + "\n--- stderr ---\n" + r.stderr, encoding="utf-8", newline="\n")
    if r.returncode != 0 or not ses.exists():
        print(r.stdout[-3000:], r.stderr[-3000:])
        raise SystemExit(f"freerouting {tag} failed (exit {r.returncode})")
    passes_log = re.findall(r"Auto-routing pass #(\d+) .*?\((\d+) unrouted", r.stdout)
    return {"command": " ".join(f'"{c}"' if " " in c else c for c in cmd), "seconds": round(time.time() - t0, 1),
            "exit_code": r.returncode, "dsn_sha256": sha(dsn), "ses_sha256": sha(ses),
            "passes_unrouted": [[int(a), int(b)] for a, b in passes_log], "stdout_tail": r.stdout[-2500:]}


def unconnected(board) -> int:
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.BuildConnectivity()
    try:
        return board.GetConnectivity().GetUnconnectedCount(False)
    except AttributeError:  # SWIG sometimes hands back an untyped shared_ptr; kicad-cli DRC counts it instead
        return None


def fix_neckdowns(board) -> list[dict]:
    """Widen Freerouting neck-downs below the board minimum (0.127 mm on 2 layers, 0.1 mm on 4 layers -
    build_schematic.MIN_TRACK)."""
    fixed = []
    min_w = MIN_W if board.GetCopperLayerCount() == 2 else pcbnew.FromMM(0.1)
    for t in board.GetTracks():
        if t.GetClass() == "PCB_TRACK" and t.GetWidth() < min_w:
            fixed.append({"net": t.GetNetname(), "layer": t.GetLayerName(), "was_mm": round(pcbnew.ToMM(t.GetWidth()), 4),
                          "x": round(pcbnew.ToMM(t.GetStart().x), 3), "y": round(pcbnew.ToMM(t.GetStart().y), 3)})
            t.SetWidth(min_w)
    return fixed


def remove_dangling_vias(board) -> list[dict]:
    """A via whose tracks all sit on one layer (and that is not on a pad) is a stub Freerouting left behind."""
    removed = []
    tracks = [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]
    zone_nets = {z.GetNetCode() for z in board.Zones()}
    for v in [t for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]:
        if v.GetNetCode() in zone_nets:
            continue  # a GND via may be fed by the pour on the other layer (stitch), not a stub
        if _geo_key(v) in FIXED_KEYS:
            continue  # rev 3: a pre-routed via (wide tracks pass through it, they do not end at its centre)
        p = v.GetPosition()
        layers = {t.GetLayer() for t in tracks if t.GetNetCode() == v.GetNetCode() and (t.GetStart() == p or t.GetEnd() == p)}
        on_pad = any(pad.HitTest(p) for fp in board.GetFootprints() for pad in fp.Pads() if pad.GetNetCode() == v.GetNetCode())
        if len(layers) <= 1 and not on_pad:
            removed.append({"net": v.GetNetname(), "x": round(pcbnew.ToMM(p.x), 3), "y": round(pcbnew.ToMM(p.y), 3)})
            board.Delete(v)
    return removed


def shrink_vias(board) -> int:
    """Undo the DSN via inflation: every via goes back to its class size (0.6 / 0.8 / 1.2 mm)."""
    n = 0
    d = pcbnew.FromMM(VIA_INFLATE_MM)
    for v in board.GetTracks():
        if v.GetClass() == "PCB_VIA":
            w = v.GetWidth(pcbnew.F_Cu)
            nominal = round(pcbnew.ToMM(w - d), 3)
            if nominal in (0.6, 0.8, 1.0, 1.2):
                cu = [pcbnew.F_Cu, pcbnew.B_Cu] + ([pcbnew.In1_Cu, pcbnew.In2_Cu] if board.GetCopperLayerCount() == 4 else [])
                for lay in cu:
                    v.SetWidth(lay, w - d)
                n += 1
    return n


def holes(board) -> list[tuple[float, float, float]]:
    out = []
    for v in board.GetTracks():
        if v.GetClass() == "PCB_VIA":
            out.append((pcbnew.ToMM(v.GetPosition().x), pcbnew.ToMM(v.GetPosition().y), pcbnew.ToMM(v.GetDrillValue()) / 2))
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.HasHole():
                ds = pad.GetDrillSize()
                out.append((pcbnew.ToMM(pad.GetPosition().x), pcbnew.ToMM(pad.GetPosition().y), pcbnew.ToMM(max(ds.x, ds.y)) / 2))
    return out


def add_stitching_vias(board) -> list[list[float]]:
    """GND vias on a fixed grid wherever the filled GND copper of both layers holds the whole via (+0.05 mm) and
    the drill keeps >= 0.55 mm to every other hole; outside every rule area (ESP-12E antenna keep-out). Deterministic order."""
    import math

    net = board.FindNet("GND")
    fills = []
    keepouts = []
    for z in board.Zones():
        if z.GetIsRuleArea():
            keepouts.append(z.Outline())
        elif z.GetNetname() == "GND":
            for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
                if z.IsOnLayer(lay):
                    fills.append(z.GetFilledPolysList(lay))
    hl = holes(board)
    # rev 2c (rev-2b review PF-M3: 5 stitching vias sat in SMD pads, 2 on pad edges): every pad on an outer layer is
    # an obstacle (via disk + 0.1 mm), whatever its net - solid-connected SMD pads merge into the fill
    pad_shapes = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
                if pad.IsOnLayer(lay):
                    bb = pad.GetBoundingBox()
                    pad_shapes.append((pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()), pcbnew.ToMM(bb.GetRight()),
                                       pcbnew.ToMM(bb.GetBottom()), pad.GetEffectiveShape(lay)))
    added = []
    r_test = 0.3 + 0.05
    ring = [(0.0, 0.0)] + [(r_test * math.cos(k * math.pi / 4), r_test * math.sin(k * math.pi / 4)) for k in range(8)]
    y = 65.0 + 1.25
    while y < 135.0 - 1.0:
        x = 62.0 + 1.25
        while x < 138.0 - 1.0:
            pts = [pcbnew.VECTOR2I(pcbnew.FromMM(x + dx), pcbnew.FromMM(y + dy)) for dx, dy in ring]
            ok = all(f.Contains(p) for f in fills for p in pts) and len(fills) == 2
            if ok and any(k.Contains(pts[0]) for k in keepouts):
                ok = False
            if ok:
                vc = pcbnew.SHAPE_CIRCLE(pts[0], pcbnew.FromMM(0.3))
                for x0, y0, x1, y1, sh in pad_shapes:
                    if x0 - 0.5 < x < x1 + 0.5 and y0 - 0.5 < y < y1 + 0.5 and sh.Collide(vc, pcbnew.FromMM(0.1)):
                        ok = False
                        break
            if ok:
                for hx, hy, hr in hl:
                    if math.hypot(hx - x, hy - y) - hr - 0.15 < 0.55:
                        ok = False
                        break
            if ok:
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
                v.SetViaType(pcbnew.VIATYPE_THROUGH)
                v.SetWidth(pcbnew.F_Cu, pcbnew.FromMM(0.6))
                v.SetWidth(pcbnew.B_Cu, pcbnew.FromMM(0.6))
                v.SetDrill(pcbnew.FromMM(0.3))
                v.SetNet(net)
                board.Add(v)
                hl.append((x, y, 0.15))
                added.append([round(x, 3), round(y, 3)])
            x = round(x + STITCH_PITCH_MM, 4)
        y = round(y + STITCH_PITCH_MM, 4)
    return added


def finish(board) -> dict:
    seed_ids("finish")
    shrunk = shrink_vias(board)
    fixed = fix_neckdowns(board)
    widened = widen_power(board)
    items = list(board.GetTracks())
    n_vias = sum(1 for t in items if t.GetClass() == "PCB_VIA")
    n_segs = len(items) - n_vias
    dangling = remove_dangling_vias(board)
    board.BuildConnectivity()
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    leftovers = close_leftovers_step(board)
    stitch = add_stitching_vias(board)
    board.BuildConnectivity()
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    import close_leftovers

    # GND / +5V pad islands (e.g. an F.Cu pour island holding the NT1 ground tie): joined after the stitching vias,
    # which are the nearest main-cluster targets
    leftovers += close_leftovers.close_islands(board)
    import silk

    silk_log = silk.add_silk(board)
    board.Save(str(BOARD))
    narrowed = fix_small_clearance(board)
    if narrowed:
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        board.Save(str(BOARD))
    stubs = remove_dangling_tracks(board)
    if stubs:
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        board.Save(str(BOARD))
    # unconnected items are counted by kicad-cli DRC (checks_exports.py), not here
    return {"vias_shrunk_after_inflation": shrunk, "neckdowns_widened_to_0.127mm": fixed, "dangling_vias_removed": dangling,
            "segments": n_segs, "routed_vias": n_vias - len(dangling), "gnd_stitching_vias": len(stitch),
            "tracks_narrowed_for_clearance": narrowed, "power_tracks_widened": widened, "leftovers_routed_by_script": leftovers,
            "dangling_track_stubs_removed": stubs, "silk": silk_log, "routed_board_sha256": sha(BOARD)}


#: class width per power / servo net (common.net_class_of; .kicad_pro classes)
POWER_CLASS_W = {"SERVO_5V": 1.5, "GND_PWR": 1.5, "VBAT_LOGIC_RAW": 1.0, "VBAT_LOGIC": 1.0, "VBAT_LOGIC_SW": 1.0,
                 "+5V": 0.5, "+3V3": 0.5, "GPS_VCC": 0.5}
EDGE_CLR_MM = 0.3
CLASS_CLR_MM = 0.13


def _bbmm(item) -> tuple[float, float, float, float]:
    bb = item.GetBoundingBox()
    return (pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()), pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom()))


def widen_power(board) -> dict:
    """rev 2c (rev-2b review PF-M6): Freerouting necks power / servo tracks down near pads (SERVO_5V 0.9-1.125 mm,
    VBAT_LOGIC_SW 0.75 mm against 1.5 / 1.0 mm). Every such segment is widened to the largest width in
    {class width, class - 0.05, ...} whose copper keeps the class clearance (0.13 mm + 0.005 mm margin) to every pad,
    track, via and footprint copper drawing of another net on that layer, 0.3 mm to the board edge and stays out of
    the rule areas. Nothing is moved; segments that cannot grow stay as they are (listed, and measured again by
    board_metrics.py). Deterministic order."""
    import math as _m

    tracks = list(board.GetTracks())
    items = []
    for t in tracks:
        for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
            if t.IsOnLayer(lay):
                items.append((lay, t.GetNetCode(), _bbmm(t), t))
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
                if pad.IsOnLayer(lay):
                    items.append((lay, pad.GetNetCode(), _bbmm(pad), pad))
        for g in fp.GraphicalItems():
            for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
                if g.IsOnLayer(lay):
                    items.append((lay, -1, _bbmm(g), g))
    rule_areas = [z.Outline() for z in board.Zones() if z.GetIsRuleArea()]
    out = {"widened": [], "kept": []}
    cand = sorted((t for t in tracks if t.GetClass() == "PCB_TRACK" and t.GetNetname() in POWER_CLASS_W
                   and pcbnew.ToMM(t.GetWidth()) < POWER_CLASS_W[t.GetNetname()] - 1e-6),
                  key=lambda t: (t.GetNetname(), t.GetLayer(), t.GetStart().x, t.GetStart().y, t.GetEnd().x, t.GetEnd().y))
    for t in cand:
        net, lay, w0 = t.GetNetname(), t.GetLayer(), pcbnew.ToMM(t.GetWidth())
        a, b = t.GetStart(), t.GetEnd()
        ax, ay, bx, by = (pcbnew.ToMM(v) for v in (a.x, a.y, b.x, b.y))
        best = None
        w = POWER_CLASS_W[net]
        while w > w0 + 1e-6:
            half = w / 2
            if (min(ax, bx) - half < 62.0 + EDGE_CLR_MM or max(ax, bx) + half > 138.0 - EDGE_CLR_MM
                    or min(ay, by) - half < 65.0 + EDGE_CLR_MM or max(ay, by) + half > 135.0 - EDGE_CLR_MM):
                w = round(w - 0.05, 3)
                continue
            seg = pcbnew.SHAPE_SEGMENT(a, b, pcbnew.FromMM(w))
            ok = True
            r = half + CLASS_CLR_MM + 0.01
            for il, nc, (x0, y0, x1, y1), it in items:
                if il != lay or nc == t.GetNetCode() or it is t:
                    continue
                if x1 < min(ax, bx) - r or x0 > max(ax, bx) + r or y1 < min(ay, by) - r or y0 > max(ay, by) + r:
                    continue
                if it.GetEffectiveShape(lay).Collide(seg, pcbnew.FromMM(CLASS_CLR_MM + 0.005)):
                    ok = False
                    break
            if ok and rule_areas:
                for k in range(0, 11):
                    pt = pcbnew.VECTOR2I(int(a.x + (b.x - a.x) * k / 10), int(a.y + (b.y - a.y) * k / 10))
                    if any(ra.Collide(pt, pcbnew.FromMM(half + 0.05)) for ra in rule_areas):
                        ok = False
                        break
            if ok:
                best = w
                break
            w = round(w - 0.05, 3)
        if best is not None:
            t.SetWidth(pcbnew.FromMM(best))
            out["widened"].append({"net": net, "layer": t.GetLayerName(), "from_mm": round(w0, 3), "to_mm": best,
                                   "length_mm": round(_m.hypot(bx - ax, by - ay), 2)})
        else:
            out["kept"].append({"net": net, "layer": t.GetLayerName(), "width_mm": round(w0, 3),
                                "length_mm": round(_m.hypot(bx - ax, by - ay), 2), "at": [round(ax, 2), round(ay, 2)]})
    return out


def close_leftovers_step(board) -> list[dict]:
    """rev 2c: route the connections Freerouting left open (kicad-cli DRC unconnected items) with the scripted A*
    router (close_leftovers.py: signal nets only, new locked copper only, clearance-checked)."""
    import close_leftovers

    cli = KICAD_CLI
    rep = WORK / "drc_leftovers.json"
    board.Save(str(BOARD))
    subprocess.run([str(cli), "pcb", "drc", "--refill-zones", "--format", "json", "--units", "mm", "-o", str(rep), str(BOARD)],
                   capture_output=True, text=True, encoding="utf-8", errors="replace")
    res = close_leftovers.close(board, json.loads(rep.read_text(encoding="utf-8")))
    if any(r.get("routed") for r in res):
        board.BuildConnectivity()
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    return res


def remove_dangling_tracks(board) -> list[dict]:
    """rev 2c (rev-2b review PF-m5): delete the track stubs kicad-cli DRC reports as 'track_dangling' (Freerouting
    leftovers that end nowhere), up to 3 rounds (a removed stub can expose the next one). Only tracks flagged by DRC
    are deleted, never a via."""
    cli = KICAD_CLI
    rep = WORK / "drc_dangling.json"
    removed = []
    for _round in range(3):
        board.Save(str(BOARD))
        subprocess.run([str(cli), "pcb", "drc", "--severity-all", "--format", "json", "--units", "mm", "-o", str(rep), str(BOARD)],
                       capture_output=True, text=True, encoding="utf-8", errors="replace")
        d = json.loads(rep.read_text(encoding="utf-8"))
        by_uuid = {t.m_Uuid.AsString(): t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"}
        hit = []
        for v in d.get("violations", []):
            if v.get("type") != "track_dangling":
                continue
            for it in v.get("items", []):
                t = by_uuid.get(it.get("uuid"))
                if t is not None and t not in hit and _geo_key(t) not in FIXED_KEYS:
                    hit.append(t)  # rev 3: never a pre-route / fan-out stub
        if not hit:
            break
        for t in hit:
            removed.append({"net": t.GetNetname(), "layer": t.GetLayerName(), "x": round(pcbnew.ToMM(t.GetStart().x), 3),
                            "y": round(pcbnew.ToMM(t.GetStart().y), 3), "length_mm": round(pcbnew.ToMM(t.GetLength()), 3)})
            board.Delete(t)
    return removed


def fix_small_clearance(board) -> list[dict]:
    """Freerouting leaves a few angled track pieces some um closer to a pad than the class clearance (M: 2 x 0.1213 mm
    against 0.13 mm in the 4-layer run). kicad-cli DRC finds them; a track short by < 0.03 mm is narrowed by twice the
    shortfall + 0.01 mm (never below the board minimum track). Nothing else is moved."""
    cli = KICAD_CLI
    rep = WORK / "drc_clearance_fix.json"
    subprocess.run([str(cli), "pcb", "drc", "--format", "json", "--units", "mm", "-o", str(rep), str(BOARD)],
                   capture_output=True, text=True, encoding="utf-8", errors="replace")
    d = json.loads(rep.read_text(encoding="utf-8"))
    by_uuid = {t.m_Uuid.AsString(): t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"}
    out = []
    for v in d.get("violations", []):
        if v.get("type") != "clearance":
            continue
        nums = re.findall(r"([\d.]+) mm", v.get("description", ""))
        if len(nums) < 2:
            continue
        need, got = float(nums[0]), float(nums[1])
        deficit = need - got
        if not 0 < deficit < 0.03:
            continue
        for it in v.get("items", []):
            t = by_uuid.get(it.get("uuid"))
            if t is None:
                continue
            w = pcbnew.ToMM(t.GetWidth())
            nw = round(w - 2 * deficit - 0.01, 3)
            if nw >= (0.127 if board.GetCopperLayerCount() == 2 else 0.1) and nw < w:
                t.SetWidth(pcbnew.FromMM(nw))
                out.append({"net": t.GetNetname(), "from_mm": round(w, 3), "to_mm": nw, "shortfall_mm": round(deficit, 4)})
    return out


def lock_all(board) -> int:
    n = 0
    for t in board.GetTracks():
        if not t.IsLocked():
            t.SetLocked(True)
            n += 1
    return n


def run_stage(board, tag: str, nets: set[str], done: set[str], via: str | None, passes: int, threads: str,
              keep_bplane: bool) -> dict:
    dsn, ses = WORK / f"vtol_carrier_{tag}.dsn", WORK / f"vtol_carrier_{tag}.ses"
    export_dsn(board, dsn, keep_bplane, allow_in2=tag != "gnd")
    text = keep_nets(dsn.read_text(encoding="utf-8"), nets | done)
    if via:
        text = only_via(text, via)
    dsn.write_text(text, encoding="utf-8", newline="\n")
    st = freerouting(dsn, ses, passes, threads, tag)
    st.update({"nets_routed_here": len(nets), "via": via or "all", "b_cu_gnd_plane_in_dsn": keep_bplane})
    ses.write_text(keep_ses_nets(ses.read_text(encoding="utf-8"), (nets | done) - set(PREROUTED_NETS)),
                   encoding="utf-8", newline="\n")
    seed_ids(f"import:{tag}")
    if not pcbnew.ImportSpecctraSES(board, str(ses)):
        raise SystemExit(f"ImportSpecctraSES ({tag}) failed")
    # imported vias carry the inflated DSN size; back to nominal now, or the next stage exports them inflated and
    # inflates them again (268 fixed-via clearance violations in the first 4-layer try, M)
    st["vias_shrunk"] = shrink_vias(board)
    st["items_locked"] = lock_all(board)
    st["unconnected_after_import_and_fill"] = unconnected(board)
    st["ses"] = ses.name
    return st


def _geo_key(t) -> tuple:
    if t.GetClass() == "PCB_VIA":
        return ("via", t.GetNetname(), t.GetPosition().x // 1000, t.GetPosition().y // 1000)
    a, b = sorted([(t.GetStart().x // 1000, t.GetStart().y // 1000), (t.GetEnd().x // 1000, t.GetEnd().y // 1000)])
    return ("seg", t.GetNetname(), t.GetLayer(), a, b)


def fixed_copper(board) -> set:
    """Geometry keys (net, layer, end points / via position, 1 um grid) of the copper locked before routing."""
    return {_geo_key(t) for t in board.GetTracks() if t.IsLocked()}


def polish(board, fixed: set, passes: int, threads: str) -> dict:
    """Unlock every routed track / via that is not fixed copper, run Freerouting on all nets once more with the
    existing wiring as rip-up-able routes (In1 / In2 planes kept, all via padstacks), import, shrink, lock."""
    n_unlocked = 0
    for t in board.GetTracks():
        if _geo_key(t) not in fixed and t.IsLocked():
            t.SetLocked(False)
            n_unlocked += 1
    before = unconnected(board)
    done = {str(n) for n in board.GetNetsByName().keys() if str(n) and not str(n).startswith("unconnected-")}
    done -= set(PREROUTED_NETS)
    st = run_stage(board, "polish", set(), done, None, passes, threads, True)
    st.update({"unconnected_before": before, "items_unlocked": n_unlocked})
    return st


def check_prerouted(src: Path) -> dict:
    """rev 3: the PREROUTED_NETS must be complete on the unrouted board (kicad-cli DRC: no unconnected item of
    theirs). Anything else means build_pcb.power_preroutes and the placement no longer match -> stop, no guess."""
    rep = WORK / "drc_unrouted.json"
    subprocess.run([str(KICAD_CLI), "pcb", "drc", "--format", "json", "--units", "mm", "-o", str(rep), str(src)],
                   capture_output=True, text=True, encoding="utf-8", errors="replace")
    d = json.loads(rep.read_text(encoding="utf-8"))
    bad = []
    for u in d.get("unconnected_items", []):
        text = " ; ".join(i.get("description", "") for i in u.get("items", []))
        if any(n in PREROUTED_NETS for n in re.findall(r"\[([^\]]+)\]", text)):
            bad.append(text)
    if bad:
        raise SystemExit("pre-routed nets are not complete on the unrouted board:\n  " + "\n  ".join(bad))
    return {"nets": list(PREROUTED_NETS), "open_connections": 0, "tool": "kicad-cli pcb drc (unconnected items)"}


def _pad_sets(board) -> dict[str, frozenset]:
    out: dict[str, set] = {}
    for fp in board.GetFootprints():
        for p in fp.Pads():
            n = p.GetNetname()
            if n and not n.startswith("unconnected-"):
                sz = p.GetBoundingBox()
                out.setdefault(n, set()).add((fp.GetReference(), p.GetNumber(), p.GetPosition().x // 1000,
                                              p.GetPosition().y // 1000, sz.GetWidth() // 1000, sz.GetHeight() // 1000))
    return {k: frozenset(v) for k, v in out.items()}


def _seed_session(text: str, keep: set[str]) -> str:
    """A recorded session reduced to the routes of ``keep`` and WITHOUT component placement (ImportSpecctraSES would
    move every footprint to the position recorded in the session - the rev 2c one)."""
    a = text.index("(placement")
    b = _block_end(text, a)
    text = text[:a] + "(placement\n    (resolution um 10)\n  )" + text[b:]
    return keep_ses_nets(text, keep)


#: DRC findings that are no conflict of a seeded route with the rev 3 board
SEED_IGNORE = {"track_dangling", "via_dangling", "unconnected_items"}


def eco_seed(board, src: Path) -> tuple[dict, list[list[str]]]:
    """rev 3 ECO seed. 274 of the 288 rev 3 nets have exactly the pads they had on rev 2c (M, same references, pad
    numbers, positions and pad boxes on the two unrouted boards), and the rev 2c board routed completely. Those nets
    are imported from the recorded rev 2c stage sessions; kicad-cli DRC (project rules, zones refilled) on a scratch
    copy then removes every seeded net that conflicts with anything on the rev 3 board (new parts, pre-routes,
    fan-out) - whole nets, repeated until no conflict is left - and only the survivors are imported on the real
    board. Removed and changed nets are routed by the normal stages. Clearance shortfalls < 0.03 mm are left to
    fix_small_clearance as on rev 2c. No rule is touched: the seed is ordinary locked copper for all later steps and
    the final board is judged by kicad-cli DRC like any other. The seed sessions written here (kicad/work/
    seed_<stage>.ses) are replay stages, so ``--reuse-ses`` reproduces them."""
    rp = json.loads((SEED_DIR / "ses_replay.json").read_text(encoding="utf-8"))
    old = pcbnew.LoadBoard(str(SEED_DIR / "vtol_carrier_unrouted.kicad_pcb"))
    a, b = _pad_sets(old), _pad_sets(board)
    seed = {n for n in b if a.get(n) == b[n]} - set(PREROUTED_NETS)
    log: dict = {"source": str(SEED_DIR.relative_to(ROOT)).replace("\\", "/") if ROOT in SEED_DIR.parents else str(SEED_DIR),
                 "source_stages": rp["stages"], "nets_on_board": len(b), "nets_with_identical_pads": len(seed),
                 "changed_or_new_nets": sorted(set(b) - seed - set(PREROUTED_NETS)), "rounds": [], "removed": {}}
    texts = {tag: (SEED_DIR / name).read_text(encoding="utf-8") for tag, name in rp["stages"]}
    base = WORK / "seed_base.kicad_pcb"
    board.Save(str(base))
    trial_path = WORK / "seed_try.kicad_pcb"
    shutil.copyfile(KICAD / "vtol_carrier.kicad_pro", WORK / "seed_try.kicad_pro")
    trial = pcbnew.LoadBoard(str(base))
    fixed = fixed_copper(trial)
    for tag, _name in rp["stages"]:
        p = WORK / f"seed_{tag}.ses"
        p.write_text(_seed_session(texts[tag], seed), encoding="utf-8", newline="\n")
        if not pcbnew.ImportSpecctraSES(trial, str(p)):
            raise SystemExit(f"ImportSpecctraSES seed_{tag} failed")
        shrink_vias(trial)
        lock_all(trial)
    rep = WORK / "drc_seed.json"
    for rnd in range(12):
        trial.Save(str(trial_path))
        subprocess.run([str(KICAD_CLI), "pcb", "drc", "--severity-all", "--refill-zones", "--format", "json", "--units",
                        "mm", "-o", str(rep), str(trial_path)], capture_output=True, text=True, encoding="utf-8",
                       errors="replace")
        d = json.loads(rep.read_text(encoding="utf-8"))
        seeded = {t.m_Uuid.AsString(): t for t in trial.GetTracks() if _geo_key(t) not in fixed}
        drop: dict[str, str] = {}
        small = 0
        for v in d.get("violations", []):
            if v.get("type") in SEED_IGNORE:
                continue
            hit = [seeded[i.get("uuid")] for i in v.get("items", []) if i.get("uuid") in seeded]
            if not hit:
                continue
            if v.get("type") == "clearance":
                nums = re.findall(r"([\d.]+) mm", v.get("description", ""))
                if len(nums) >= 2 and 0 < float(nums[0]) - float(nums[1]) < 0.03:
                    small += 1
                    continue
            for t in hit:
                drop.setdefault(t.GetNetname(), v.get("type"))
        log["rounds"].append({"round": rnd, "violations": len(d.get("violations", [])), "small_clearance_left_to_fix": small,
                              "nets_removed": sorted(drop)})
        if not drop:
            break
        for t in [t for t in trial.GetTracks() if t.GetNetname() in drop and _geo_key(t) not in fixed]:
            trial.Delete(t)
        seed -= set(drop)
        log["removed"].update(drop)
    else:
        raise SystemExit("eco_seed: conflicts did not converge in 12 rounds")
    del trial, old
    for f in list(WORK.glob("seed_base.*")) + list(WORK.glob("seed_try.*")) + list(WORK.glob("~seed_*.lck")):
        f.unlink()  # scratch boards of the conflict search (the report drc_seed.json stays as evidence)
    stages = []
    for tag, _name in rp["stages"]:
        p = WORK / f"seed_{tag}.ses"
        p.write_text(_seed_session(texts[tag], seed), encoding="utf-8", newline="\n")
        seed_ids(f"import:seed_{tag}")
        if not pcbnew.ImportSpecctraSES(board, str(p)):
            raise SystemExit(f"ImportSpecctraSES seed_{tag} failed")
        shrink_vias(board)
        lock_all(board)
        stages.append([f"seed_{tag}", p.name])
    log.update({"nets_seeded": len(seed), "nets_removed_for_conflicts": len(log["removed"]),
                "unconnected_after_seed_and_fill": unconnected(board),
                "ses_sha256": {name: sha(WORK / name) for _t, name in stages}})
    return log, stages


def main() -> int:
    reuse = "--reuse-ses" in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    passes = int(args[0]) if len(args) > 0 else 30
    threads = args[1] if len(args) > 1 else "4"
    src = WORK / "vtol_carrier_unrouted.kicad_pcb"
    shutil.copyfile(src, BOARD)
    board = pcbnew.LoadBoard(str(BOARD))
    seed_ids("start")
    final_ses = WORK / "vtol_carrier.ses"
    replay_path = WORK / "ses_replay.json"
    if reuse:
        # replay: the session of each stage holds only that stage's new routes (M: the last 2-layer session had 47
        # wires of ~1000), so the recorded stages are imported again in their order, with the same fan-out, via shrink
        # and locking as in the live run -> deterministic copy of the routed board
        if not replay_path.exists():
            raise SystemExit("--reuse-ses: no kicad/work/ses_replay.json")
        rp = json.loads(replay_path.read_text(encoding="utf-8"))
        if rp.get("unrouted_board_sha256") and rp["unrouted_board_sha256"] != sha(src):
            # rev 3: sessions recorded on another placement / pre-route set must never be laid over this board
            raise SystemExit(f"--reuse-ses: the sessions were recorded on unrouted board {rp['unrouted_board_sha256']}, "
                             f"this one is {sha(src)} - a fresh live route is needed")
        res = {"mode": "replay of the recorded Freerouting sessions", "replay": rp, "ses_sha256": {}}
        if rp.get("fanout"):
            import fanout

            res["fanout"] = fanout.run(board)
        fixed = fixed_copper(board)  # pre-routes + fan-out (locked before any Freerouting stage)
        FIXED_KEYS.update(fixed)
        for tag, name in rp["stages"]:
            p = WORK / name
            res["ses_sha256"][name] = sha(p)
            if tag.startswith("polish"):  # a polish session replaces every route that is not fixed copper
                for t in board.GetTracks():
                    if _geo_key(t) not in fixed and t.IsLocked():
                        t.SetLocked(False)
            seed_ids(f"import:{tag}")
            if not pcbnew.ImportSpecctraSES(board, str(p)):
                raise SystemExit(f"ImportSpecctraSES {name} failed")
            shrink_vias(board)
            lock_all(board)
        if "--polish" in sys.argv and not any(t == "polish" for t, _n in rp["stages"]):
            res["polish"] = polish(board, fixed, passes, threads)
            rp["stages"].append(["polish", res["polish"]["ses"]])
            rp["polish_source"] = (f"route.py --reuse-ses --polish {passes} {threads} on top of the recorded stages "
                                   "(rip-up of everything but the pre-routes / fan-out)")
            replay_path.write_text(json.dumps(rp, indent=1) + "\n", encoding="utf-8", newline="\n")
            res["ses_sha256"][res["polish"]["ses"]] = sha(WORK / res["polish"]["ses"])
        res.update(finish(board))
        (WORK / "route_reuse_log.json").write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8", newline="\n")
        print(json.dumps({k: v for k, v in res.items() if k not in ("silk", "fanout")}, indent=1))
        return 0
    log: dict = {"freerouting_jar_sha256": sha(JAR), "unrouted_board_sha256": sha(src), "stages": {}}
    all_nets = {str(n) for n in board.GetNetsByName().keys() if str(n) and not str(n).startswith("unconnected-")}
    done: set[str] = set()
    last = None
    layers = board.GetCopperLayerCount()
    log["copper_layers"] = layers
    if layers == 4 and IN2_MODE == "plane":
        import fanout

        log["fanout"] = fanout.run(board)  # GND / +5V pads -> vias to In1 / In2 first, locked (fanout.py)
    FIXED_KEYS.update(fixed_copper(board))
    log["prerouted"] = check_prerouted(src)
    all_nets -= set(PREROUTED_NETS)
    seed_stages: list[list[str]] = []
    if os.environ.get("CARRIER_SEED", "1") != "0":
        log["eco_seed"], seed_stages = eco_seed(board, src)
    # ORDER: "power_first" (rev-1 / 2-layer practice: servo, power_high, power, signals, GND) or "rest_together"
    # (4-layer: servo + power_high first, then every other net incl. the GND fan-out to the In1.Cu plane in ONE
    # Freerouting run - M: power_first on 4 layers left 31 power + 148 signal connections open, the 0.5 mm +5V / +3V3
    # tracks routed alone first block the signals)
    order = os.environ.get("CARRIER_ROUTE_ORDER", "power_first")  # M: rest_together left 75-128 open on 4 layers
    log["route_order"] = order
    plane_via = None if layers == 2 else "Via[0-3]_600:300_um"
    stages = power_stages(layers) if order == "power_first" else power_stages(layers)[:2]
    for tag, pat, via in stages:
        nets = stage_nets(board, pat) - set(PREROUTED_NETS)
        if not nets:
            continue  # rev 3: power_high holds only pre-routed nets
        log["stages"][tag] = run_stage(board, tag, nets, done, via, 20, threads, False)
        done |= nets
        last = tag
    plane_nets = {"GND", "+5V"} if (layers == 4 and IN2_MODE == "plane") else {"GND"}
    log["in2_mode"] = IN2_MODE if layers == 4 else None
    if order == "power_first":
        signals = all_nets - done - plane_nets
        log["stages"]["signals"] = run_stage(board, "signals", signals, done, None, passes, threads, False)
        done |= signals
        log["stages"]["gnd"] = run_stage(board, "gnd", plane_nets, done, plane_via, 20, threads, True)
        done |= plane_nets
        last = "gnd"
    else:
        rest = all_nets - done
        log["stages"]["rest"] = run_stage(board, "rest", rest, done, plane_via, passes, threads, True)
        done |= rest
        last = "rest"
    if log["stages"][last]["unconnected_after_import_and_fill"]:
        log["stages"]["complete"] = run_stage(board, "complete", set(), done, None, 20, threads, True)
        last = "complete"
    shutil.copyfile(WORK / f"vtol_carrier_{last}.ses", final_ses)
    log["final_session"] = f"vtol_carrier_{last}.ses"
    replay = {"fanout": "fanout" in log, "stages": seed_stages + [[t, log["stages"][t]["ses"]] for t in log["stages"]],
              "unrouted_board_sha256": log["unrouted_board_sha256"]}
    replay_path.write_text(json.dumps(replay, indent=1) + "\n", encoding="utf-8", newline="\n")
    log["replay"] = replay
    log.update(finish(board))
    (WORK / "route_log.json").write_text(json.dumps(log, indent=1) + "\n", encoding="utf-8", newline="\n")
    brief = {k: v for k, v in log.items() if k not in ("stages", "silk")}
    brief["stages"] = {t: {k: v for k, v in st.items() if k not in ("stdout_tail", "command")} for t, st in log["stages"].items()}
    print(json.dumps(brief, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
