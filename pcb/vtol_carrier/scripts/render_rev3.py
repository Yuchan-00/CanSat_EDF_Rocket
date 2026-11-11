"""Carrier rev 3: the sensor-mounting renders (plan section 11: views a1-a3, b1-b4, c1-c5, d1).

Invariant this module enforces: the routed board is never modified.  Three derived copies are written next to it and
only those are rendered:

    kicad/render_as_delivered.kicad_pcb     what JLCPCB ships: the 3D models of every user-fitted part are hidden
                                            (U1..U4, the GY-521 strip in J1; no module, no GPS plug)
    kicad/render_completed.kicad_pcb        after the user's own work: U1..U4 shown, GY-521 on J1, BMP280 in J2,
                                            OpenLog in J5, JST-GH plug with the start of the GPS lead in J7
    kicad/render_completed_limit.kicad_pcb  the completed board plus a red frame at the 13.4 mm height limit (view b4)

The module models come from ``make_module_models.py`` (kicad/3d/*.step + models_manifest.json).  Each one is written
in the 3D frame of its connector footprint, so it is attached with offset 0 / rotation 0; this script refuses a
footprint whose library id differs from the one the model was written for.  A library 3D model whose file is not on
disk is reported, and J14 (Molex 22035025: footprint in the library, model file missing) gets the project model.

The pictures are illustrations.  Module dimensions are tagged F / A in spec/module_dimensions.json; whether the
OpenLog and the BMP280 fit is decided by the fit check, not by these renders.

Run with KiCad's python (pcbnew); under any other interpreter the script re-runs itself with it:
    "C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/bin/python.exe" -B -X utf8 scripts/render_rev3.py
Options: --board FILE (default kicad/vtol_carrier.kicad_pcb), --kicad-dir DIR (where the derived copies go),
         --out DIR (default out/render), --views a1,c2,... (default all), --quality high|basic, --prepare-only,
         --slides-only (compose the Korean slides e1-e4 from the rendered views and from d2 of render_d2_scene.py)

Order of a full run: render_rev3.py (views a1..d1) -> render_d2_scene.py (d2) -> render_rev3.py --slides-only.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODELS_DIR = ROOT / "kicad" / "3d"
WIDTH, HEIGHT = 1920, 1080

# ---------------------------------------------------------------------------------------------------------------
# VIEW TABLE - every picture and its camera in one place.
#   board   : D = render_as_delivered, C = render_completed, L = render_completed_limit
#   side    : kicad-cli --side
#   rotate  : kicad-cli --rotate "X,Y,Z" in degrees (None = straight view of that side)
#   zoom    : kicad-cli --zoom
#   persp   : perspective projection (False = orthographic, used where sizes must compare)
#   focus   : None = board centre; ("model", name) = centre of that attached model; ("fp", ref) = footprint origin;
#             optional 3rd / 4th entries shift the focus by (dx, dy) mm in board coordinates.
#             The focus becomes kicad-cli --pivot (cm from the board centre, y up).
#   floor   : kicad-cli --floor (floor, shadows, post-processing)
# ---------------------------------------------------------------------------------------------------------------
VIEWS: tuple[dict, ...] = (
    {"id": "a1", "file": "a1_as_delivered_top", "board": "D", "side": "top", "rotate": None, "zoom": 1.0,
     "persp": False, "focus": None, "floor": False, "what": "as delivered by JLCPCB, top"},
    {"id": "a2", "file": "a2_as_delivered_front", "board": "D", "side": "top", "rotate": (-45, 0, 30), "zoom": 0.9,
     "persp": True, "focus": None, "floor": True, "what": "as delivered by JLCPCB, 3/4 front"},
    {"id": "a3", "file": "a3_as_delivered_bottom", "board": "D", "side": "bottom", "rotate": None, "zoom": 1.0,
     "persp": False, "focus": None, "floor": False, "what": "as delivered by JLCPCB, bottom"},
    {"id": "b1", "file": "b1_completed_top", "board": "C", "side": "top", "rotate": None, "zoom": 1.0,
     "persp": False, "focus": None, "floor": False, "what": "completed, top"},
    {"id": "b2", "file": "b2_completed_front", "board": "C", "side": "top", "rotate": (-45, 0, 30), "zoom": 0.9,
     "persp": True, "focus": None, "floor": True, "what": "completed, 3/4 front"},
    {"id": "b3", "file": "b3_completed_back", "board": "C", "side": "top", "rotate": (-50, 0, -150), "zoom": 0.9,
     "persp": True, "focus": None, "floor": True, "what": "completed, 3/4 back"},
    {"id": "b4", "file": "b4_completed_side_limit", "board": "L", "side": "front", "rotate": None, "zoom": 1.0,
     "persp": False, "focus": None, "floor": False, "what": "completed, side elevation with the 13.4 mm limit frame"},
    {"id": "c1", "file": "c1_gy521_on_J1", "board": "C", "side": "top", "rotate": (-55, 0, 125), "zoom": 2.4,
     "persp": True, "focus": ("model", "gy521_on_J1"), "floor": True, "what": "GY-521 soldered on J1"},
    {"id": "c2", "file": "c2_bmp280_in_J2", "board": "C", "side": "top", "rotate": (-65, 0, 70), "zoom": 2.4,
     "persp": True, "focus": ("model", "bmp280_in_J2"), "floor": True, "what": "BMP280 plugged into J2"},
    {"id": "c3", "file": "c3_openlog_in_J5", "board": "C", "side": "top", "rotate": (-65, 0, -70), "zoom": 2.4,
     "persp": True, "focus": ("model", "openlog_in_J5"), "floor": True, "what": "OpenLog plugged into J5, microSD side"},
    {"id": "c4", "file": "c4_esp12f_keepout", "board": "C", "side": "top", "rotate": (-50, 0, 25), "zoom": 2.4,
     "persp": True, "focus": ("fp", "U7"), "floor": True, "what": "ESP-12F with its antenna keep-out"},
    {"id": "c5", "file": "c5_mcu_pin1", "board": "C", "side": "top", "rotate": (-40, 0, 20), "zoom": 2.8,
     "persp": True, "focus": ("fp", "U3"), "floor": True, "what": "one ATmega128A with the pin-1 mark"},
    {"id": "d1", "file": "d1_gps_plug_J7", "board": "C", "side": "top", "rotate": (-60, 0, 35), "zoom": 2.6,
     "persp": True, "focus": ("model", "gh_plug_in_J7"), "floor": True, "what": "JST-GH plug and the GPS lead at J7"},
)

# Parts the user fits: their library models are hidden in the as-delivered copy.  J1 keeps the library pin-header
# footprint (8 plated holes); its header model is hidden in every copy because the GY-521 model brings its own strip.
HIDE_AS_DELIVERED = ("U1", "U2", "U3", "U4", "J1")
HIDE_ALWAYS = ("J1",)
ATTACH_COMPLETED = ("gy521_on_J1", "bmp280_in_J2", "openlog_in_J5", "gh_plug_in_J7")
PROJECT_MODEL_IF_LIBRARY_MISSING = {"J14": "molex_22035025_J14"}
LIMIT_MODEL, LIMIT_ANCHOR = "height_limit_frame", "J1"
OUTLINE_TOL = 0.1          # mm, board edge bounding box against the plan outline


# ------------------------------------------------------------------------------------------------- tools
def find_kicad_cli() -> Path:
    """kicad-cli from PATH, else the per-user Windows install (discovered, never assumed)."""
    hit = shutil.which("kicad-cli")
    if hit:
        return Path(hit)
    cand = Path(r"C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/bin/kicad-cli.exe")
    if cand.is_file():
        return cand
    raise FileNotFoundError("kicad-cli not found (put it on PATH)")


def run(cmd: list[str], timeout: int = 900) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=timeout)


def ensure_pcbnew() -> None:
    """pcbnew only exists in KiCad's own python: re-run this script with it when needed."""
    try:
        import pcbnew  # noqa: F401
        return
    except ImportError:
        pass
    cli = find_kicad_cli()
    py = cli.parent / ("python.exe" if os.name == "nt" else "python3")
    if not py.is_file() or os.environ.get("RENDER_REV3_REEXEC") == "1":
        raise SystemExit("pcbnew is not importable; run this script with KiCad's python")
    env = dict(os.environ, RENDER_REV3_REEXEC="1", PYTHONDONTWRITEBYTECODE="1")
    raise SystemExit(subprocess.run([str(py), "-B", "-X", "utf8", str(Path(__file__).resolve()), *sys.argv[1:]],
                                    env=env).returncode)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


# ------------------------------------------------------------------------------------------------- board copies
def model_file_on_disk(filename: str, board_dir: Path, share: Path) -> bool:
    """True when a footprint's 3D model reference resolves to a file (any of the usual extensions)."""
    p = filename.replace("\\", "/")
    for var in ("KICAD10_3DMODEL_DIR", "KICAD9_3DMODEL_DIR", "KICAD8_3DMODEL_DIR", "KICAD7_3DMODEL_DIR",
                "KICAD6_3DMODEL_DIR", "KISYS3DMOD"):
        p = p.replace("${" + var + "}", str(os.environ.get(var) or share / "3dmodels").replace("\\", "/"))
    p = p.replace("${KIPRJMOD}", str(board_dir).replace("\\", "/"))
    if "${" in p:
        return False
    base = Path(p)
    return any(base.with_suffix(ext).is_file() for ext in (base.suffix, ".step", ".stp", ".STEP", ".wrl", ".stpZ",
                                                           ".step.gz"))


def fp_to_board(fp, x3: float, y3: float) -> tuple[float, float]:
    """Point of a footprint's 3D frame (x, y = -footprint y) -> board coordinates in mm (KiCad rotation, y down)."""
    import pcbnew
    th = math.radians(fp.GetOrientationDegrees())
    x, y = x3, -y3
    px, py = pcbnew.ToMM(fp.GetPosition().x), pcbnew.ToMM(fp.GetPosition().y)
    return (px + x * math.cos(th) + y * math.sin(th), py - x * math.sin(th) + y * math.cos(th))


def set_show(fp, show: bool) -> int:
    """Set m_Show on every model of a footprint (iterating Models() yields copies, so the list is rebuilt)."""
    import pcbnew
    ms = fp.Models()
    keep = []
    for m in ms:
        c = pcbnew.FP_3DMODEL()
        c.m_Filename, c.m_Offset, c.m_Rotation, c.m_Scale, c.m_Opacity = (m.m_Filename, m.m_Offset, m.m_Rotation,
                                                                          m.m_Scale, m.m_Opacity)
        c.m_Show = show
        keep.append(c)
    ms.clear()
    for c in keep:
        ms.push_back(c)
    return len(keep)


def attach(fp, filename: str, offset: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
    import pcbnew
    m = pcbnew.FP_3DMODEL()
    m.m_Filename = filename
    m.m_Offset = pcbnew.VECTOR3D(*offset)
    m.m_Rotation = pcbnew.VECTOR3D(0, 0, 0)
    m.m_Scale = pcbnew.VECTOR3D(1, 1, 1)
    m.m_Show = True
    fp.Models().push_back(m)


def prepare(board_in: Path, kicad_dir: Path, manifest: dict, share: Path) -> dict:
    """Write the three derived copies; return what was done (paths, attachments, focus points, missing models)."""
    import pcbnew
    models = manifest["models"]
    d3 = kicad_dir / "3d"
    if d3.resolve() != MODELS_DIR.resolve():                    # proof runs on a scratch copy of the board
        d3.mkdir(parents=True, exist_ok=True)
        for rec in models.values():
            shutil.copyfile(MODELS_DIR / rec["file"], d3 / rec["file"])
    for rec in models.values():
        if sha256(d3 / rec["file"]) != rec["sha256"]:
            raise RuntimeError(f"{rec['file']}: differs from models_manifest.json - run make_module_models.py again")

    info: dict = {"boards": {}, "attached": {}, "hidden": {}, "focus_mm": {}, "library_models_missing": [],
                  "project_model_used": {}}
    out_paths = {"D": kicad_dir / "render_as_delivered.kicad_pcb", "C": kicad_dir / "render_completed.kicad_pcb",
                 "L": kicad_dir / "render_completed_limit.kicad_pcb"}
    for key, dst in out_paths.items():
        if dst.resolve() == board_in.resolve():
            raise RuntimeError("the input board must not be one of the derived render copies")
        board = pcbnew.LoadBoard(str(board_in))
        fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
        bb = board.GetBoardEdgesBoundingBox()
        edges = [pcbnew.ToMM(v) for v in (bb.GetLeft(), bb.GetTop(), bb.GetRight(), bb.GetBottom())]
        centre = ((edges[0] + edges[2]) / 2, (edges[1] + edges[3]) / 2)
        hidden, attached = [], []

        def need(ref: str):
            if ref not in fps:
                raise RuntimeError(f"{board_in.name}: footprint {ref} not on the board")
            return fps[ref]

        def attach_model(name: str) -> None:
            rec = models[name]
            fp = need(rec["attach_ref"])
            lib_id = fp.GetFPID().GetUniStringLibId()
            if lib_id != rec["footprint"]:
                raise RuntimeError(f"{rec['attach_ref']}: footprint is {lib_id}, the model {name} was written for "
                                   f"{rec['footprint']}")
            if fp.IsDNP():
                raise RuntimeError(f"{rec['attach_ref']} is DNP: kicad-cli would not draw the model {name}")
            attach(fp, "${KIPRJMOD}/3d/" + rec["file"])
            attached.append(f"{rec['attach_ref']} <- {rec['file']}")

        if key == "D":                                          # library models that do not resolve (reported once)
            for ref in sorted(fps):
                if fps[ref].IsDNP():
                    continue
                for m in fps[ref].Models():
                    if m.m_Show and not model_file_on_disk(m.m_Filename, kicad_dir, share):
                        info["library_models_missing"].append(f"{ref}: {m.m_Filename}")
        for ref in (HIDE_AS_DELIVERED if key == "D" else HIDE_ALWAYS):
            set_show(need(ref), False)
            hidden.append(ref)
        for ref, name in PROJECT_MODEL_IF_LIBRARY_MISSING.items():
            fp = need(ref)
            if not any(model_file_on_disk(m.m_Filename, kicad_dir, share) for m in fp.Models()):
                set_show(fp, False)
                attach_model(name)
                info["project_model_used"][ref] = models[name]["file"]
        if key in ("C", "L"):
            for name in ATTACH_COMPLETED:
                attach_model(name)
        if key == "L":
            rec = models[LIMIT_MODEL]
            anchor = need(LIMIT_ANCHOR)
            if abs(anchor.GetOrientationDegrees()) > 1e-9:
                raise RuntimeError(f"{LIMIT_ANCHOR} is rotated: the limit frame needs an unrotated anchor footprint")
            size = (edges[2] - edges[0], edges[3] - edges[1])
            want = (rec["info"]["board_x"], rec["info"]["board_y"])
            if abs(size[0] - want[0]) > OUTLINE_TOL or abs(size[1] - want[1]) > OUTLINE_TOL:
                raise RuntimeError(f"board edges {size[0]:.3f} x {size[1]:.3f} mm differ from the plan outline {want}")
            ax, ay = pcbnew.ToMM(anchor.GetPosition().x), pcbnew.ToMM(anchor.GetPosition().y)
            attach(anchor, "${KIPRJMOD}/3d/" + rec["file"], (centre[0] - ax, -(centre[1] - ay), 0.0))
            attached.append(f"{LIMIT_ANCHOR} <- {rec['file']} (marker)")
        if key == "C":
            for view in VIEWS:
                f = view["focus"]
                if f is None:
                    continue
                if f[0] == "model":
                    rec = models[f[1]]
                    lo, hi = rec["bbox"]["min"], rec["bbox"]["max"]
                    x, y = fp_to_board(need(rec["attach_ref"]), (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2)
                else:
                    fp = need(f[1])
                    x, y = pcbnew.ToMM(fp.GetPosition().x), pcbnew.ToMM(fp.GetPosition().y)
                if len(f) >= 4:
                    x, y = x + f[2], y + f[3]
                info["focus_mm"][view["id"]] = [round(x, 3), round(y, 3)]
            info["board_centre_mm"] = [round(centre[0], 4), round(centre[1], 4)]
            info["board_edges_mm"] = [round(e, 4) for e in edges]
        pcbnew.SaveBoard(str(dst), board)
        info["boards"][key] = dst
        info["attached"][key] = attached
        info["hidden"][key] = hidden
    return info


# ------------------------------------------------------------------------------------------------- render
def render_cmd(cli: Path, view: dict, board: Path, png: Path, info: dict, quality: str) -> list[str]:
    cmd = [str(cli), "pcb", "render", "-o", str(png), "-w", str(WIDTH), "-h", str(HEIGHT), "--quality", quality,
           "--background", "opaque", "--side", view["side"], "--zoom", f"{view['zoom']:g}"]
    if view["rotate"] is not None:
        cmd += ["--rotate", ",".join(f"{a:g}" for a in view["rotate"])]
    if view["persp"]:
        cmd.append("--perspective")
    if view["floor"]:
        cmd.append("--floor")
    if view["focus"] is not None:
        fx, fy = info["focus_mm"][view["id"]]
        cx, cy = info["board_centre_mm"]
        cmd += ["--pivot", f"{(fx - cx) / 10:.3f},{-(fy - cy) / 10:.3f},0"]
    return cmd + [str(board)]


def label_limit(png: Path, limit: float) -> None:
    """Explain the red frame of view b4 inside the picture (Korean when Malgun Gothic is installed)."""
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        return
    img = Image.open(png).convert("RGB")
    font_path = Path("C:/Windows/Fonts/malgun.ttf")
    if font_path.is_file():
        font, text = ImageFont.truetype(str(font_path), 34), f"빨간 테두리 = 기판 윗면에서 {limit:g} mm (높이 한계)"
    else:
        font, text = ImageFont.load_default(), f"red frame = {limit:g} mm above the board top (height limit)"
    draw = ImageDraw.Draw(img)
    box = draw.textbbox((0, 0), text, font=font)
    x, y = 40, img.height - (box[3] - box[1]) - 60
    draw.rectangle((x - 14, y - 10, x + (box[2] - box[0]) + 14, y + (box[3] - box[1]) + 22), fill=(255, 255, 255),
                   outline=(200, 30, 30), width=3)
    draw.text((x, y), text, font=font, fill=(160, 20, 20))
    img.save(png)


# ------------------------------------------------------------------------------------------------- slides e1-e4
# The slides only compose pictures that were rendered before (views a1..d1 by this script, d2 by render_d2_scene.py).
# Every number and pin name on them is read from spec/carrier_spec.json, spec/module_dimensions.json,
# kicad/3d/models_manifest.json, out/render/d2_neo7m_top_cap.json or the board; nothing is typed in by hand.
SLIDE_FILES = {"e1": "e1_delivered_vs_completed.png", "e2a": "e2a_gy521_J1.png", "e2b": "e2b_bmp280_J2.png",
               "e2c": "e2c_openlog_J5.png", "e3": "e3_gps_lead_J7.png", "e4": "e4_overview_callouts.png"}
D2_PNG = "d2_neo7m_top_cap.png"
BOARD_RATIO_TOL = 0.01         # top view: detected board box must have the outline's aspect ratio within 1 %
COLOUR_KO = {"red": "빨강", "yellow": "노랑", "green": "초록", "black": "검정"}


def module_pin_names(text: str, n: int) -> list[str]:
    """The n pin names written as one space-separated run of upper-case tokens in a dimensions-file source text."""
    import re
    tok = r"[A-Z][A-Z0-9]{1,3}"
    m = re.search(rf"(?<![A-Za-z0-9])({tok}(?: {tok}){{{n - 1}}})(?![A-Za-z0-9])", text)
    if not m:
        raise ValueError(f"no run of {n} pin names in: {text!r}")
    return m.group(1).split(" ")


def connector_pins(spec: dict) -> tuple[dict, dict]:
    """{ref: {pin: net}} for every connector pin that has a net, and {net: [members]}."""
    pins: dict = {}
    members: dict = {}
    for net in spec["nets"]:
        members[net["name"]] = net["members"]
        for m in net["members"]:
            ref, pin = m.split(".", 1)
            pins.setdefault(ref, {})[pin] = net["name"]
    return pins, members


def top_view_map(png: Path, edges: list[float]):
    """mm -> pixel map of a straight top view: the board is found as the pixels that are not the bluish background
    (B < max(R, G) + 6); the box must have the outline's aspect ratio (else the map would be wrong)."""
    from PIL import Image, ImageChops
    with Image.open(png) as im:
        r, g, b = im.convert("RGB").split()
    mask = ImageChops.subtract(ImageChops.lighter(r, g).point(lambda v: min(255, v + 6)), b).point(
        lambda v: 255 if v > 0 else 0)
    bb = mask.getbbox()
    want = (edges[2] - edges[0]) / (edges[3] - edges[1])
    got = (bb[2] - bb[0]) / (bb[3] - bb[1])
    if abs(got / want - 1) > BOARD_RATIO_TOL:
        raise RuntimeError(f"{png.name}: board box {bb} has aspect {got:.4f}, outline {want:.4f}")

    def px(x: float, y: float) -> tuple[float, float]:
        return (bb[0] + (x - edges[0]) / (edges[2] - edges[0]) * (bb[2] - bb[0]),
                bb[1] + (y - edges[1]) / (edges[3] - edges[1]) * (bb[3] - bb[1]))
    return px, bb


def make_slides(board_in: Path, out: Path) -> dict:
    """Compose e1-e4 (1920 x 1080, Korean) from the rendered views; returns {slide id: file}."""
    import re

    import pcbnew
    from PIL import Image, ImageChops, ImageDraw
    import slides_ko as sk

    need = [f"{v['file']}.png" for v in VIEWS] + [D2_PNG]
    missing = [n for n in need if not (out / n).is_file()]
    if missing:
        raise SystemExit(f"slides need the rendered views first; missing in {out}: {missing}")
    spec = json.loads((ROOT / "spec" / "carrier_spec.json").read_text(encoding="utf-8"))
    dims = json.loads((ROOT / "spec" / "module_dimensions.json").read_text(encoding="utf-8"))
    models = json.loads((MODELS_DIR / "models_manifest.json").read_text(encoding="utf-8"))["models"]
    d2 = json.loads((out / D2_PNG).with_suffix(".json").read_text(encoding="utf-8"))
    parts = {p["refdes"]: p for p in spec["parts"]}
    pins, members = connector_pins(spec)
    board = pcbnew.LoadBoard(str(board_in))
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    bb = board.GetBoardEdgesBoundingBox()
    edges = [pcbnew.ToMM(v) for v in (bb.GetLeft(), bb.GetTop(), bb.GetRight(), bb.GetBottom())]
    px, board_px = top_view_map(out / "a1_as_delivered_top.png", edges)
    with Image.open(out / "a1_as_delivered_top.png") as im:
        view_size = im.size              # kicad-cli on this PC writes 1904 x 1064 for -w 1920 -h 1080 (measured)
    footer = f"그림의 기판: {board_in.name} (sha256 {sha256(board_in)[:12]}). 맞춤 검사의 증거가 아님."
    probe = ImageDraw.Draw(Image.new("RGB", (W_SLIDE, 60)))
    if len(sk.wrap(probe, f"{footer}   {sk.ASSUMED_NOTE}", sk.font(21), W_SLIDE - 2 * sk.MARGIN)) != 1:
        raise RuntimeError("slide footer does not fit on one line: shorten it")   # compose_slide draws one line only

    def ref_list(refs: list[str]) -> str:
        """['J3', 'J19', 'J20', 'J21'] -> 'J3, J19~J21'."""
        out_, run = [], []
        for r in refs + [None]:
            if r is not None and run and int(r[1:]) == int(run[-1][1:]) + 1:
                run.append(r)
                continue
            if run:
                out_.append(run[0] if len(run) == 1 else f"{run[0]}~{run[-1]}")
            run = [r] if r is not None else []
        return ", ".join(out_)

    def pin_key(p: str) -> tuple:
        return (0, int(p)) if p.isdigit() else (1, p)

    def pad_count(ref: str) -> int:
        return len({p.GetNumber() for p in fps[ref].Pads() if p.GetNumber()})

    def model_box(name: str) -> tuple[float, float, float, float]:
        rec = models[name]
        lo, hi = rec["bbox"]["min"], rec["bbox"]["max"]
        pts = [fp_to_board(fps[rec["attach_ref"]], x, y) for x in (lo[0], hi[0]) for y in (lo[1], hi[1])]
        return (min(p[0] for p in pts), min(p[1] for p in pts), max(p[0] for p in pts), max(p[1] for p in pts))

    def crop_px(box: tuple[float, float, float, float], aspect: float, margin: float = 3.0) -> tuple[int, ...]:
        x0, y0, x1, y1 = box[0] - margin, box[1] - margin, box[2] + margin, box[3] + margin
        w, h = x1 - x0, y1 - y0
        if w / h < aspect:
            x0, x1 = (x0 + x1) / 2 - h * aspect / 2, (x0 + x1) / 2 + h * aspect / 2
        else:
            y0, y1 = (y0 + y1) / 2 - w / aspect / 2, (y0 + y1) / 2 + w / aspect / 2
        (a, b), (c, d) = px(x0, y0), px(x1, y1)
        lo_x, lo_y, hi_x, hi_y = board_px[0] - 24, board_px[1] - 24, board_px[2] + 24, board_px[3] + 24
        if c - a <= hi_x - lo_x:                        # slide the region back over the board, never off the picture
            shift = max(lo_x - a, 0) - max(c - hi_x, 0)
            a, c = a + shift, c + shift
        if d - b <= hi_y - lo_y:
            shift = max(lo_y - b, 0) - max(d - hi_y, 0)
            b, d = b + shift, d + shift
        sx = max(-a, 0) - max(c - view_size[0], 0)                    # and never past the picture's own edges
        sy = max(-b, 0) - max(d - view_size[1], 0)
        return (round(a + sx), round(b + sy), round(c + sx), round(d + sy))

    def pair(box: tuple[float, float, float, float], width: int = 900) -> Image.Image:
        """Same region of a1 (as delivered) over b1 (completed), each with its label bar."""
        region = crop_px(box, 1.9)
        bar = 52
        tiles = []
        for name, key in (("a1_as_delivered_top.png", "delivered"), ("b1_completed_top.png", "completed")):
            with Image.open(out / name) as im:
                t = im.convert("RGB").crop(region)
            t = t.resize((width, round(width * t.height / t.width)), Image.LANCZOS)
            tile = Image.new("RGB", (width, t.height + bar), (255, 255, 255))
            dr = ImageDraw.Draw(tile)
            dr.rectangle([0, 0, width, bar], fill=sk.BADGE_BG[key])
            dr.text((16, 9), sk.BADGE_TEXT[key], font=sk.font(28, True), fill=(255, 255, 255))
            tile.paste(t, (0, bar))
            tiles.append(tile)
        img = Image.new("RGB", (width, sum(t.height for t in tiles) + 16), (255, 255, 255))
        img.paste(tiles[0], (0, 0))
        img.paste(tiles[1], (0, tiles[0].height + 16))
        return img

    def pin_rows(ref: str, names: list[str]) -> list[list[str]]:
        rows = []
        for i, name in enumerate(names, start=1):
            net = pins.get(ref, {}).get(str(i))
            if net is None:
                text = "연결 없음"
            else:
                others = sorted({m.split(".")[0] for m in members[net]} - {ref})
                text = net
                if others and all(parts.get(o, {}).get("fit") == "none" for o in others):
                    text = f"{net} (상대 부품 {', '.join(others)} 미실장: 쓰이지 않음)"
            rows.append([str(i), name, text])
        if pad_count(ref) != len(names):
            raise RuntimeError(f"{ref}: {pad_count(ref)} pads on the board, {len(names)} module pin names")
        return rows

    written: dict = {}
    # ---------------------------------------------------------------- e1: as delivered vs completed + who fits what
    n_jlc = sum(1 for p in spec["parts"] if p.get("fit") == "jlc")
    mcu = [r for r, p in parts.items() if p.get("fit") == "jlc_optional"]
    mcu_pins = sum(pad_count(r) for r in mcu)
    prog = sorted((r for r, p in parts.items() if p.get("fit") == "none" and "PinHeader" in p["footprint"]),
                  key=lambda r: int(r[1:]))
    prog_pins = sum(pad_count(r) for r in prog)
    j1_pins = pad_count("J1")
    who = sk.Table("누가 무엇을 하나 (spec 파일 기준)", ["누가", "무엇을"], [
        ["JLC", f"부품 {n_jlc}개 실장: ESP-12F, 소켓 J2·J5, GPS 커넥터 J7, 전원·서보 커넥터 포함"],
        ["사용자", f"MCU {len(mcu)}개 ({', '.join(mcu)}) 납땜, 핀 {mcu_pins}개 (JLC에 맡기는 파일 쌍도 있음)"],
        ["사용자", f"GY-521: 모듈 핀 스트립 {j1_pins}핀을 J1 구멍에 납땜"],
        ["사용자", "BMP280: ㄱ자 헤더로 J2 소켓에 꽂기"],
        ["사용자", "OpenLog: ㄱ자 6핀 헤더를 모듈에 납땜한 뒤 J5에 꽂기"],
        ["사용자", "NEO-7M: 윗뚜껑에 붙이고 4선 리드선을 J7에 꽂기"],
        ["사용자 (필요할 때)", f"프로그래밍 헤더 {len(prog)}곳 ({ref_list(prog)}, {prog_pins}핀): 빈 구멍으로 받음"],
    ], col_weights=[1.15, 3.0])
    sk.compose_slide(
        "JLC에서 받은 상태 vs 직접 장착한 뒤", [
            sk.Panel(out / "a2_as_delivered_front.png", "JLC에서 받는 상태. MCU 자리 U1~U4, J1 구멍, 프로그래밍 구멍은 비어 있음",
                     badge="delivered", crop=(0.17, 0.17, 0.87, 1.0)),
            sk.Panel(out / "b2_completed_front.png", "MCU 4개와 GY-521 납땜, BMP280·OpenLog와 GPS 플러그를 꽂은 뒤. 모듈 모양은 가정 치수",
                     assumed=True, badge="completed", crop=(0.17, 0.17, 0.87, 1.0)),
            sk.Panel(out / "a1_as_delivered_top.png", "윗면, 받은 상태", badge="delivered", crop=(0.2, 0.0, 0.8, 1.0)),
            sk.Panel(out / "b1_completed_top.png", "윗면, 장착한 뒤 (모듈 가정 치수)", assumed=True, badge="completed",
                     crop=(0.2, 0.0, 0.8, 1.0)),
        ], out / SLIDE_FILES["e1"], subtitle="캐리어 기판 rev 3", table=who, footer=footer, cols=2, table_w=600)
    written["e1"] = SLIDE_FILES["e1"]

    # ---------------------------------------------------------------- e2a-c: one slide per sensor module
    sensors = (
        ("e2a", "J1", "gy521_on_J1", "c1_gy521_on_J1.png", "GY-521 (MPU-6050)",
         module_pin_names(dims["gy521"]["header_n"]["src"], dims["gy521"]["header_n"]["v"]),
         dims["mounting"]["J1"]["pin1_is"]["tag"] == "A",
         "모듈을 J1에 납땜한 모습: 부품 면이 위, 1번 핀 VCC, 모듈 밑면은 기판 위 "
         f"{models['gy521_on_J1']['info'].get('underside_z', dims['gy521']['support_h']['v'])} mm (가정)",
         "JLC: J1은 빈 도금 구멍 8개 (부품 없음). 사용자: 모듈의 핀 스트립을 J1에 납땜. 모듈이 U1 위에 걸치므로 "
         "U1을 먼저 납땜하고, 모듈 아래에 받침(약 2.5 mm, 가정)을 둡니다. 모듈 핀 이름 순서는 판매 페이지 값(가정)."),
        ("e2b", "J2", "bmp280_in_J2", "c2_bmp280_in_J2.png", "BMP280 (TYE-BMP29)",
         module_pin_names(dims["bmp280"]["header_n"]["src"], dims["bmp280"]["header_n"]["v"]),
         dims["bmp280"]["face_up"]["tag"] == "A",
         "ㄱ자 헤더로 J2 소켓에 꽂아 눕힌 모습: 모듈 밑면은 기판 위 "
         f"{models['bmp280_in_J2']['info']['underside_z']} mm, 센서 면이 기판 쪽 (가정)",
         "JLC: ㄱ자 소켓 J2 실장. 사용자: 모듈을 J2에 꽂기. 받은 모듈에 곧은 헤더가 달려 있으면 ㄱ자 헤더로 바꿔 "
         "납땜해야 합니다 (헤더 종류와 핀 인쇄 순서는 확인 안 됨, 가정)."),
        ("e2c", "J5", "openlog_in_J5", "c3_openlog_in_J5.png", "OpenLog (microSD 기록기)",
         module_pin_names(dims["openlog"]["header_x"]["src"], dims["openlog"]["header_n"]["v"]),
         dims["openlog"]["face_up"]["tag"] == "A",
         "microSD 면이 위로 오게 J5에 꽂은 모습. 가정한 맞춤이며, 실제로 들어가는지는 실물을 재서 결정합니다 (가정)",
         "JLC: ㄱ자 소켓 J5 실장. 사용자: OpenLog에 ㄱ자 6핀 헤더를 납땜하고 J5에 꽂기. 핀 이름 순서는 "
         "OpenLog.brd 인쇄 순서(문서 값). 모듈 아래 부품 높이와 카드 돌출은 가정 값이라 맞춤은 확인 안 됨."),
    )
    sensor_subtitle = {"J1": "사용자가 J1 구멍에 직접 납땜", "J2": "JLC가 단 소켓 J2에 꽂기",
                       "J5": "JLC가 단 소켓 J5에 꽂기 (맞춤은 실측 후 결정)"}
    for sid, ref, model, closeup, title, names, assumed, cap, note in sensors:
        box = model_box(model)
        fb = fps[ref].GetBoundingBox(False)
        fbox = [pcbnew.ToMM(v) for v in (fb.GetLeft(), fb.GetTop(), fb.GetRight(), fb.GetBottom())]
        box = (min(box[0], fbox[0]), min(box[1], fbox[1]), max(box[2], fbox[2]), max(box[3], fbox[3]))
        table = sk.Table(f"{ref} 핀 연결 (spec 파일에서 읽음)", [f"{ref} 핀", "모듈 핀", "기판 신호"],
                         pin_rows(ref, names), assumed=assumed, note=f"누가 무엇을: {note}", col_weights=[0.8, 1.1, 3.0])
        sk.compose_slide(
            f"센서 장착: {title} → {ref}", [
                sk.Panel(pair(box), "윗면 같은 자리: 위는 받은 상태, 아래는 장착한 뒤 (모듈 가정 치수)", assumed=True),
                sk.Panel(out / closeup, cap, assumed=True, badge="completed", crop=(0.2, 0.0, 0.8, 1.0)),
            ], out / SLIDE_FILES[sid], subtitle=sensor_subtitle[ref], table=table, footer=footer,
            cols=2, table_w=600)
        written[sid] = SLIDE_FILES[sid]

    # ---------------------------------------------------------------- e3: GPS lead
    gps_names = module_pin_names(dims["neo7m"]["header_n"]["src"], dims["neo7m"]["header_n"]["v"])
    colours = dict(re.findall(r"(?:pad )?(\d) \S+ (red|yellow|green|black)", models["gh_plug_in_J7"]["info"]["wire_order"]))
    jp1 = re.search(r"1-2 = ([\d.]+ V)", parts["JP1"]["role"])
    rows = []
    for p in sorted(pins["J7"], key=pin_key):
        net = pins["J7"][p]
        mod = next((n for n in gps_names if n in net.split("_")), "-")
        text = net + (f" (JP1 1-2 = {jp1.group(1)} 기본)" if net == "GPS_VCC" and jp1 else "")
        if p == "MP":
            text, mod = f"{net} (고정 탭)", "-"
        rows.append([p, text, COLOUR_KO.get(colours.get(p, ""), "-"), mod])
    e3 = sk.compose_slide(
        "GPS 리드선: NEO-7M → J7", [
            sk.Panel(out / D2_PNG, "NEO-7M을 윗뚜껑 위에 두고 4선 리드선을 J7까지 내린 모습. 모듈 치수와 방향, 리드선 경로는 "
                     "가정이며 간섭 검사는 하지 않았음", assumed=True, badge="completed")],
        out / SLIDE_FILES["e3"], subtitle="JST-GH 1.25 mm 4핀, 윗면 꽂음", footer=footer, table_w=560,
        table=sk.Table("J7 핀 연결 (spec 파일에서 읽음)", ["J7 핀", "기판 신호", "선 색", "모듈 핀"], rows, assumed=True,
                       note=f"선 색과 NEO-7M 쪽 핀 순서는 가정 (실물 확인). 리드선 길이: 차체 CAD 예산 "
                            f"{d2['lead_len_vehicle_cad_mm']['v']:g} mm (cad/layout.py).",
                       col_weights=[0.75, 2.5, 0.85, 1.0]))
    # inset: the plug close-up d1 under the table (the table's last drawn row is measured, the inset never covers it)
    slide = Image.open(e3).convert("RGB")
    iw = 560
    x0 = W_SLIDE - sk.MARGIN - iw
    col = slide.crop((x0, sk.TITLE_H, W_SLIDE - sk.MARGIN, H_SLIDE - sk.FOOT_H))
    bg = Image.new("RGB", col.size, sk.BG)
    table_bottom = sk.TITLE_H + ImageChops.difference(col, bg).getbbox()[3]
    y_cap = H_SLIDE - sk.FOOT_H - sk.MARGIN - 34
    room = y_cap - 8 - (table_bottom + 20)
    if room < 200:
        raise RuntimeError(f"e3: only {room} px under the J7 table for the plug picture")
    with Image.open(out / "d1_gps_plug_J7.png") as im:
        d1 = im.convert("RGB")
    ih = min(room, round(iw * 0.75))
    cw = round(d1.height * 0.8 * iw / ih)                                 # crop around the plug, inset aspect kept
    cx0 = max(0, min(d1.width - cw, 1100 - cw // 2))
    d1 = d1.crop((cx0, 0, cx0 + cw, round(d1.height * 0.8))).resize((iw, ih), Image.LANCZOS)
    dr = ImageDraw.Draw(slide)
    f_cap = sk.font(22)
    cap = sk.caption_text("J7에 꽂은 GH 플러그 (리드선 모양은 가정)", True)
    y0 = y_cap - 8 - d1.height
    slide.paste(d1, (x0, y0))
    dr.rectangle([x0, y0, x0 + iw, y0 + d1.height], outline=sk.PANEL_EDGE, width=2)
    sk._draw_lines(dr, (x0, y_cap), [cap], f_cap, sk.INK)
    slide.save(e3, format="PNG")
    written["e3"] = SLIDE_FILES["e3"]

    # ---------------------------------------------------------------- e4: overview with call-outs on the top view
    img = Image.open(out / "b1_completed_top.png").convert("RGB")
    dr = ImageDraw.Draw(img)
    keep = next((z for z in board.Zones() if z.GetZoneName() == "ESP12E_antenna_keepout"), None)
    if keep is not None:
        kb = keep.GetBoundingBox()
        (ka, kb_), (kc, kd) = px(max(pcbnew.ToMM(kb.GetLeft()), edges[0]), max(pcbnew.ToMM(kb.GetTop()), edges[1])), \
            px(min(pcbnew.ToMM(kb.GetRight()), edges[2]), min(pcbnew.ToMM(kb.GetBottom()), edges[3]))
        for i in range(int(ka), int(kc), 18):                            # dashed red outline
            dr.line([i, kb_, min(i + 10, kc), kb_], fill=sk.ACCENT, width=4)
            dr.line([i, kd, min(i + 10, kc), kd], fill=sk.ACCENT, width=4)
        for j in range(int(kb_), int(kd), 18):
            dr.line([ka, j, ka, min(j + 10, kd)], fill=sk.ACCENT, width=4)
            dr.line([kc, j, kc, min(j + 10, kd)], fill=sk.ACCENT, width=4)

    module_on = {rec["attach_ref"]: name for name, rec in models.items() if name in ATTACH_COMPLETED}

    def at(ref: str, dx: float = 0.0, dy: float = 0.0) -> tuple[float, float]:
        """Footprint origin, or the plan centre of the module model attached to that connector (+ dx, dy mm)."""
        if ref in module_on and ref != "J7":
            b4 = model_box(module_on[ref])
            return px((b4[0] + b4[2]) / 2 + dx, (b4[1] + b4[3]) / 2 + dy)
        f = fps[ref]
        return px(pcbnew.ToMM(f.GetPosition().x) + dx, pcbnew.ToMM(f.GetPosition().y) + dy)

    def role(ref: str) -> str:
        return parts[ref]["role"].split(" (")[0].split(":")[0]

    left_x, right_x = 10, board_px[2] + 14
    calls = [
        ("U7", 0, -4, "U7 ESP-12F 무선 모듈 (JLC 실장)\n빨간 점선 = 안테나 구리 금지 영역", False, left_x),
        ("J14", 0, 0, "J14 1S 배터리 입력 (JLC)", False, left_x),
        ("J10", -4, 0, f"J8~J11 날개 서보 4개 (JLC)", False, left_x),
        ("U3", 0, 0, f"U3 {role('U3').split()[0]} MCU (사용자 납땜)", False, left_x),
        ("J21", 0, 0, f"{ref_list(prog)}\n프로그래밍 헤더 자리 (빈 구멍,\n필요할 때 직접 납땜)", False, left_x),
        ("U4", 0, 0, f"U4 {role('U4').split()[0]} MCU (사용자 납땜)", False, left_x),
        ("J7", 0, -2, "J7 GPS 리드선 커넥터 (JLC)\n플러그와 선은 그림용", True, left_x),
        ("J2", 0, 0, "J2 BMP280 소켓 (JLC) + 모듈\n모듈은 가정 치수", True, right_x),
        ("U2", 4, 2, f"U2 {role('U2').split()[0]} MCU (사용자 납땜)", False, right_x),
        ("J5", 0, 0, "J5 OpenLog 소켓 (JLC) + 모듈\n맞춤은 실측으로 결정", True, right_x),
        ("J1", 0, 0, f"J1 GY-521 (사용자 납땜)\nU1 {role('U1').split()[0]} MCU 위에 걸침 - U1 먼저", True, right_x),
        ("L5", 0, 0, "L5·U8 5 V 승압 회로 (JLC)", False, right_x),
    ]
    callouts = [sk.Callout(text, at(ref, dx, dy), (bx, at(ref, dx, dy)[1] - 30), assumed)
                for ref, dx, dy, text, assumed, bx in calls]
    ann = sk.annotate(img, callouts, size=25,
                      banner="모듈, GPS 플러그와 선은 판매 페이지 값과 가정 치수로 그린 것입니다. 실제 맞춤은 실물을 재서 확인합니다.",
                      banner_assumed=True)
    sk.compose_slide("한눈에 보기: 누가 어디를 장착하나", [
        sk.Panel(ann, "윗면, 직접 장착한 뒤 (모듈은 가정 치수). 'JLC' = JLC가 실장, '사용자' = 직접 납땜하거나 꽂음",
                 assumed=True, badge="completed")], out / SLIDE_FILES["e4"], subtitle="캐리어 기판 rev 3", footer=footer)
    written["e4"] = SLIDE_FILES["e4"]
    report = {"schema": "vtol_carrier.render_slides/1", "generated_by": "scripts/render_rev3.py --slides-only",
              "label": "composed illustrations; '가정' marks assumed dimensions / routes; not fit evidence",
              "source_board": board_in.name, "source_board_sha256": sha256(board_in),
              "inputs": {n: sha256(out / n) for n in need},
              "slides": {k: {"file": v, "sha256": sha256(out / v)} for k, v in written.items()},
              "board_box_px_a1": list(board_px)}
    (out / "slides_report.json").write_bytes((json.dumps(report, indent=1, sort_keys=True, ensure_ascii=False)
                                              + "\n").encode("utf-8"))
    return written


W_SLIDE, H_SLIDE = 1920, 1080


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--board", type=Path, default=ROOT / "kicad" / "vtol_carrier.kicad_pcb")
    ap.add_argument("--kicad-dir", type=Path, default=None, help="folder for the derived copies (default: the board's)")
    ap.add_argument("--out", type=Path, default=ROOT / "out" / "render")
    ap.add_argument("--views", default="", help="comma-separated view ids (default: all)")
    ap.add_argument("--quality", default="high", choices=("high", "basic"))
    ap.add_argument("--prepare-only", action="store_true", help="write the derived boards, render nothing")
    ap.add_argument("--slides-only", action="store_true",
                    help="compose the slides e1-e4 from the rendered views and d2 (run after render_d2_scene.py)")
    args = ap.parse_args(argv)
    ensure_pcbnew()
    if args.slides_only:
        for sid, name in make_slides(args.board.resolve(), args.out.resolve()).items():
            print(f"{sid} ok  {name}")
        return 0

    cli = find_kicad_cli()
    share = cli.parent.parent / "share" / "kicad"
    board_in = args.board.resolve()
    kicad_dir = (args.kicad_dir or board_in.parent).resolve()
    manifest = json.loads((MODELS_DIR / "models_manifest.json").read_text(encoding="utf-8"))
    wanted = [v.strip() for v in args.views.split(",") if v.strip()] or [v["id"] for v in VIEWS]
    unknown = sorted(set(wanted) - {v["id"] for v in VIEWS})
    if unknown:
        raise SystemExit(f"unknown view ids: {unknown}")

    before = {p.name for p in kicad_dir.iterdir()} if kicad_dir.is_dir() else set()
    src_hash = sha256(board_in)
    info = prepare(board_in, kicad_dir, manifest, share)
    if sha256(board_in) != src_hash:
        raise RuntimeError("the input board changed while the copies were written")
    for key in ("D", "C", "L"):
        print(f"board {key}: {info['boards'][key].name}  hidden {info['hidden'][key]}  attached {info['attached'][key]}")
    for line in info["library_models_missing"]:
        print("library model file missing:", line)
    version = run([str(cli), "version"]).stdout.strip()
    report = {"schema": "vtol_carrier.render_rev3/1", "generated_by": "scripts/render_rev3.py",
              "label": "illustrations; module dimensions are F / A as tagged in spec/module_dimensions.json; "
                       "not fit evidence",
              "kicad_cli_version": version, "source_board": board_in.name, "source_board_sha256": src_hash,
              "derived_boards": {info["boards"][k].name: sha256(info["boards"][k]) for k in ("D", "C", "L")},
              "models": {n: r["sha256"] for n, r in sorted(manifest["models"].items())},
              "hidden": info["hidden"], "attached": info["attached"], "project_model_used": info["project_model_used"],
              "library_models_missing": info["library_models_missing"], "focus_mm": info["focus_mm"],
              "quality": args.quality, "size_requested": [WIDTH, HEIGHT], "views": {}}
    failed = 0
    if not args.prepare_only:
        args.out.mkdir(parents=True, exist_ok=True)
        for view in VIEWS:
            if view["id"] not in wanted:
                continue
            png = args.out / f"{view['file']}.png"
            if png.exists():
                png.unlink()
            cmd = render_cmd(cli, view, info["boards"][view["board"]], png, info, args.quality)
            res = run(cmd)
            ok = res.returncode == 0 and png.is_file() and png.stat().st_size > 10_000
            if ok and view["id"] == "b4":
                label_limit(png, manifest["models"][LIMIT_MODEL]["info"]["limit_above_board"])
            size = None
            if ok:
                from PIL import Image
                with Image.open(png) as im:
                    size = list(im.size)        # kicad-cli may write less than -w / -h (1904 x 1064 here, measured)
            report["views"][view["id"]] = {"file": png.name, "what": view["what"], "board": info["boards"][view["board"]].name,
                                           "args": cmd[5:-1], "rendered": ok, "png_size": size,
                                           "sha256": sha256(png) if ok else None}
            print(f"{view['id']} {'ok' if ok else 'FAILED'}  {png.name}  {' '.join(cmd[5:-1])}")
            if not ok:
                failed += 1
                print((res.stdout + res.stderr)[-600:])
        (args.out / "render_report.json").write_bytes(
            (json.dumps(report, indent=1, sort_keys=True, ensure_ascii=False) + "\n").encode("utf-8"))
    # kicad-cli / pcbnew leave project side files next to a board they open: remove the ones this run created
    for p in kicad_dir.iterdir():
        if p.name not in before and p.name.startswith("render_") and p.suffix in (".kicad_prl", ".kicad_pro"):
            p.unlink()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
