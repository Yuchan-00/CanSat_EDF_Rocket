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
         --out DIR (default out/render), --views a1,c2,... (default all), --quality high|basic, --prepare-only
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


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--board", type=Path, default=ROOT / "kicad" / "vtol_carrier.kicad_pcb")
    ap.add_argument("--kicad-dir", type=Path, default=None, help="folder for the derived copies (default: the board's)")
    ap.add_argument("--out", type=Path, default=ROOT / "out" / "render")
    ap.add_argument("--views", default="", help="comma-separated view ids (default: all)")
    ap.add_argument("--quality", default="high", choices=("high", "basic"))
    ap.add_argument("--prepare-only", action="store_true", help="write the derived boards, render nothing")
    args = ap.parse_args(argv)
    ensure_pcbnew()

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
              "quality": args.quality, "size": [WIDTH, HEIGHT], "views": {}}
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
            report["views"][view["id"]] = {"file": png.name, "what": view["what"], "board": info["boards"][view["board"]].name,
                                           "args": cmd[5:-1], "rendered": ok,
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
