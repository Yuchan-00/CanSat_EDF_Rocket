"""View d2 (plan section 11): the NEO-7M on the vehicle's top cap with its lead down to J7 of the carrier board.

Invariant: this script draws, it does not decide or verify anything.  Every position comes from a named source and
is reported with its truth label in the JSON written next to the picture:
  * vehicle geometry (cap, avionics tube, tray, battery / ESC / misc boxes, the cap's 'GPS lead' hole, the GPS board /
    patch / ground-plane places): `cad/layout.py` build(config="vtol") and `cad/cadquery/rocket_geometry.py` of this
    worktree, imported read-only (label F: the vehicle CAD);
  * carrier board: the GLB that kicad-cli exports from `kicad/render_completed.kicad_pcb` (made by render_rev3.py),
    placed with x_v = Xk - cx, y_v = cy - Yk, z_v = Layout.lp_board_top_z + height above the board top;
  * NEO-7M board and antenna: the solids of make_module_models.py (dimensions tagged in spec/module_dimensions.json);
  * the lead: an ILLUSTRATIVE route (label A) from the module's header over the cap top, through the cap hole, down
    the avionics bay and into the end of the lead drawn on the J7 plug model.  No clearance is checked along it.
The picture carries a Korean banner with the word '가정' (slides_ko.annotate).

Run with the CAD interpreter (CadQuery + PyVista + trimesh), never with KiCad's python:
    PYTHONDONTWRITEBYTECODE=1 "<main checkout>/.venv/Scripts/python.exe" -B -X utf8 scripts/render_d2_scene.py
        [--board kicad/render_completed.kicad_pcb] [--out out/render/d2_neo7m_top_cap.png]
        [--board-label "rev 3 기판"]
The PNG is written by VTK (GPU / driver dependent pixels): it is a picture, not a byte-reproducible artifact.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent                      # pcb/vtol_carrier
WT = ROOT.parents[1]                    # the worktree
for p in (HERE, WT / "cad", WT / "cad" / "cadquery"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import cadquery as cq                   # noqa: E402
import pyvista as pv                    # noqa: E402
import trimesh                          # noqa: E402

import make_module_models as mm         # noqa: E402
import render_rev3                      # noqa: E402  (find_kicad_cli, run; no pcbnew needed for those)
import rocket_geometry as rg            # noqa: E402
import slides_ko                        # noqa: E402
from layout import build as build_layout  # noqa: E402

# ------------------------------------------------------------------------------------------------ scene table
WIDTH, HEIGHT = 1920, 1080
CUT_WEDGE_DEG = (160.0, 305.0)          # sector of the tube and of the cap's skirt / ribs that is cut away (deg, vehicle frame)
CAMERA = {"azimuth_deg": 236.0, "elevation_deg": 22.0, "distance": 500.0, "focus": (2.0, 0.0, 404.0), "view_angle": 26.0}
GPS_BOARD_YAW_DEG = 180.0               # A: header edge of the receiver board toward the cap hole
LEAD_WIRE_OD = None                     # None -> gh_plug.wire_od of module_dimensions.json
BACKGROUND = ("#eef1f6", "#b9c2d3")     # bottom, top

PART_COLORS = {
    "avionics_module": (0.93, 0.93, 0.91), "top_cap": (0.86, 0.87, 0.89), "avionics_tray": (0.80, 0.82, 0.86),
    "battery": (0.22, 0.34, 0.60), "esc": (0.25, 0.26, 0.28), "misc_electronics": (0.42, 0.50, 0.44),
    "foam": (0.35, 0.36, 0.38), "ground_plane": (0.74, 0.46, 0.22),
}
WIRE_GROUPS = ("wire_red", "wire_yellow", "wire_green", "wire_black")   # pad 1..4, same order as the J7 plug model


# ------------------------------------------------------------------------------------------------ helpers
def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def to_poly(shape: cq.Shape, tol: float = 0.05, ang: float = 0.25) -> pv.PolyData:
    verts, tris = shape.tessellate(tol, ang)
    v = np.array([(p.x, p.y, p.z) for p in verts], dtype=float)
    f = np.hstack([np.full((len(tris), 1), 3, dtype=np.int64), np.array(tris, dtype=np.int64)]).ravel()
    return pv.PolyData(v, f)


def wedge(th0: float, th1: float, r: float, z0: float, z1: float) -> cq.Shape:
    pts = [(0.0, 0.0)] + [(r * math.cos(math.radians(t)), r * math.sin(math.radians(t))) for t in np.linspace(th0, th1, 25)]
    return cq.Workplane("XY").polyline(pts).close().extrude(z1 - z0).translate((0, 0, z0)).val()


def board_facts(board: Path) -> dict:
    """J7 position / rotation and the Edge.Cuts bounding box, read from the board file text (no pcbnew here)."""
    text = board.read_text(encoding="utf-8")
    k = text.find('(property "Reference" "J7"')
    if k < 0:
        raise ValueError(f"{board.name}: no footprint J7")
    start = text.rfind("\n\t(footprint ", 0, k)
    m = re.search(r'\(footprint "([^"]+)".*?\(at\s+(-?[\d.]+)\s+(-?[\d.]+)(?:\s+(-?[\d.]+))?\)', text[start:k], re.S)
    if not m:
        raise ValueError(f"{board.name}: J7 has no (at ...)")
    if m.group(1) != mm.FOOTPRINTS["gh_plug_in_J7"]:
        raise ValueError(f"{board.name}: J7 is {m.group(1)}, the plug model was made for {mm.FOOTPRINTS['gh_plug_in_J7']}")
    rot = float(m.group(4) or 0.0)
    xs, ys = [], []
    for blk in re.finditer(r"\n\t\((gr_line|gr_arc|gr_rect|gr_poly)\b(.*?)\n\t\)", text, re.S):
        if '(layer "Edge.Cuts")' not in blk.group(2):
            continue
        for q in re.finditer(r"\((?:start|end|mid|xy)\s+(-?[\d.]+)\s+(-?[\d.]+)\)", blk.group(2)):
            xs.append(float(q.group(1)))
            ys.append(float(q.group(2)))
    if not xs:
        raise ValueError(f"{board.name}: no Edge.Cuts graphics found")
    return {"j7": (float(m.group(2)), float(m.group(3))), "j7_rot": rot,
            "edge": (min(xs), min(ys), max(xs), max(ys)), "centre": ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)}


def export_glb(board: Path, glb: Path) -> None:
    cli = render_rev3.find_kicad_cli()
    before = {p.name for p in board.parent.iterdir()}
    res = render_rev3.run([str(cli), "pcb", "export", "glb", "--force", "--include-pads", "--include-silkscreen",
                           "--include-soldermask", "--subst-models", "-o", str(glb), str(board)])
    for p in board.parent.iterdir():        # kicad-cli leaves project files beside the board: remove only the new ones
        if p.name not in before and p.suffix in (".kicad_prl", ".kicad_pro", ".lck"):
            p.unlink()
    if res.returncode != 0 or not glb.is_file():
        raise RuntimeError(f"kicad-cli pcb export glb failed ({res.returncode}): {res.stderr[-400:]}")


def carrier_meshes(glb: Path, centre: tuple[float, float], board_t: float, z_board_top: float) -> list[tuple[pv.PolyData, tuple]]:
    """The GLB (metres, Y up, x = Xk, z = Yk, y = height above the board bottom) in the vehicle frame, one mesh per colour."""
    scene = trimesh.load(str(glb))
    by_color: dict[tuple, list] = {}
    for m in scene.dump(concatenate=False):
        col = (128, 128, 128, 255)
        mat = getattr(m.visual, "material", None)
        if mat is not None and getattr(mat, "baseColorFactor", None) is not None:
            col = tuple(int(c) for c in mat.baseColorFactor)
        by_color.setdefault(col[:3], []).append(m)
    out = []
    for col in sorted(by_color):
        tm = trimesh.util.concatenate(by_color[col])
        v = np.asarray(tm.vertices) * 1000.0
        vv = np.column_stack([v[:, 0] - centre[0], centre[1] - v[:, 2], z_board_top + (v[:, 1] - board_t)])
        f = np.hstack([np.full((len(tm.faces), 1), 3, dtype=np.int64), np.asarray(tm.faces, dtype=np.int64)]).ravel()
        out.append((pv.PolyData(vv, f), tuple(c / 255.0 for c in col)))
    return out


def placed(model: mm.Model, yaw_deg: float, at: tuple[float, float, float]) -> dict[str, cq.Shape]:
    out = {}
    for grp, shapes in model.groups.items():
        c = cq.Compound.makeCompound(shapes).rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), yaw_deg).translate(cq.Vector(*at))
        out[grp] = c
    return out


def spline_len(pts: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())


# ------------------------------------------------------------------------------------------------ scene
def build_scene(board: Path, work: Path) -> dict:
    L = build_layout(config="vtol")
    dims = mm.load_dims()
    models = mm.build_models(dims)
    facts = board_facts(board)
    plan_x, plan_y = mm.val(dims, "height_limit", "board_x"), mm.val(dims, "height_limit", "board_y")
    ex0, ey0, ex1, ey1 = facts["edge"]
    if abs((ex1 - ex0) - plan_x) > render_rev3.OUTLINE_TOL or abs((ey1 - ey0) - plan_y) > render_rev3.OUTLINE_TOL:
        raise ValueError(f"board outline {ex1 - ex0:.3f} x {ey1 - ey0:.3f} is not the plan's {plan_x} x {plan_y}")
    if facts["j7_rot"] != 0.0:
        raise ValueError("J7 is rotated: the lead-end transform of this script assumes rotation 0 (ANCHORS)")
    cx, cy = facts["centre"]
    z_top = L.lp_board_top_z

    def veh(xk: float, yk: float, h: float) -> np.ndarray:
        return np.array([xk - cx, cy - yk, z_top + h])

    meshes: list[dict] = []

    def add(name: str, poly: pv.PolyData, color: tuple, edges: bool = False, **kw) -> None:
        meshes.append({"name": name, "poly": poly, "color": color, "edges": edges, **kw})

    # printed parts, cut open
    th0, th1 = CUT_WEDGE_DEG
    r_cut = L.body_r + 10.0
    tube = rg.avionics_module(L).cut(wedge(th0, th1, r_cut, L.z_avionics_bottom - 30.0, L.z_avionics_top + 30.0))
    add("avionics_module", to_poly(tube), PART_COLORS["avionics_module"], edges=True)
    cap = rg.top_cap(L).cut(wedge(th0, th1, r_cut, L.z_cap_skirt_bottom - 5.0, L.z_cap_plate_bottom))
    add("top_cap", to_poly(cap), PART_COLORS["top_cap"], edges=True)
    add("avionics_tray", to_poly(rg.avionics_tray(L)), PART_COLORS["avionics_tray"], edges=True)
    for nm, fn in (("battery", rg.battery), ("esc", rg.esc), ("misc_electronics", rg.misc_electronics)):
        if nm == "battery" and not L.has_battery:
            continue
        add(nm, to_poly(fn(L)), PART_COLORS[nm], edges=True)
    foam = mm.box(L.fcu_x[0], L.fcu_x[1], L.fcu_y[0], L.fcu_y[1], L.fcu_z[0] - L.fcu_foam_t, L.fcu_z[0])
    add("foam under the carrier", to_poly(foam), PART_COLORS["foam"], edges=True)

    # carrier board from KiCad
    glb = work / "carrier.glb"
    export_glb(board, glb)
    board_t = L.fcu_z[0] + (z_top - L.fcu_z[0]) - L.fcu_z[0]         # = lp_board_top_z - fcu_z[0] (board thickness in the CAD)
    for i, (poly, col) in enumerate(carrier_meshes(glb, (cx, cy), board_t, z_top)):
        add(f"carrier_{i}", poly, col)

    # NEO-7M on the cap (places of the vehicle CAD, solids of make_module_models.py)
    (bx0, bx1), (by0, by1) = L.cap_board_xy
    (px0, px1), (py0, py1) = L.cap_patch_xy
    b_c = ((bx0 + bx1) / 2, (by0 + by1) / 2, L.cap_board_z[0])
    a_c = ((px0 + px1) / 2, (py0 + py1) / 2, L.cap_patch_z[0])
    add("gps ground plane", to_poly(rg.cap_gps_ground_plane(L)), PART_COLORS["ground_plane"], edges=True)
    for grp, shp in placed(mm.neo7m_board_parts(dims), GPS_BOARD_YAW_DEG, b_c).items():
        add(f"neo7m_board__{grp}", to_poly(shp), mm.COLORS[grp])
    for grp, shp in placed(mm.neo7m_antenna_parts(dims), 0.0, a_c).items():
        add(f"neo7m_antenna__{grp}", to_poly(shp), mm.COLORS[grp])

    yaw = math.radians(GPS_BOARD_YAW_DEG)
    pcb_l, pcb_t = mm.val(dims, "neo7m", "pcb_l"), mm.val(dims, "neo7m", "pcb_t")

    def on_board(u: float, v: float, h: float) -> np.ndarray:
        return np.array([b_c[0] + u * math.cos(yaw) - v * math.sin(yaw), b_c[1] + u * math.sin(yaw) + v * math.cos(yaw), b_c[2] + h])

    header = on_board(pcb_l / 2 - 1.6, 0.0, pcb_t + 0.6)
    ufl = on_board(-13.5, 6.0, pcb_t + 1.2)
    ant_edge = np.array([a_c[0] - 4.0, py0, a_c[2] + 1.6])
    coax = pv.Spline(np.array([ufl, ufl + (4.0, -1.0, 1.5), (ufl + ant_edge) / 2 + (2.0, -5.0, 2.5), ant_edge + (0.0, -4.0, 1.0),
                               ant_edge]), 120).tube(radius=0.57, n_sides=14)
    add("antenna lead (illustrative)", coax, mm.COLORS["coax"])

    # the GPS lead: from the end of the lead drawn on the plug model up to the module header (illustrative, A)
    plug = models["gh_plug_in_J7"]
    bb = plug.bbox()
    od = LEAD_WIRE_OD or mm.val(dims, "gh_plug", "wire_od")
    jx, jy = facts["j7"]
    x3 = (bb["min"][0] + bb["max"][0]) / 2
    end = veh(jx + x3, jy - bb["max"][1], plug.info["lead_top_z"] - od / 2)       # y3d = -(fp y); J7 rotation 0
    hx, hy, hd = L.cap_holes["GPS lead"]
    hole_top = np.array([hx, hy, L.z_cap_top])
    route = np.array([
        end, end + (0.0, 5.0, 0.3), end + (-7.0, 8.0, 3.0), end + (-12.0, 2.0, 9.0), end + (-9.0, -8.0, 20.0),
        (hx - 4.0, hy - 1.0, end[2] + 34.0), (hx, hy, end[2] + 52.0), (hx, hy, L.z_cap_plate_bottom - 6.0),
        (hx, hy, L.z_cap_top + 1.0), hole_top + (header - hole_top) * 0.35 + (0.0, 0.0, 4.5),
        header + (-3.0, 0.5, 1.5), header], dtype=float)
    pitch = mm.val(dims, "gh_plug", "pitch")
    centre_line = pv.Spline(route, 600).points
    for k, grp in enumerate(WIRE_GROUPS):
        off = np.array([(k - 1.5) * pitch, 0.0, 0.0])
        add(f"gps lead {grp}", pv.Spline(route + off, 600).tube(radius=od / 2, n_sides=12), mm.COLORS[grp])

    targets = {
        "gps_board": on_board(4.0, 0.0, pcb_t + 2.4), "antenna": np.array([a_c[0], a_c[1], L.cap_patch_z[0] + mm.val(dims, "neo7m", "antenna_h")]),
        "hole": hole_top, "lead_mid": centre_line[330], "plug": veh(jx + x3, jy, plug.info["housing_top_z"]),
        "carrier": veh(cx + 20.0, cy + 30.0, 1.0), "battery": np.array([L.battery_x[0], L.battery_y[0], sum(L.battery_z) / 2])
        if L.has_battery else None, "cap": np.array([-40.0, 35.0, L.z_cap_top]),
        "tube": np.array([L.body_r * math.cos(math.radians(th1 + 4)), L.body_r * math.sin(math.radians(th1 + 4)), 400.0]),
    }
    numbers = {
        "board_file": board.name, "board_sha256": sha256(board), "board_centre_kicad": [cx, cy],
        "map": "x_v = Xk - cx, y_v = cy - Yk, z_v = lp_board_top_z + height above the board top",
        "lp_board_top_z": z_top, "z_cap_top": L.z_cap_top, "z_cap_plate_bottom": L.z_cap_plate_bottom,
        "cap_hole_gps_lead": {"x": hx, "y": hy, "d": hd, "label": "F cad/layout.py cap_holes"},
        "gps_board_centre": [round(v, 3) for v in b_c], "gps_antenna_centre": [round(v, 3) for v in a_c],
        "gps_board_yaw_deg": {"v": GPS_BOARD_YAW_DEG, "label": "A"},
        "j7_plug_lead_end_vehicle": [round(float(v), 3) for v in end],
        "lead_route_points": [[round(float(c), 3) for c in p] for p in route],
        "lead_len_drawn_mm": {"v": round(spline_len(centre_line), 1), "label": "E along the illustrative route (A)"},
        "lead_len_vehicle_cad_mm": {"v": round(float(L.gps_lead_len), 1), "label": "F cad/layout.py gps_lead_len (route to the carrier box top centre)"},
        "cut_wedge_deg": list(CUT_WEDGE_DEG), "camera": CAMERA,
        "assumed": ["NEO-7M board / antenna dimensions tagged A in spec/module_dimensions.json (and that the SZH-NT07 is the generic 35 x 25 mm board)",
                    "receiver board orientation on the cap (header toward the hole)", "antenna lead route",
                    "GPS lead route inside the avionics bay and its slack loop; no clearance is checked along it",
                    "wire colours", "the lead drawn on the J7 plug model (rise, bend, run)"],
        "not_shown": ["intake / propulsion modules below the avionics bay", "servo, motor and logic-power leads", "joint screws"],
    }
    return {"meshes": meshes, "targets": targets, "numbers": numbers}


def render(scene: dict, png: Path, board_label: str) -> None:
    pl = pv.Plotter(off_screen=True, window_size=(WIDTH, HEIGHT), lighting="three lights")
    pl.set_background(BACKGROUND[0], top=BACKGROUND[1])
    for m in scene["meshes"]:
        pl.add_mesh(m["poly"], color=m["color"], smooth_shading=False, ambient=0.25, diffuse=0.75, specular=0.08)
        if m["edges"]:
            e = m["poly"].extract_feature_edges(feature_angle=35.0, boundary_edges=True, non_manifold_edges=False, manifold_edges=False)
            if e.n_cells:
                pl.add_mesh(e, color=(0.18, 0.2, 0.24), line_width=1.2)
    az, el = math.radians(CAMERA["azimuth_deg"]), math.radians(CAMERA["elevation_deg"])
    f = np.array(CAMERA["focus"], dtype=float)
    pos = f + CAMERA["distance"] * np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    pl.camera.position, pl.camera.focal_point, pl.camera.up = tuple(pos), tuple(f), (0.0, 0.0, 1.0)
    pl.camera.view_angle = CAMERA["view_angle"]
    pl.enable_anti_aliasing("ssaa")
    pl.show(auto_close=False)
    img = pl.screenshot(None, return_img=True)
    ren = pl.renderer

    def px(p: np.ndarray) -> tuple[float, float]:
        ren.SetWorldPoint(float(p[0]), float(p[1]), float(p[2]), 1.0)
        ren.WorldToDisplay()
        x, y, _ = ren.GetDisplayPoint()
        sx, sy = img.shape[1] / pl.window_size[0], img.shape[0] / pl.window_size[1]
        return x * sx, img.shape[0] - y * sy

    t = {k: px(v) for k, v in scene["targets"].items() if v is not None}
    pl.close()
    n = scene["numbers"]
    labels = [
        ("gps_board", "NEO-7M 수신 기판 (윗뚜껑 위, 양면테이프)\n35 x 25 mm 일반형 기판으로 그림", True),
        ("antenna", "세라믹 패치 안테나 + 구리 접지판\n안테나 선 경로는 그림용", True),
        ("hole", f"윗뚜껑 구멍 'GPS lead' 지름 {n['cap_hole_gps_lead']['d']:g} mm\n(차체 CAD 값)", False),
        ("lead_mid", f"GPS 리드선 4가닥, 그린 길이 약 {n['lead_len_drawn_mm']['v']:.0f} mm\n경로와 여유 고리는 그림용", True),
        ("plug", "캐리어 기판 J7 (JST-GH 4핀)에\n플러그를 꽂은 모습", False),
        ("carrier", f"캐리어 기판: {board_label}", False),
        ("battery", "배터리 (자리 표시 상자)", False),
        ("cap", "윗뚜껑 (top cap)", False),
        ("tube", "항전부 몸통 - 안이 보이도록 일부를 잘라 낸 그림", False),
    ]
    from PIL import Image
    pil = Image.fromarray(img[:, :, :3])
    if pil.size != (WIDTH, HEIGHT):
        scale = (WIDTH / pil.size[0], HEIGHT / pil.size[1])
        pil = pil.resize((WIDTH, HEIGHT), Image.LANCZOS)
        t = {k: (v[0] * scale[0], v[1] * scale[1]) for k, v in t.items()}
    left, right = [], []
    for key, text, assumed in labels:
        if key in t:
            (left if t[key][0] < WIDTH * 0.47 else right).append((key, text, assumed))
    callouts = []
    for side, items in (("L", left), ("R", right)):
        y_next = 40.0
        for key, text, assumed in sorted(items, key=lambda it: t[it[0]][1]):
            y = max(y_next, t[key][1] - 40.0)
            x = 36.0 if side == "L" else WIDTH - 36.0 - 590.0
            callouts.append(slides_ko.Callout(text if not assumed else text + f" ({slides_ko.ASSUMED_WORD})", t[key], (x, y), assumed))
            y_next = y + 92.0
    banner = (f"그림 d2: NEO-7M을 차체 윗뚜껑 위에 두고 리드선을 캐리어 기판 J7까지 내린 모습입니다. "
              f"차체 형상은 cad/layout.py (vtol), 기판은 {board_label}. "
              f"GPS 모듈 치수와 방향, 리드선 경로는 가정 값이며 간섭 검사는 하지 않았습니다.")
    slides_ko.annotate(pil, callouts, out=png, size=24, banner=banner, banner_assumed=True)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--board", type=Path, default=ROOT / "kicad" / "render_completed.kicad_pcb")
    ap.add_argument("--out", type=Path, default=ROOT / "out" / "render" / "d2_neo7m_top_cap.png")
    ap.add_argument("--board-label", default="rev 3 기판 (render_completed.kicad_pcb)")
    args = ap.parse_args(argv)
    if not args.board.is_file():
        raise FileNotFoundError(f"{args.board}: run render_rev3.py --prepare-only first")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="d2_scene_") as tmp:
        scene = build_scene(args.board.resolve(), Path(tmp))
    render(scene, args.out, args.board_label)
    report = {"schema": "vtol_carrier.render_d2/1", "picture": args.out.name, "board_label": args.board_label,
              "truth": "illustration: positions F (vehicle CAD) / A (module, lead); nothing here is a fit check",
              **scene["numbers"]}
    args.out.with_suffix(".json").write_text(json.dumps(report, ensure_ascii=False, indent=1) + "\n", encoding="utf-8", newline="\n")
    print(f"wrote {args.out}")
    print(f"lead drawn {report['lead_len_drawn_mm']['v']} mm (E, illustrative route); vehicle CAD lead {report['lead_len_vehicle_cad_mm']['v']} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
