"""v0.8.1 GNS-board placement drawing (Korean labels, for the build guide / presentation).

(a) a true plan section of the twin's exported STLs (cad/exports/cadquery/stl: avionics_module with the GNS bosses, the tray)
at the top bosses' height, with the Layout boxes (battery, NAV, ESC projected, GNS board) and the projected GPS / glink lead
polylines; (b) a side projection (x-z) of the bay from the Layout boxes, the bosses, the GPS pod and the lead polylines.
Every number comes from cad/layout.py (Layout) and the STLs - run build_rocket.py first.

Usage: python cad/cadquery/make_gns_figure.py [OUT.png ...]   (default cad/exports/gns_board_layout.png)"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "cad"))
from layout import build  # noqa: E402

for f in (r"C:\Windows\Fonts\malgun.ttf", r"C:\Windows\Fonts\malgunbd.ttf"):
    if os.path.exists(f):
        font_manager.fontManager.addfont(f)
plt.rcParams.update({"font.family": ["Malgun Gothic", "DejaVu Sans"], "axes.unicode_minus": False, "font.size": 10})

L = build()
STL = os.path.join(ROOT, "cad", "exports", "cadquery", "stl")
C = {"tube": "#3a3a3a", "tray": "#8a8a8a", "battery": "#f29a1a", "esc": "#d93333", "nav": "#33a640", "misc": "#9966cc",
     "com": "#1ab2c7", "radio": "#8cd933", "fcu": "#0d735a", "gns": "#268c8c", "boss": "#111111", "gps": "#1a4dbf",
     "glink": "#c23a8a"}


def section(path, z):
    m = trimesh.load(path, force="mesh")
    s = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    if s is None:
        return []
    p2, T = s.to_2D()
    out = []
    for poly in p2.polygons_full:
        def to3(coords):
            a = np.array(coords)
            return (T @ np.c_[a, np.zeros(len(a)), np.ones(len(a))].T).T[:, :2]
        out.append((to3(poly.exterior.coords), [to3(h.coords) for h in poly.interiors]))
    return out


def rect_xy(ax, box, color, label=None, ls="-", alpha=0.85, fill=True, lw=1.0):
    (x0, x1), (y0, y1) = box[0], box[1]
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor=color if fill else "none", edgecolor="k" if fill else color,
                           alpha=alpha, lw=lw, ls=ls, label=label))


def rect_xz(ax, box, color, label=None, ls="-", alpha=0.85, fill=True, lw=1.0):
    (x0, x1), _, (z0, z1) = box
    ax.add_patch(Rectangle((x0, z0), x1 - x0, z1 - z0, facecolor=color if fill else "none", edgecolor="k" if fill else color,
                           alpha=alpha, lw=lw, ls=ls, label=label))


fig = plt.figure(figsize=(20, 11.25), dpi=120)
fig.suptitle("v0.8.1 GNS 보드(ATmega128A 4번째: GPS 수신 + 위치 유지) 배치 - CadQuery 트윈, 치수 mm", fontsize=15)

# ---------------- (a) plan section at the top bosses ----------------
ax = fig.add_axes([0.04, 0.06, 0.43, 0.84])
z_cut = L.gns_hole_vz[0][1]
for name, col, lab in (("avionics_module", C["tube"], "항공전자 모듈 (단면, 보스 포함)"), ("avionics_tray", C["tray"], "트레이 (단면)")):
    for pts, holes in section(os.path.join(STL, f"{name}.stl"), z_cut):
        ax.add_patch(Polygon(pts, closed=True, facecolor=col, edgecolor="k", lw=0.6, label=lab))
        lab = None
        for h in holes:
            ax.add_patch(Polygon(h, closed=True, facecolor="white", edgecolor="k", lw=0.4))
rect_xy(ax, (L.battery_x, L.battery_y), C["battery"], "배터리 6S 2200 (349 g)")
rect_xy(ax, (L.nav_x, L.nav_y), C["nav"], "NAV 보드 (센서·자세)")
rect_xy(ax, (L.esc_x, L.esc_y), C["esc"], "ESC (아래쪽, 투영)", ls="--", fill=False, lw=1.6)
rect_xy(ax, (L.gns_x, L.gns_y), C["gns"], f"GNS 보드 50 x 50 x 15 ({L.gns_mass_g:g} g, 가정)")
e = L.gps_lead_polyline[L.gps_lead_bay_start - 1]
gx, gy, gz = zip(*L.gps_lead_polyline[L.gps_lead_bay_start - 1:])
ax.plot(gx, gy, color=C["gps"], lw=2.5, label="GPS 선 (입구 구멍 -> GNS, 투영)")
lx, ly, lz = zip(*L.glink_lead_polyline)
ax.plot(lx, ly, color=C["glink"], lw=2.0, ls="--", label="glink 선 GNS -> NAV (트레이 위, 투영)")
ax.add_patch(Circle((e[0], e[1]), 2.0, facecolor="yellow", edgecolor="k", zorder=5))
ax.annotate(f"GPS 선 입구 구멍 Ø{L.gps_entry_d:g}\n(θ {L.gps_entry_theta:g}°, z {L.gps_entry_z:.1f})", (e[0], e[1]), (-73, -45),
            arrowprops=dict(arrowstyle="->"), fontsize=9)
x, y, _ = L.gns_xyz(L.gns_u_back + 1.5, -L.gns_pattern / 2, 0.0)
ax.annotate("인쇄 보스 4개 (45° 마름모, M3 탭 구멍)\n보드 뒷면이 보스 끝에 닿음", (x - 2, y + 2), (-73, 47),
            arrowprops=dict(arrowstyle="->"), fontsize=9)
ax.annotate(f"배터리까지 {L.gns_clearance['battery']:.0f} mm", ((L.gns_x[1] + L.battery_x[0]) / 2, 0), (-30, 40),
            arrowprops=dict(arrowstyle="->"), fontsize=9, ha="center")
ax.annotate("", (L.battery_x[0], 0), (L.gns_x[1], 0), arrowprops=dict(arrowstyle="<->"))
ax.annotate(f"ESC까지 {L.gns_distance_mm['ESC (box)']:.0f} mm (목표 100 mm;\n베이 안에서는 이 벽 위치가 가장 멂)", (L.esc_x[0], -18),
            (5, -66), arrowprops=dict(arrowstyle="->"), fontsize=9)
ax.set_xlim(-75, 75)
ax.set_ylim(-70, 70)
ax.set_aspect("equal")
ax.grid(alpha=0.3)
ax.set_xlabel("x [mm] (+x = 베인 1 방향)")
ax.set_ylabel("y [mm]")
ax.set_title(f"(가) 평면 단면 z = {z_cut:.1f} mm (위쪽 보스 높이), 위에서 본 모습")
ax.legend(loc="upper right", fontsize=8.0, framealpha=0.95)

# ---------------- (b) side projection ----------------
ax = fig.add_axes([0.47, 0.06, 0.37, 0.84])
r = L.body_r
ax.add_patch(Rectangle((-r, L.z_avionics_bottom), L.body_wall, L.z_avionics_spigot_bottom - L.z_avionics_bottom, facecolor=C["tube"],
                       edgecolor="k", lw=0.5, label="튜브 벽 (x-z 투영)"))
ax.add_patch(Rectangle((r - L.body_wall, L.z_avionics_bottom), L.body_wall, L.z_avionics_spigot_bottom - L.z_avionics_bottom,
                       facecolor=C["tube"], edgecolor="k", lw=0.5))
for xw in (-r, r - L.body_wall):
    ax.add_patch(Rectangle((xw, L.z_payload_bottom), L.body_wall, L.gps_z_lid_top - L.z_payload_bottom, facecolor="#555555",
                           edgecolor="k", lw=0.5))
ax.add_patch(Rectangle((-L.body_id / 2, L.z_payload_bulkhead_bottom), L.body_id, L.z_payload_bulkhead_top - L.z_payload_bulkhead_bottom,
                       facecolor="#555555", edgecolor="k", lw=0.5, label="페이로드 격벽 / 흡입 격벽"))
ax.add_patch(Rectangle((-L.body_id / 2, L.z_intake_bulkhead_bottom), L.body_id, L.z_intake_bulkhead_top - L.z_intake_bulkhead_bottom,
                       facecolor="#555555", edgecolor="k", lw=0.5))
u_far = L.gps_u_out * math.cos(math.radians(L.gps_theta - 180.0))
ax.add_patch(Rectangle((-u_far, L.gps_z_foot), u_far - r, L.gps_z_lid_top - L.gps_z_foot, facecolor="#777777", edgecolor="k",
                       alpha=0.35, lw=0.6, label="GPS 포드 (θ 192°, 투영)"))
ax.axhline(L.z_avionics_spigot_bottom, color="0.5", lw=0.8, ls=":")
ax.text(r + 2, L.z_avionics_spigot_bottom + 2, "항공전자/페이로드\n결합부 (나사)", fontsize=8, va="bottom")
rect_xz(ax, ((L.tray_x - L.tray_t / 2, L.tray_x + L.tray_t / 2), None, (L.z_tray_bottom, L.z_tray_top)), C["tray"], "트레이")
rect_xz(ax, (L.battery_x, None, L.battery_z), C["battery"], "배터리")
rect_xz(ax, (L.esc_x, None, L.esc_z), C["esc"], "ESC")
rect_xz(ax, (L.nav_x, None, L.nav_z), C["nav"], "NAV 보드")
rect_xz(ax, (L.misc_x, None, L.misc_z), C["misc"], "기타 (UBEC 등)")
rect_xz(ax, (L.com_board_x, None, L.com_board_z), C["com"], "COM 보드")
rect_xz(ax, (L.radio_log_x, None, L.radio_log_z), C["radio"], "LoRa·로거 상자")
rect_xz(ax, (L.fcu_x, None, L.fcu_z), C["fcu"], "FCU 보드")
rect_xz(ax, (L.gns_x, None, L.gns_z), C["gns"], "GNS 보드 (새로 추가)", lw=1.6)
for bb in L.gns_boss_boxes:
    ax.add_patch(Rectangle((bb[0][0], bb[2][0]), bb[0][1] - bb[0][0], bb[2][1] - bb[2][0], facecolor=C["boss"], edgecolor="k", lw=0.4))
ax.plot([], [], color=C["boss"], lw=6, label="보스 4개 (튜브 벽과 한 몸으로 출력)")
px, py, pz = zip(*L.gps_lead_polyline)
ax.plot(px, pz, color=C["gps"], lw=2.5, label=("GPS 선 (포드 -> 선 통로 -> 커넥터 칸\n -> 입구 구멍 -> GNS, "
                                              f"{L.gps_lead_route_len:.0f} mm)"))
ax.plot(lx, lz, color=C["glink"], lw=2.0, ls="--", label=f"glink 선 GNS -> NAV ({L.glink_route_len:.0f} mm)")
ax.add_patch(Circle((e[0], e[2]), 2.5, facecolor="yellow", edgecolor="k", zorder=5))
ant = L.gps_antenna_xyz
ax.plot([ant[0]], [ant[2]], marker="+", color="gold", ms=14, mew=2.5)
ax.annotate(f"GPS 안테나 위상 중심 (가정)\nGNS까지 {L.gns_distance_mm['GPS antenna phase centre']:.0f} mm", (ant[0], ant[2]),
            (-118, ant[2] - 75), arrowprops=dict(arrowstyle="->"), fontsize=9)
ax.annotate("GNS 윗면이 GPS 입구 구멍 바로 아래\n(베이 안 GPS 선 13 mm)", (e[0], e[2]), (-118, 415), arrowprops=dict(arrowstyle="->"),
            fontsize=9)
ax.set_xlim(-122, 100)
ax.set_ylim(L.z_avionics_bottom - 5, 715)
ax.set_aspect("equal")
ax.grid(alpha=0.3)
ax.set_xlabel("x [mm]")
ax.set_ylabel("높이 z [mm] (분사구 출구 = 0)")
ax.set_title("(나) 옆에서 본 투영도 (-y 쪽에서), 항공전자 베이와 페이로드 모듈 아래쪽")
ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=9.0, framealpha=0.95)
for out in (sys.argv[1:] or [os.path.join(ROOT, "cad", "exports", "gns_board_layout.png")]):
    fig.savefig(out, dpi=120)
    print("wrote", out)
