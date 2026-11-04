"""Tests for the pure parts of the Blender pipeline (no Blender needed): shot list from events, time remapping /
slow motion, vane transform sign vs tools/sim_video.py, jet / windsock / person models, cameras, HUD text, driver
helpers.  Run:  python -m pytest -q tools/test_blender_render.py
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from render import hud as HUD  # noqa: E402
from render import plan as PL  # noqa: E402
from render import shots as SH  # noqa: E402
from render.logio import Log, quat_to_matrix  # noqa: E402

FPS = 30.0


# --------------------------------------------------------------------------- #
# a synthetic VTOL log (kinematic, deterministic)
# --------------------------------------------------------------------------- #
def synth_log(push: bool = False, gust: bool = True, t_end: float = 30.0, dt: float = 0.01) -> Log:
    t = np.arange(0.0, t_end, dt)
    n = t.size
    t_on, t_lift, t_land, t_td = 1.0, 3.0, 22.0, 26.0
    h = np.interp(t, [0, t_lift, t_lift + 3, t_land, t_td, t_end], [0, 0, 1.5, 1.5, 0, 0])
    pos = np.zeros((n, 3))
    pos[:, 2] = h
    tilt = np.zeros(n)
    q = np.tile([1.0, 0.0, 0.0, 0.0], (n, 1))
    wind = np.zeros((n, 3))
    gust_a = np.zeros(n, bool)
    if gust:
        g = (t >= 10.0) & (t < 11.0)
        gust_a[g] = True
        wind[g] = [5.0, 0.0, 0.0]
    ang = np.radians(3.0) * np.exp(-((t - 10.6) / 0.6) ** 2)
    q[:, 0], q[:, 2] = np.cos(ang / 2), np.sin(ang / 2)
    tilt = np.degrees(ang)
    state = np.where((t >= t_on) & (t < t_td), 2, 0)
    phase = np.select([t < t_on, t < t_lift, t < t_lift + 3, t < t_land, t < t_td], [0, 1, 2, 3, 4], 5)
    thrust = np.where(state == 2, 29.0, 0.0)
    delta = np.zeros((n, 4))
    delta[:, 1] = np.radians(8.0) * np.exp(-((t - 10.6) / 0.5) ** 2)
    pf = pp = pa = None
    if push:
        pa = ((t >= 14.0) & (t < 14.6)) | ((t >= 18.0) & (t < 18.5))
        pf = np.zeros((n, 3))
        pf[pa] = [0.0, 12.0, 0.0]
        pp = pos + np.array([0.0, -0.06, 0.45])
    return Log(t=t, pos=pos, q=q, vel=np.gradient(pos, t, axis=0), delta=delta, throttle=thrust / 32.0, thrust=thrust,
               state=state, phase=phase, h_target=np.where(phase >= 2, 1.5, np.nan), tilt_deg=tilt, wind=wind,
               gust_active=gust_a, ground_z=-0.11, event_t=np.zeros(0), event_text=[], cmd_t=np.array([1.0, 21.9]),
               cmd_name=["ON", "OFF"], meta={"scenario": "synthetic", "t_max_n": 30.3, "z_cg_m": 0.376},
               sha256="0" * 64, source="synthetic", push_force=pf, push_point=pp, push_active=pa)


# --------------------------------------------------------------------------- #
# events and shots
# --------------------------------------------------------------------------- #
def test_events_from_log():
    ev = SH.detect_events(synth_log(push=True))
    kinds = [e.kind for e in ev]
    for k in ("start", "motor_on", "liftoff", "hover", "gust", "push", "land", "touchdown", "end"):
        assert k in kinds, k
    lift = SH.first(ev, "liftoff")
    assert abs(lift.t - 3.0) < 0.2
    pushes = [e for e in ev if e.kind == "push"]
    assert len(pushes) == 2 and abs(pushes[0].value - 12.0) < 1e-9
    assert [e.t for e in ev] == sorted(e.t for e in ev)


def test_auto_shots_cover_story_without_overlap():
    log = synth_log(push=True)
    ev = SH.detect_events(log)
    shots = SH.auto_shots(ev, float(log.t[0]), float(log.t[-1]), indoor=True)
    assert shots, "no shots"
    for a, b in zip(shots, shots[1:]):
        assert a.t1 <= b.t0 + 1e-9, "shots overlap / time runs backwards"
        if abs(a.t1 - b.t0) < 1e-6:
            assert a.cam != b.cam, "contiguous shots share a rig"
    notes = " ".join(s.note for s in shots)
    assert "liftoff" in notes and "landing" in notes and "push#1" in notes and "push#2" in notes
    for s in shots:
        assert s.cam in SH.CAM_KINDS and s.t1 - s.t0 >= 1.2 - 1e-9
        for w in s.slow:
            assert s.t0 - 1e-9 <= w.a < w.b <= s.t1 + 1e-9 and 0 < w.factor < 1
    # every disturbance has its shot with a slow-motion window starting before it
    for e in ev:
        if e.kind in ("push", "gust"):
            assert any(s.t0 <= e.t <= s.t1 and any(w.a <= e.t + 0.2 for w in s.slow) for s in shots), e


def test_shots_json_roundtrip_and_validation():
    s = [SH.Shot(1.0, 4.0, "orbit", [SH.Slow(2.0, 3.0, 0.25)], "x", {"radius": 3.0})]
    back = SH.shots_from_json([x.to_json() for x in s])
    assert back[0].cam == "orbit" and back[0].slow[0].factor == 0.25 and back[0].params["radius"] == 3.0
    with pytest.raises(ValueError):
        SH.shots_from_json([{"t0": 0, "t1": 1, "cam": "drone"}])


# --------------------------------------------------------------------------- #
# time remapping
# --------------------------------------------------------------------------- #
def test_time_remap_real_time_and_slow_motion():
    plain = SH.shot_times(SH.Shot(0.0, 2.0, "wide"), FPS)
    assert plain.size == 60 and np.allclose(np.diff(plain), 1 / FPS)
    slow = SH.Shot(0.0, 4.0, "spectator", [SH.Slow(1.0, 3.0, 0.25)])
    ts = SH.shot_times(slow, FPS)
    d = np.diff(ts)
    inside = (ts[:-1] > 1.3) & (ts[:-1] < 2.7)
    assert np.allclose(d[inside], 0.25 / FPS)                        # 4x more frames inside the window
    assert np.allclose(d[ts[:-1] < 1.0], 1 / FPS)
    assert np.max(np.abs(np.diff(d))) < 0.25 / FPS                    # eased: no speed jump
    assert ts[0] == 0.0 and ts[-1] < 4.0 and np.all(d > 0)
    # video duration: 2 s real time + 2 s at ~0.25x (ramps make it a bit shorter than 8 s)
    dur = ts.size / FPS
    assert 7.0 < dur < 10.0


def test_timeline_concatenates_shots():
    shots = [SH.Shot(0.0, 1.0, "wide"), SH.Shot(5.0, 6.0, "orbit")]
    t, seg, sp = SH.timeline(shots, FPS)
    assert t.size == 60 and seg[0] == 0 and seg[-1] == 1 and np.all(sp == 1.0)
    assert t[30] == pytest.approx(5.0)


# --------------------------------------------------------------------------- #
# vanes: same transform as tools/sim_video.py (SPEC 8.2)
# --------------------------------------------------------------------------- #
def test_vane_transform_matches_sim_video():
    sv = pytest.importorskip("tools.sim_video")
    for theta in (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi, 0.3):
        for delta in (-0.3, -0.05, 0.0, 0.12, 0.35):
            for th_stl in (0.0, 0.7):
                A = PL.vane_world_matrix(theta, delta, 0.037, th_stl)
                B = sv.vane_local_matrix(theta, delta, 0.037, th_stl)
                assert np.allclose(A, B, atol=1e-12)


def test_vane_positive_delta_moves_trailing_edge_to_minus_e_theta():
    hz = 0.037
    for i in range(4):
        theta = i * 0.5 * math.pi
        e_t = np.array([-math.sin(theta), math.cos(theta), 0.0])
        # a trailing-edge point of the STL (lying along +x, theta_stl = 0) below the hinge
        p = np.array([0.03, 0.0, hz - 0.02, 1.0])
        p0 = PL.vane_world_matrix(theta, 0.0, hz) @ p
        p1 = PL.vane_world_matrix(theta, math.radians(10.0), hz) @ p
        assert float((p1 - p0)[:3] @ e_t) < -1e-4                    # trailing edge -> -e_theta: force +e_theta
        # Blender hinge Euler XYZ (-delta, 0, theta) = Rz(theta) Rx(-delta)
        assert PL.vane_euler(theta, 0.2) == (-0.2, 0.0, theta)


# --------------------------------------------------------------------------- #
# models
# --------------------------------------------------------------------------- #
def test_jet_state_numbers():
    pos = np.array([[0.0, 0.0, 1.5], [0.0, 0.0, 0.0], [0.0, 0.0, 30.0]])
    R = quat_to_matrix(np.tile([1.0, 0, 0, 0], (3, 1)))
    js = PL.jet_state(pos, R, np.array([29.4, 29.4, 29.4]), -0.11)
    A = math.pi * 0.045 ** 2 * (1 - 0.45 ** 2)
    v0 = math.sqrt(29.4 / (1.2 * A))
    assert js["v0"][0] == pytest.approx(v0)
    assert js["hit"][0] and js["x_ground"][0] == pytest.approx(1.61)
    assert js["u_ground"][0] == pytest.approx(v0 * 6.2 * 0.09 / 1.61)  # past the potential core: 1/x decay
    assert js["u_ground"][1] == pytest.approx(v0)                      # 0.11 m: inside the core
    assert not js["hit"][2] and js["dust"][2] == 0.0                   # too high: nothing reaches the ground
    assert js["dust"][0] == pytest.approx(1.0) and js["impact"][0][2] == pytest.approx(-0.11)


def test_windsock_calm_hangs_and_wind_extends():
    t = np.arange(0, 5, 0.02)
    yaw, droop = PL.windsock(t, np.zeros((t.size, 3)))
    assert np.degrees(droop[-1, 0]) > 70
    w = np.tile([0.0, 8.0, 0.0], (t.size, 1))
    yaw, droop = PL.windsock(t, w)
    assert yaw[-1] == pytest.approx(math.pi / 2, abs=1e-3)              # points where the air goes (+y)
    assert abs(np.degrees(droop[-1, 0])) < 9


def test_person_track_pole_reaches_contact_and_keeps_distance():
    pytest.importorskip("tools.sim_video")
    log = synth_log(push=True)
    tg = np.arange(log.t[0], log.t[-1], 0.02)
    from render.plan import hold, interp, slerp_resample
    pos = interp(log.t, log.pos, tg)
    R = quat_to_matrix(slerp_resample(log.t, log.q, tg))
    pa = hold(log.t, log.push_active, tg)
    pf = interp(log.t, log.push_force, tg)
    pp = interp(log.t, log.push_point, tg)
    pt = PL.person_track(tg, pos, R, -0.11, pa, pf, pp)
    # the pole is rigid and always in both hands
    assert np.allclose(np.linalg.norm(pt.pad - pt.butt, axis=1), PL.POLE_LEN)
    assert np.allclose(np.linalg.norm(pt.grip_l - pt.grip_r, axis=1), PL.POLE_GRIP_M[1] - PL.POLE_GRIP_M[0])
    k = np.flatnonzero(pa)[5]
    C = PL.visual_contact(pp[k], pf[k], pos[k], R[k])
    assert pt.push_w[k] == pytest.approx(1.0)
    assert np.linalg.norm(pt.pad[k] - C) == pytest.approx(PL.POLE_PAD_R, abs=1e-6)   # foam touches the contact
    # held end near the nominal hands of the figure (sim_video FIGURE_HANDS) while pushing
    Rz = PL._rot2(float(pt.yaw[k]))
    assert np.linalg.norm(pt.hold[k] - (pt.feet[k] + Rz @ PL.FIGURE_HANDS)) < 0.3
    # carry long before the first push: the pole upright at the ready, the pad high and clear of the vehicle
    k0 = 10
    assert pt.push_w[k0] == 0.0 and pt.pad[k0, 2] > -0.11 + 1.9
    assert float(np.hypot(*(pt.pad[k0, :2] - pos[k0, :2]))) > 1.0
    cg = pos + R[:, :, 2] * PL.Z_CG_DEFAULT
    dmin = float(np.min(np.hypot(*(pt.feet[:, :2] - cg[:, :2]).T)))
    assert dmin > 1.4                                                   # outside the vehicle's reach
    assert pt.feet[k, 1] < pos[k, 1]                                    # pushes toward +y: stands on the -y side
    # the weight ramps smoothly into the push (no pop)
    a = np.flatnonzero(pa)[0]
    assert 0.0 < pt.push_w[a - 5] < 1.0 and np.max(np.abs(np.diff(pt.push_w))) < 0.1


# --------------------------------------------------------------------------- #
# cameras
# --------------------------------------------------------------------------- #
def test_look_quat_projects_target_to_centre():
    pos, tgt = np.array([3.0, -4.0, 1.6]), np.array([0.2, 0.1, 1.1])
    x, y, depth = PL.project(tgt, pos, tgt, 0.0, 50.0, 1920, 1080)
    assert x == pytest.approx(960.0) and y == pytest.approx(540.0) and depth > 0
    # a point above the target is above the image centre (y down)
    x2, y2, _ = PL.project(tgt + [0, 0, 0.2], pos, tgt, 0.0, 50.0, 1920, 1080)
    assert y2 < 540.0


def test_build_plan_arrays_and_cuts():
    log = synth_log(push=True)
    ev = SH.detect_events(log)
    shots = SH.auto_shots(ev, float(log.t[0]), float(log.t[-1]), indoor=True)
    plan = PL.build_plan(log, shots, FPS, PL.SceneCfg(indoor=True))
    n = plan["meta"]["frames"]
    for k in ("pos", "q", "delta", "cam_pos", "cam_quat", "cam_lens", "jet_impact", "sock_droop", "person_pad", "person_grip_r"):
        assert plan[k].shape[0] == n, k
    assert np.all(np.isfinite(plan["cam_quat"])) and np.all(plan["cam_lens"] >= 18)
    assert int(np.sum(plan["cut_after"])) == len(shots)
    assert plan["meta"]["scene"] == "indoor" and plan["meta"]["person"]
    # the vehicle stays in front of every camera
    for k in range(0, n, 7):
        cg = plan["pos"][k] + quat_to_matrix(plan["q"][k])[:, 2] * 0.376
        _, _, depth = PL.project(cg, plan["cam_pos"][k], plan["cam_tgt"][k], 0.0, float(plan["cam_lens"][k]), 1920, 1080)
        assert depth > 0.2


def test_turntable_plan():
    p = PL.turntable_plan(FPS, 9.0)
    assert p["meta"]["frames"] == 270 and p["meta"]["scene"] == "studio"
    assert np.all(np.diff(p["tt_angle"]) > 0)
    assert np.max(np.abs(p["delta"])) <= math.radians(15.0) + 1e-9 and np.max(np.abs(p["delta"])) > math.radians(5)
    assert np.all(p["spin"] == 0)


# --------------------------------------------------------------------------- #
# HUD text + driver helpers
# --------------------------------------------------------------------------- #
def test_hud_values_korean():
    fi = HUD.FrameInfo(t=12.34, speed=0.25, phase=3, state=2, alt=1.523, h_target=1.5, tilt=2.44, throttle=0.913,
                       wind=np.array([3.0, 4.0, 0.0]), push_active=True, push_n=12.2, cam_right=np.array([1.0, 0, 0]),
                       cam_fwd=np.array([0.0, 1.0, 0.0]))
    v = HUD.frame_values(fi)
    assert v["phase"] == "호버링" and v["alt"] == "1.52 m" and v["target"] == "목표 1.5 m"
    assert v["tilt"] == "2.4°" and v["thr"] == "91 %" and v["wind"] == "5.0 m/s" and v["push"] == "12 N"
    assert v["slow"] == "슬로모션  0.25×" and v["time"] == "T+ 12.3 s"
    fi.speed, fi.slow_factor = 0.61, 0.25                             # ramping into the window: badge shows the window
    assert HUD.frame_values(fi)["slow"] == "슬로모션  0.25×"
    fi.speed, fi.slow_factor = 0.25, float("nan")
    fi.state = 5
    assert HUD.frame_values(fi)["phase"] == "비상정지"
    fi.speed, fi.push_active, fi.wind = 1.0, False, np.zeros(3)
    v = HUD.frame_values(fi)
    assert v["slow"] == "" and v["push"] == "—" and v["wind"] == "무풍"
    # wind toward +y with the camera looking along +y: away from the camera = screen up
    fi.wind = np.array([0.0, 3.0, 0.0])
    assert HUD.wind_screen_angle(fi) == pytest.approx(math.pi / 2)
    assert HUD.TAG_CLEAN == "시뮬레이션 (CG)" and HUD.TAG_PRO == "시뮬레이션 - 실제 비행 영상 아님"


def test_hud_events_and_summary():
    assert HUD.event_text_ko("cmd", "UP") == "지상 명령  UP (고도 +)"
    assert HUD.event_text_ko("gust", value=5.04) == "돌풍  5.0 m/s"
    assert HUD.event_text_ko("push", value=12.0) == "밀기  12 N"
    s = dict(HUD.summary_numbers(synth_log(push=True)))
    assert s["최고 고도"] == "1.50 m" and s["밀기"] == "2회 · 최대 12 N" and s["최대 돌풍"] == "5.0 m/s"


def test_hud_overlay_renders():
    pytest.importorskip("PIL")
    from PIL import Image
    if not os.path.exists(HUD.FONT_REG):
        pytest.skip("Malgun Gothic not installed")
    log = synth_log(push=True)
    tele = HUD.telemetry_from_log(log, SH.detect_events(log), person=True)
    fi = HUD.FrameInfo(t=14.2, speed=0.25, phase=3, state=2, alt=1.5, h_target=1.5, tilt=3.0, throttle=0.9,
                       wind=np.array([2.0, 0.0, 0.0]), push_active=True, push_n=12.0, cam_right=np.array([1.0, 0, 0]),
                       cam_fwd=np.array([0.0, 1.0, 0.0]), frame=200)
    img = Image.new("RGB", (960, 540), (90, 110, 80))
    out = HUD.overlay_pro(img, fi, tele)
    assert out.size == (960, 540) and np.asarray(out).std() > 5
    clean = np.asarray(HUD.tag_clean(img)).astype(int)
    changed = np.any(clean != np.asarray(img).astype(int), axis=2)
    ys, xs = np.nonzero(changed)
    assert xs.min() > 960 * 0.6 and ys.min() > 540 * 0.85              # only the small tag, bottom right
    card = HUD.end_card(img, tele, 1.0)
    assert card.size == (960, 540)


# --------------------------------------------------------------------------- #
# art-direction round 1: physics of the ground response, provenance, camera rules, pole, QA
# --------------------------------------------------------------------------- #
def test_wall_jet_speed_matches_impinging_jet_correlation():
    pos = np.array([[0.0, 0.0, 0.24], [0.0, 0.0, 30.0]])
    R = quat_to_matrix(np.tile([1.0, 0, 0, 0], (2, 1)))
    js = PL.jet_state(pos, R, np.array([25.0, 25.0]), -0.11)
    assert js["wall_k"][0] == pytest.approx(math.sqrt(25.0 / 1.2))
    assert js["wall_k"][1] == 0.0                                      # the jet does not reach the ground
    u = PL.wall_jet_speed(0.78, js["u_ground"][0], js["half_width"][0], js["wall_k"][0])
    # Poreh et al. 1967: u_m / U_j = 1.03 (r / D)^-1.12 for an impinging round jet
    v0 = float(js["v0"][0])
    poreh = 1.03 * v0 * (0.78 / 0.09) ** -1.12
    assert abs(float(u) - poreh) / poreh < 0.15
    assert float(PL.wall_jet_speed(0.01, js["u_ground"][0], js["half_width"][0], js["wall_k"][0])) <= js["u_ground"][0] + 1e-9
    assert float(PL.wall_jet_speed(1.0, 0.0, 0.1, 0.0)) == 0.0


def test_plan_has_no_haze_and_carries_wall_jet():
    log = synth_log(push=False)
    shots = [SH.Shot(2.5, 5.0, "orbit", params={"azimuth_deg": 30.0})]
    plan = PL.build_plan(log, shots, FPS, PL.SceneCfg())
    assert np.all(plan["haze"] == 0.0)
    assert "jet_k" in plan and float(np.max(plan["jet_k"])) > 3.0
    assert not any("haze" in s for s in plan["meta"]["illustrations"])


def test_provenance_strings():
    syn = HUD.provenance({"scenario": "synthetic_push", "title": "SYNTHETIC TEST PATTERN (kinematic, not a simulation)"})
    six = HUD.provenance({"scenario": "V_nominal", "notes": "SIMULATION (sim.sim6dof 6-DOF, VTOL build v0.7.1)"})
    assert syn.kind == "synthetic" and six.kind == "sim6dof"
    for txt in (syn.tag_pro, syn.tag_clean, syn.card_note, syn.credit):
        assert "6자유도" not in txt
    assert "물리 시뮬레이션 아님" in syn.tag_pro and "물리 시뮬레이션" in syn.credit
    assert six.tag_pro == HUD.TAG_PRO and six.tag_clean == HUD.TAG_CLEAN and "6자유도" in six.credit
    assert all("6자유도" not in c for c in HUD.credit_lines(True, syn))
    log = synth_log(push=True)
    log.meta["scenario"] = "synthetic_push"
    assert "synthetic" not in HUD.default_subtitle(log) and "막대" in HUD.default_subtitle(log)
    log.meta["scenario"] = "X_unknown_case"
    assert "X_unknown_case" not in HUD.default_subtitle(log)


def test_event_callout_follows_logged_push():
    ev = (14.0, "push", "밀기  12 N", 14.6)
    assert HUD.event_alpha(ev, 13.9) == 0.0
    assert HUD.event_alpha(ev, 14.4) == pytest.approx(1.0)
    assert 0.0 < HUD.event_alpha(ev, 14.75) < 1.0
    assert HUD.event_alpha(ev, 14.6 + HUD.EVENT_TAIL_S + 0.01) == 0.0       # gone 0.3 s after the logged push
    cmd = (5.0, "cmd", "지상 명령  UP", float("nan"))
    assert HUD.event_alpha(cmd, 6.0) == pytest.approx(1.0) and HUD.event_alpha(cmd, 5.0 + HUD.EVENT_HOLD_S + 0.1) == 0.0
    assert HUD.hud_box_frac() <= 0.11


def _indoor_plan():
    log = synth_log(push=True)
    ev = SH.detect_events(log)
    shots = SH.auto_shots(ev, float(log.t[0]), float(log.t[-1]), indoor=True)
    return log, shots, PL.build_plan(log, shots, FPS, PL.SceneCfg(indoor=True))


def test_camera_rules_one_side_no_rear_view_on_stage():
    from render import hall as HL
    log, shots, plan = _indoor_plan()
    side = plan["meta"]["cam_side"]
    assert side in (1, -1) and plan["meta"]["hall"] is not None
    hj = plan["meta"]["hall"]
    hall = HL.Hall(rot=hj["rot_rad"], veh_img=np.array(hj["veh_img"]), floor_z=hj["floor_z"])
    seg = plan["seg"]
    for i, s in enumerate(shots):
        sel = np.flatnonzero(seg == i)
        cg = plan["pos"][sel] + np.array([0, 0, 0.376])
        feet = plan["person_feet"][sel]
        cam = plan["cam_pos"][sel]
        # every indoor camera is on the stage of the real room
        assert float(np.min(HL.stage_margin(hall.to_img(cam[:, :2])))) >= 0.3 - 1e-6, s.note
        if s.cam == "closeup":
            continue
        # one side of the vehicle -> person line, the same side for the whole edit
        a = feet[:, :2] - cg[:, :2]
        b = cam[:, :2] - cg[:, :2]
        cross = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
        assert np.all(np.sign(cross) == side), s.note
        # never from behind: the camera is in front of the person's facing half-plane (he faces the vehicle)
        fdir = np.column_stack([np.cos(plan["person_yaw"][sel]), np.sin(plan["person_yaw"][sel])])
        assert np.all(np.sum(fdir * (cam[:, :2] - feet[:, :2]), axis=1) > -0.05), s.note
        if s.cam == "spectator":
            # the person never more than half the frame height
            d = np.linalg.norm(feet + np.array([0, 0, 0.9]) - cam, axis=1)
            frac = 2 * np.degrees(np.arctan(0.9 / d)) / (2 * np.degrees(np.arctan(0.5 * 36 * 1080 / 1920 / plan["cam_lens"][sel])))
            assert float(np.max(frac)) <= PL.PERSON_MAX_FRAC + 0.03


def test_pole_near_horizontal_and_operator_standoff():
    log, shots, plan = _indoor_plan()
    pw = plan["person_push_w"]
    full = pw > 0.99
    assert np.any(full)
    pitch = np.degrees(np.arcsin(np.clip(plan["person_pole_dir"][full, 2], -1, 1)))
    assert float(np.max(np.abs(pitch))) <= PL.POLE_MAX_PITCH_DEG + 0.5
    cg = plan["pos"] + np.array([0, 0, 0.376])
    d = np.hypot(*(plan["person_feet"][:, :2] - cg[:, :2]).T)
    assert float(np.min(d)) >= PL.MIN_STANDOFF_M - 1e-6
    # outside the taped keep-out square around the pad
    assert np.all(np.max(np.abs(plan["person_feet"][:, :2]), axis=1) > PL.TAPE_HALF + 0.2)


def test_hall_geometry_helpers():
    from render import hall as HL
    h = HL.Hall(rot=0.3, veh_img=np.array([0.6, 0.2]), floor_z=-0.128)
    p = np.array([1.2, -0.7])
    assert np.allclose(h.to_world(h.to_img(p)), p)
    assert float(HL.stage_margin(h.to_img(np.zeros(2)))[0]) > 1.5           # the pad sits well inside the stage
    far = h.to_world(np.array([0.0, 30.0]))
    c = HL.clamp_xy(h, np.zeros(2), far)
    assert abs(float(HL.stage_margin(h.to_img(c))[0]) - 0.35) < 0.02          # pulled back to 0.35 m inside
    kind, hit = HL.proxy_hit(h, np.array([0.0, 0.0, 1.4]), np.array([0.0, 0.0, -1.0]))
    assert kind == "cg"                                                      # straight down: the CG stage floor
    d_out = np.array([*(HL.rot2(-h.rot) @ np.array([0.0, -1.0])), 0.0])    # toward the auditorium
    kind, hit = HL.proxy_hit(h, np.array([0.0, 0.0, 1.4]), d_out)
    assert kind == "proj" and hit[1] < HL.STAGE[2]
    q = HL.proxy_quads(h)
    assert len(q["stage_floor"]) == 1 and len(q["auditorium"]) >= 5


def test_tape_flutter_and_site():
    t = np.arange(0, 2, 1 / 30)
    flat = PL.tape_flutter(t, np.zeros(t.size))
    assert np.allclose(flat, 0.0)
    blow = PL.tape_flutter(t, np.full(t.size, 6.0))
    assert np.all(blow[:, 0] > 0.2) and np.std(blow[:, 3]) > 0.05
    root, d = PL.tape_end_site(np.array([[2.0, 2.0], [2.2, 1.9]]))
    assert root[0] < 0 or root[1] < 0                                         # the corner away from the operator
    assert np.linalg.norm(d) == pytest.approx(1.0)


def test_tele_establishing_shot_frames_pad_vehicle_and_sock():
    """Round 2: a level telephoto from ~26 m on the sun's side: the climbing vehicle AND the pad between the headroom and
    the HUD band, focus on the vehicle, the windsock whole in the background."""
    log = synth_log(push=False)
    shots = [SH.Shot(3.4, 5.0, "wide", params={"mode": "tele"})]
    cfg = PL.SceneCfg()
    plan = PL.build_plan(log, shots, FPS, cfg)
    k = plan["t"].size // 2
    cam, tgt, lens, sh = plan["cam_pos"][k], plan["cam_tgt"][k], float(plan["cam_lens"][k]), plan["cam_shift"][k]
    assert abs(float(tgt[2] - cam[2])) < 1e-6                                   # level: verticals stay vertical
    assert 50.0 <= lens <= 135.0
    assert float(np.hypot(*(cam[:2] - plan["pos"][k, :2]))) > 20.0
    for P_ in (plan["pos"][k], plan["pos"][k] + np.array([0, 0, 0.856]), np.array([0.0, 0.0, -0.11])):
        x, y, _ = PL.project(P_, cam, tgt, 0.0, lens, 1920, 1080, sh)
        assert 0 < x < 1920 and 0.10 * 1080 <= y <= 0.86 * 1080
    cg = plan["pos"][k] + np.array([0, 0, 0.376])
    assert abs(float(plan["cam_focus"][k]) - float(np.linalg.norm(cg - cam))) < 1e-6
    sx, sy = plan["meta"]["windsock_xy"]
    hz = plan["meta"]["windsock_h"]
    for z in (-0.11, -0.11 + hz):
        x, y, d = PL.project(np.array([sx, sy, z]), cam, tgt, 0.0, lens, 1920, 1080, sh)
        assert 0 < x < 1920 and 0 < y < 1080 and d > float(np.linalg.norm(cg - cam)) + 20.0
    assert np.all(plan["sock_vis"][plan["seg"] == 0] == 1.0)


def test_windsock_hidden_when_only_its_mast_is_in_frame():
    cfg = PL.SceneCfg()
    n = 3
    cam = {"pos": np.tile([0.0, -6.0, 1.5], (n, 1)), "tgt": np.tile([0.0, 0.0, 1.5], (n, 1)), "roll": np.zeros(n),
           "lens": np.full(n, 35.0), "shift": np.zeros((n, 2))}
    assert np.all(PL.windsock_visibility((0.0, 0.0), -0.11, cam, cfg) == 0.0)        # 6 m mast 6 m away: top cut
    assert np.all(PL.windsock_visibility((0.0, 60.0), -0.11, cam, cfg) == 1.0)       # far away: whole
    assert np.all(PL.windsock_visibility((40.0, -10.0), -0.11, cam, cfg) == 1.0)     # out of frame
    # a shot is decided as a whole: no mast popping in / out mid-shot
    cam2 = dict(cam, lens=np.array([35.0, 35.0, 12.0]))
    v = PL.windsock_visibility((0.0, 0.0), -0.11, cam2, cfg)
    assert v[2] == 1.0 and v[0] == 0.0
    assert np.all(PL.windsock_visibility((0.0, 0.0), -0.11, cam2, cfg, seg=np.zeros(3, int)) == 0.0)


def test_shell_qa_mask_projects_the_body():
    from render import qa as QA
    if not os.path.exists(os.path.join(ROOT, "cad", "exports", "cadquery", "stl", "avionics_module.stl")):
        pytest.skip("CAD STLs not exported")
    tris = QA.shell_tris(ROOT)
    pos, q = np.zeros(3), np.array([1.0, 0, 0, 0])
    cam, tgt = np.array([2.5, 0.0, 0.4]), np.array([0.0, 0.0, 0.4])
    cq = PL.look_quat(cam, tgt)
    m = QA.shell_mask(tris, pos, q, cam, cq, 50.0, 960, 540, 1.0)
    ys, xs = np.nonzero(m)
    assert m[270, 480]                                                        # the body covers the image centre
    expect = 0.856 * (50.0 / 36.0 * 960) / 2.5                               # pinhole height of the 0.856 m body
    assert abs((ys.max() - ys.min()) - expect) < 0.08 * expect and (xs.max() - xs.min()) < 200


def test_driver_helpers():
    import tools.blender_render as BR
    log = synth_log(push=True)
    assert BR.resolve_scene("auto", log) == "indoor"                   # pushes at a low hover = the demo
    log.meta["profile"] = "field"
    assert BR.resolve_scene("auto", log) in ("indoor", "outdoor")
    assert BR.resolve_scene("outdoor", log) == "outdoor"
    log2 = synth_log(push=False)
    log2.meta["profile"] = "demo"
    assert BR.resolve_scene("auto", log2) == "indoor"
    with pytest.raises(ValueError):
        BR.resolve_scene("moon", log2)
    t = np.array([0.0, 0.033, 0.066, 5.0, 5.033])
    assert BR.still_frames(t, np.zeros(5, int), [0.07, 5.0]) == [3, 4]
    with pytest.raises(ValueError):
        BR.still_frames(t, np.zeros(5, int), [2.5])
    plan = {"meta": {"frames": 3, "cam_kinds": ["wide", "closeup"]}, "seg": np.array([0, 1, 1])}
    g = BR.engine_for_frames(plan, "auto")
    assert sorted(sum(g.values(), [])) == [1, 2, 3]
    assert BR.engine_for_frames(plan, "cycles") == {"cycles": [1, 2, 3]}


# --------------------------------------------------------------------------- #
# round 1b: sound bed, per-shot exposure, shouldered pole, QA outer skin
# --------------------------------------------------------------------------- #
def test_sound_bed_follows_thrust_and_timing():
    from render import audio as AU
    n = 90
    T = np.r_[np.zeros(30), np.full(60, 25.0)]
    P = {"thrust": T, "pos": np.zeros((n, 3)), "cam_pos": np.tile([2.0, 0.0, 1.0], (n, 1)), "speed": np.ones(n)}
    x = AU.synth(P, 25.0, FPS, pre_s=1.0, post_s=0.5)
    assert x.shape == (int(round((1.0 + n / FPS + 0.5) * AU.SR)), 2) and x.dtype == np.float32
    sr = AU.SR
    pre = x[:sr]
    off = x[sr + int(0.45 * sr): sr + int(0.95 * sr)]           # motor off (after the fade-in)
    on = x[sr + int(1.3 * sr): sr + int(2.6 * sr)]              # full thrust
    rms = lambda a: float(np.sqrt(np.mean(a ** 2)))            # noqa: E731
    # round 2: room tone (>= 40 dB under the bed) instead of digital silence under the cards and the motor-off part
    assert 0.0 < rms(pre) < 0.01 * rms(on) and rms(off) < 0.01 * rms(on)
    assert rms(on) > 0.05 and AU.true_peak_dbtp(x) <= AU.TRUE_PEAK_DBTP + 0.05
    # the blade passing tone sits at rpm / 60 x 12 = 5.2 kHz at T = T_ref
    sp = np.abs(np.fft.rfft(on[:, 0] * np.hanning(on.shape[0])))
    f = np.fft.rfftfreq(on.shape[0], 1.0 / sr)
    band = (f > 4000) & (f < 6500)
    assert abs(float(f[band][np.argmax(sp[band])]) - 26000.0 / 60.0 * 12) < 150.0
    # deterministic, quieter from farther away
    assert np.array_equal(x, AU.synth(P, 25.0, FPS, pre_s=1.0, post_s=0.5))
    fp_near = AU.frame_params(P, 25.0)
    P2 = dict(P, cam_pos=np.tile([8.0, 0.0, 1.0], (n, 1)))
    assert float(AU.frame_params(P2, 25.0)["roar"][-1]) < 0.5 * float(fp_near["roar"][-1])
    assert any("합성" in c and "녹음 아님" in c for c in HUD.credit_lines(False, None, audio=True))
    assert not any("합성한 EDF" in c for c in HUD.credit_lines(False, None, audio=False))


def test_per_shot_exposure():
    assert PL.shot_ev("closeup", True) < -0.5 and PL.shot_ev("closeup", False) == pytest.approx(-0.7)
    assert PL.shot_ev("orbit", True) == 0.0 and PL.shot_ev("closeup", True, {"ev": -0.3}) == -0.3
    log = synth_log(push=False)
    shots = [SH.Shot(4.0, 6.0, "orbit", params={"azimuth_deg": 30.0}), SH.Shot(6.0, 8.0, "closeup", params={"mode": "vanes"})]
    plan = PL.build_plan(log, shots, FPS, PL.SceneCfg(indoor=False))
    assert "cam_ev" in plan and np.all(plan["cam_ev"][plan["seg"] == 0] == 0.0)
    assert np.all(plan["cam_ev"][plan["seg"] == 1] == pytest.approx(-0.7))
    tp = PL.turntable_plan(FPS, 2.0)
    assert np.all(tp["cam_ev"] == 0.0)


def test_pole_is_held_beside_the_body_at_chest_height():
    """Round 2: the held end sits 0.12 m right of the body's centre line at chest height, and the pole still lies on
    the line of the logged push (the feet move sideways to compensate), pitched at most 25 deg."""
    log = synth_log(push=True)
    tg = np.arange(log.t[0], log.t[-1], 0.02)
    R = quat_to_matrix(log.q[np.searchsorted(log.t, tg).clip(0, log.t.size - 1)])
    pos = PL.interp(log.t, log.pos, tg)
    pa = PL.hold(log.t, log.push_active, tg)
    pt = PL.person_track(tg, pos, R, log.ground_z, pa, PL.interp(log.t, log.push_force, tg),
                         PL.interp(log.t, log.push_point, tg))
    k = int(np.flatnonzero(pa)[5])
    Rz = PL._rot2(float(pt.yaw[k]))
    local = Rz.T @ (pt.hold[k] - pt.feet[k])
    assert abs(local[1] - PL.FIGURE_HANDS[1]) < 0.03 and PL.FIGURE_HANDS[1] < -0.1     # right of the centre line
    # chest height (the feet are planted: the held end follows the receding contact a little)
    assert 1.1 < PL.FIGURE_HANDS[2] < 1.3 and PL.FIGURE_HANDS[2] - 0.08 < local[2] < PL.FIGURE_HANDS[2] + 0.15
    u = np.array([0.0, 1.0])                                     # the logged force direction (+y)
    d = pt.pole_dir[k]
    assert float(np.dot(d[:2] / np.linalg.norm(d[:2]), u)) > 0.995
    assert math.degrees(math.asin(abs(d[2]))) <= PL.POLE_MAX_PITCH_DEG + 0.1


def test_shell_qa_uses_the_visible_outer_skin():
    from render import qa as QA
    if not os.path.exists(os.path.join(ROOT, "cad", "exports", "cadquery", "stl", "propulsion_module.stl")):
        pytest.skip("CAD STLs not exported")
    tris = QA.shell_tris(ROOT)
    c = tris.mean(axis=1)
    low = c[:, 2] < 0.17                                           # the propulsion module (duct exit .. EDF housing)
    assert float(np.min(np.hypot(c[low, 0], c[low, 1]))) >= QA.OUTER_R_MIN
    # looking straight up the exit from below: the open duct (r < 45 mm) is not part of the mask
    cam, tgt = np.array([0.0, 0.001, -0.6]), np.array([0.0, 0.0, 0.0])
    cq = PL.look_quat(cam, tgt)
    m = QA.shell_mask(tris, np.zeros(3), np.array([1.0, 0, 0, 0]), cam, cq, 50.0, 640, 360, 1.0)
    assert not m[180, 320] and m.sum() > 0


# --------------------------------------------------------------------------- #
# round 2
# --------------------------------------------------------------------------- #
def _push_track():
    pytest.importorskip("tools.sim_video")
    log = synth_log(push=True)
    tg = np.arange(log.t[0], log.t[-1], 0.02)
    pos = PL.interp(log.t, log.pos, tg)
    R = quat_to_matrix(PL.slerp_resample(log.t, log.q, tg))
    pa = PL.hold(log.t, log.push_active, tg)
    pf = PL.interp(log.t, log.push_force, tg)
    pp = PL.interp(log.t, log.push_point, tg)
    return log, tg, pos, R, pa, pf, pp, PL.person_track(tg, pos, R, -0.128, pa, pf, pp)


def test_feet_planted_through_every_push_and_no_walk_cycle():
    log, tg, pos, R, pa, pf, pp, pt = _push_track()
    assert np.allclose(pt.feet[:, 2], -0.128)                                     # on the stage floor, not the pad top
    for a, b in PL.push_runs(pa):
        w = (tg >= tg[a] - PL.PUSH_BLEND_S) & (tg <= tg[b] + PL.PUSH_BLEND_S)
        assert float(np.max(np.ptp(pt.feet[w, :2], axis=0))) < 1e-9                 # planted: zero foot travel
        assert float(np.max(pt.walk_w[w])) == 0.0
    assert float(np.max(pt.walk_w[pt.push_w > 0.05])) == 0.0
    # the walk between the two push positions advances the gait by distance, never by time
    moving = np.hypot(*np.gradient(pt.feet[:, :2], tg, axis=0).T) > 0.05
    if np.any(moving):
        d = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pt.feet[:, :2], axis=0).T))])
        assert np.allclose(pt.walk_phase, 2 * math.pi * d / 1.15)
    cg = pos + R[:, :, 2] * PL.Z_CG_DEFAULT
    assert float(np.min(np.hypot(*(pt.feet[:, :2] - cg[:, :2]).T))) >= PL.MIN_STANDOFF_M - 1e-6


def test_pad_snapped_tangent_in_every_push_frame():
    log = synth_log(push=True)
    shots = [SH.Shot(12.5, 15.5, "orbit", params={"kind": "push", "deg_per_s": 0.0})]
    plan = PL.build_plan(log, shots, FPS, PL.SceneCfg(indoor=True))
    act = np.flatnonzero(plan["push_active"])
    assert act.size > 5
    for k in act:
        R = quat_to_matrix(plan["q"][k])
        rel = plan["person_pad"][k] - plan["pos"][k]
        radial = rel - R[:, 2] * float(rel @ R[:, 2])
        assert float(np.linalg.norm(radial)) == pytest.approx(PL.BODY_R + PL.POLE_PAD_R, abs=1e-6)


def test_pose_gates_on_a_push_plan():
    pytest.importorskip("tools.human_figure")
    from render import pose as PO
    if not os.path.exists(os.path.join(ROOT, "render", "human", "human_lbs.npz")):
        pytest.skip("human asset not built")
    log = synth_log(push=True)
    shots = [SH.Shot(12.0, 15.5, "orbit", params={"kind": "push", "deg_per_s": 0.0})]
    plan = PL.build_plan(log, shots, FPS, PL.SceneCfg(indoor=True))
    P = {k: v for k, v in plan.items() if k != "meta"}
    r = PO.pose_qa(P, plan["meta"]["floor_z"], every=3)
    assert r["ok"], r["fail"][:5]
    assert r["worst"]["sole_m"] <= PO.SOLE_MAX_M and r["worst"]["pole_head_m"] >= PO.POLE_HEAD_MIN_M
    assert r["worst"]["fist_face_m"] >= PO.FIST_FACE_MIN_M and r["worst"]["walk_w_in_push"] == 0.0
    full = P["person_push_w"] > 0.99
    pitch = np.degrees(np.arcsin(np.clip(P["person_pole_dir"][full, 2], -1, 1)))
    assert float(np.max(np.abs(pitch))) <= PL.POLE_MAX_PITCH_DEG + 0.5


def test_push_two_shot_is_level_and_frames_both():
    log = synth_log(push=True)
    shots = [SH.Shot(13.0, 15.0, "orbit", params={"kind": "push", "deg_per_s": 4.0, "lens": 32.0})]
    plan = PL.build_plan(log, shots, FPS, PL.SceneCfg(indoor=False))
    k = int(np.flatnonzero(plan["push_active"])[3])
    cam, tgt, lens, sh = plan["cam_pos"][k], plan["cam_tgt"][k], float(plan["cam_lens"][k]), plan["cam_shift"][k]
    assert abs(float(tgt[2] - cam[2])) < 1e-6 and 18.0 <= lens <= 50.0
    assert np.ptp(plan["cam_lens"][plan["seg"] == 0]) < 1e-9                    # one lens for the shot
    feet = plan["person_feet"][k]
    for P_, lo, hi in ((plan["pos"][k] + np.array([0, 0, 0.856]), 0.05, 0.9), (feet, 0.1, 0.9),
                       (feet + np.array([0, 0, 1.75]), 0.05, 0.9)):
        x, y, _ = PL.project(P_, cam, tgt, 0.0, lens, 1920, 1080, sh)
        assert 0.03 * 1920 < x < 0.97 * 1920 and lo * 1080 <= y <= hi * 1080


def test_level_frame_puts_content_between_headroom_and_hud():
    cam = np.array([0.0, -5.0, 1.3])
    pts = np.array([[0.5, 0.0, 2.4], [-0.6, 0.0, -0.1], [0.0, 0.3, 1.0]])
    lens, sy = PL.level_frame(cam, np.array([0.0, 1.0, 0.0]), pts, 1920, 1080, 14.0, 80.0)
    tgt = cam + np.array([0.0, 5.0, 0.0])
    ys = [PL.project(p, cam, tgt, 0.0, lens, 1920, 1080, (0.0, sy))[1] / 1080 for p in pts]
    assert min(ys) >= PL.FRAME_TOP - 1e-6 and max(ys) <= PL.FRAME_BOTTOM + 1e-6
    assert abs(min(ys) - PL.FRAME_TOP) < 1e-3 or abs(max(ys) - PL.FRAME_BOTTOM) < 1e-3


def test_hall_fit_depends_on_the_log_not_the_shot_list():
    pytest.importorskip("tools.sim_video")
    log = synth_log(push=True)
    a = PL.build_plan(log, [SH.Shot(12.5, 15.0, "orbit", params={"kind": "push"})], FPS, PL.SceneCfg(indoor=True))
    b = PL.build_plan(log, [SH.Shot(3.0, 6.0, "spectator"), SH.Shot(17.5, 19.0, "closeup", params={"mode": "vanes"})],
                      FPS, PL.SceneCfg(indoor=True))
    assert a["meta"]["hall"]["rot_rad"] == b["meta"]["hall"]["rot_rad"]
    assert a["meta"]["hall"]["veh_img"] == b["meta"]["hall"]["veh_img"] and a["meta"]["cam_side"] == b["meta"]["cam_side"]
    kl = a["meta"]["key_light"]
    assert kl is not None and kl["pos"][2] - a["meta"]["floor_z"] == pytest.approx(PL.KEY_SPOT["height_m"])
    assert a["meta"]["white_balance_k"] == PL.WHITE_BALANCE_K["indoor"]


def test_vane_peak_time_and_readout():
    log = synth_log(push=False)
    th = [math.radians(45 + 90 * i) for i in range(4)]
    t = SH.vane_peak_time(log, 9.0, 12.0, th)
    assert abs(t - 10.6) < 0.02                                                 # the logged correction peak
    # common-mode deflection (roll trim) carries no net side force
    w = SH.vane_work(np.full((1, 4), math.radians(7.0)), th)
    assert float(w[0]) < 1e-12
    ro = HUD.vane_readout([math.radians(7.0)] * 4, th)
    assert ro["net_deg"] < 1e-9 and ro["trim_deg"] == pytest.approx(7.0)
    ro = HUD.vane_readout([math.radians(10.0), 0.0, -math.radians(10.0), 0.0], th)
    assert ro["net_deg"] == pytest.approx(20.0, abs=1e-6) and abs(ro["trim_deg"]) < 1e-9


def test_shot_list_display_titles_and_clean_default_title():
    obj = {"title_ko": "제목", "subtitle_ko": "부제", "shots": [{"t0": 1.0, "t1": 3.0, "cam": "orbit"}]}
    assert SH.shot_list_display(obj) == {"title_ko": "제목", "subtitle_ko": "부제"}
    assert len(SH.shots_from_json(obj)) == 1 and SH.shot_list_display([]) == {}
    log = synth_log(push=True)
    log.meta["title_ko"] = "합성 테스트 패턴: 1.5 m 호버링 + 3번 밀기 (운동학 곡선, 시뮬레이션 아님)"
    assert HUD.default_title(log) == "합성 테스트 패턴: 1.5 m 호버링 + 3번 밀기"


def test_end_card_reports_the_pushes_this_clip_shows():
    log = synth_log(push=True)
    edit_t = np.arange(13.0, 15.0, 1 / FPS)
    lab, val = [x for x in HUD.summary_numbers(log, edit_t) if "밀기" in x[0]][0]
    assert "전체 2회 중 1번째" in lab and val.startswith("1회") and "12 N" in val
    lab, val = [x for x in HUD.summary_numbers(log) if "밀기" in x[0]][0]
    assert lab == "밀기" and val.startswith("2회")


def test_shell_gates_flag_colour_and_sheen():
    from render import qa as QA
    ok = {"median": 40.0, "chroma": 3.0, "hi_frac": 0.02}
    assert QA.gate_warnings(1, ok) == []
    w = QA.gate_warnings(2, {"median": 55.0, "chroma": 14.0, "hi_frac": 0.35})
    assert len(w) == 2 and any("chroma" in x for x in w) and any("sheen" in x for x in w)


def test_props_never_cut_by_the_frame_edge():
    from render import qa as QA
    cam = np.array([0.0, -5.0, 0.6])
    q = PL.look_quat(cam, np.array([0.0, 0.0, 0.6]))
    cams = {"pos": [cam], "quat": [q], "lens": [35.0], "shift": [(0.0, 0.0)], "aspect": 9 / 16}
    assert QA.frame_safe((0.0, 0.0, 0.0), 0.15, 0.45, cams)                        # centre: inside, above the HUD band
    assert QA.frame_safe((20.0, 0.0, 0.0), 0.15, 0.45, cams)                       # far outside the frame
    hw = 5.0 * 18.0 / 35.0                                                         # half width of the view at 5 m
    assert not QA.frame_safe((hw, 0.0, 0.0), 0.15, 0.45, cams)                    # straddles the right edge
    assert not QA.frame_safe((0.0, -3.0, 0.0), 0.15, 0.45, cams)                  # low in the frame: under the HUD row


def test_sound_bed_loudness_fades_and_cut_jumps():
    from render import audio as AU
    n = 150
    T = np.full(n, 25.0)
    seg = np.r_[np.zeros(75, int), np.ones(75, int)]
    cam = np.r_[np.tile([1.5, 0.0, 1.0], (75, 1)), np.tile([12.0, 0.0, 1.0], (75, 1))]
    P = {"thrust": T, "pos": np.zeros((n, 3)), "cam_pos": cam, "speed": np.ones(n), "seg": seg}
    g = AU.frame_params(P, 25.0)["gain"]
    step = 20 * math.log10(float(np.mean(g[seg == 0])) / float(np.mean(g[seg == 1])))
    assert step == pytest.approx(AU.CUT_JUMP_DB, abs=0.05)                         # 18 dB perspective jump -> 4 dB
    x = AU.synth(P, 25.0, FPS, pre_s=2.0, post_s=2.0)
    assert AU.loudness_lufs(x) == pytest.approx(AU.TARGET_LUFS, abs=0.3)
    assert AU.true_peak_dbtp(x) <= AU.TRUE_PEAK_DBTP + 0.05
    sr = AU.SR
    card = x[int(0.5 * sr):int(1.8 * sr)]                                         # the title card: room tone, not silence
    rms = float(np.sqrt(np.mean(card ** 2)))
    assert 1e-5 < rms < 10 ** (-40 / 20)
    end = x[-int(1.0 * sr):-int(0.7 * sr)]                                        # deep in the end card: room tone only
    assert float(np.sqrt(np.mean(end ** 2))) < 10 ** (-40 / 20)
    into = x[int((2.0 + n / FPS) * sr):int((2.0 + n / FPS + 0.2) * sr)]          # the bed fades INTO the end card
    assert float(np.sqrt(np.mean(into ** 2))) > 10 ** (-35 / 20)


def test_turntable_reveal_beat():
    tp = PL.turntable_plan(FPS, 8.0)
    assert tp["studio_rim"][0] == 0.0 and tp["studio_key"][0] == 0.0
    i_rim = int(np.argmax(tp["studio_rim"] > 0.5))
    i_key = int(np.argmax(tp["studio_key"] > 0.5))
    assert 0 < i_rim < i_key < int(2.5 * FPS) and tp["studio_key"][-1] == 1.0
    d = np.linalg.norm(tp["cam_pos"][:, :2], axis=1)
    assert d[0] - d[int(2.4 * FPS)] > 0.45                                        # the push-in


def test_safety_officer_stands_clear_and_behind_the_action():
    pytest.importorskip("tools.sim_video")
    log = synth_log(push=True)
    shots = [SH.Shot(2.0, 5.0, "spectator"), SH.Shot(12.5, 15.0, "orbit", params={"kind": "push", "deg_per_s": 0.0})]
    plan = PL.build_plan(log, shots, FPS, PL.SceneCfg(indoor=True))
    ko = plan["meta"]["kill_operator"]
    assert ko is not None
    p = np.array(ko["xy"])
    assert float(np.min(np.hypot(*(plan["person_feet"][:, :2] - p).T))) >= 2.0 - 1e-6
    assert float(np.min(np.hypot(*(plan["cam_pos"][:, :2] - p).T))) >= 1.5 - 1e-6
    # never a foreground figure in the push two-shot: either out of frame or farther than the vehicle
    for k in np.flatnonzero(plan["seg"] == 1)[::5]:
        x, y, d = PL.project(np.array([p[0], p[1], plan["meta"]["floor_z"]]), plan["cam_pos"][k], plan["cam_tgt"][k], 0.0,
                             float(plan["cam_lens"][k]), 1920, 1080, plan["cam_shift"][k])
        if np.isfinite(x) and d > 0 and 0 <= x <= 1920 and 0 <= y <= 1080:
            assert d > float(np.linalg.norm(plan["pos"][k] - plan["cam_pos"][k])) + 0.5
