"""Tests of tools/compare_video.py: alignment, resampling, metric extraction, spec parsing, the timing spec, the
shared camera helpers, and a small render smoke test (synthetic logs; real logs only where present, else skipped)."""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_HERE, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import compare_video as cv  # noqa: E402
import sim_video as sv  # noqa: E402

try:
    import importlib.util
    HAVE_PV = importlib.util.find_spec("pyvista") is not None and importlib.util.find_spec("imageio_ffmpeg") is not None
except ImportError:  # pragma: no cover
    HAVE_PV = False

HIST = os.path.join(_ROOT, "history", "v0.7.0", "vtol_logs")


# --------------------------------------------------------------------------- #
# synthetic VTOL-shaped logs (kinematic test patterns, not simulations)
# --------------------------------------------------------------------------- #
def make_log(path: str, hz: float = 50.0, t_end: float = 20.0, t_motor: float = 2.0, t_td: float = 15.0,
             drift_mps: float = 0.2, td_text: str | None = None, extras: bool = True, h_bias: float = 0.0) -> str:
    """SAFE -> ARMED (1 s) -> POWERED at t_motor: CLIMB to 3 m, HOLD, LAND (t_td - 4), touchdown at t_td, LANDED
    0.5 s later, SAFE at t_td + 2.  Drift along +x at ``drift_mps`` while airborne."""
    t = np.round(np.arange(0.0, t_end + 1e-9, 1.0 / hz), 9)
    n = t.size
    state = np.zeros(n, dtype=np.int8)
    phase = np.zeros(n, dtype=np.int8)
    state[(t >= 1.0) & (t < t_motor)] = 1
    phase[(t >= 1.0) & (t < t_motor)] = 1
    pw = (t >= t_motor) & (t < t_td + 0.5)
    state[pw] = 2
    phase[(t >= t_motor) & (t < t_motor + 3.0)] = 2
    phase[(t >= t_motor + 3.0) & (t < t_td - 4.0)] = 3
    phase[(t >= t_td - 4.0) & (t < t_td + 0.5)] = 4
    state[(t >= t_td + 0.5) & (t < t_td + 2.0)] = 3
    phase[(t >= t_td + 0.5) & (t < t_td + 2.0)] = 5
    h = np.clip((t - t_motor) * 1.0, 0.0, 3.0)
    h = np.where(t >= t_td - 4.0, np.clip(3.0 - (t - (t_td - 4.0)) * 0.75, 0.0, 3.0), h)
    h = np.where(t < t_motor, 0.0, h)
    x = np.clip(t - t_motor, 0.0, t_td - t_motor) * drift_mps
    pos = np.column_stack([x, np.zeros(n), h])
    q = np.tile([1.0, 0.0, 0.0, 0.0], (n, 1))
    tgt = np.where((t >= t_motor) & (t < t_td - 4.0), 3.0, np.nan)
    ref = np.where((t >= t_motor) & (t < t_td + 0.5), h - h_bias, np.nan)
    if td_text is None:
        td_text = f"touchdown #1 on the ground (z -0.11 m) at v_z -0.40 m/s, horizontal {drift_mps:.2f} m/s, tilt 1.5 deg, POWERED/LAND"
    meta = {"scenario": "synthetic", "title": "SYNTHETIC TEST PATTERN (kinematic, not a simulation)", "seed": 1,
            "mode": "vtol", "params_sha256": "abc", "profile": "field",
            "criteria": {"touchdown_vz_max_mps": 1.0, "touchdown_vh_max_mps": 0.5, "touchdown_tilt_max_deg": 5.0,
                         "hold_rms_err_max_m": 0.5}}
    data = {"t": t, "pos": pos, "vel": np.gradient(pos, t, axis=0), "q": q, "throttle": np.where(pw, 0.9, 0.0),
            "thrust": np.where(pw, 27.0, 0.0), "state": state, "phase": phase, "h_target": tgt, "h_ref": ref,
            "h_est": h, "tilt_deg": np.where(pw, 1.0, 0.0), "wind": np.zeros((n, 3)), "gust_active": np.zeros(n, bool),
            "ground_z": np.array(0.0),
            "event_t": np.array([1.0, t_motor, t_motor + 0.4, t_td, t_td + 0.5]),
            "event_text": np.array(["SAFE -> ARMED: ON", "ARMED -> POWERED: motor on", "liftoff (T = 25 N)", td_text,
                                    "POWERED -> UNPOWERED: touchdown confirmed: motor off (LANDED)"]),
            "cmd_t": np.array([0.9, t_td - 4.2]), "cmd_name": np.array(["ON", "OFF"]),
            "meta": np.array(json.dumps(meta))}
    if extras:
        vc = np.zeros(n, dtype=np.int8)
        vc[t >= t_td] = 1
        data.update({"h_true": h, "tilt_ctrl_deg": np.where(pw, 0.5, 0.0), "tilt_est_err_deg": np.where(pw, 2.0, 0.0),
                     "reason": np.zeros(n, dtype=np.uint8), "vc_td_by": vc})
    np.savez(path, **data)
    return path


def add_gps(path: str, outage: tuple[float, float] = (8.0, 9.0), hold_offset: tuple[float, float] = (0.0, 0.0)) -> str:
    """The v0.8 keys on a ``make_log`` pattern (kinematic, not a simulation): hold_point = the CG at the first HOLD
    sample (+ ``hold_offset``), GPS usable except during ``outage`` (0 satellites), 9 satellites otherwise."""
    with np.load(path) as d:
        data = {k: d[k] for k in d.files}
    t, ph = data["t"], data["phase"]
    i0 = int(np.flatnonzero(ph == 3)[0])
    cg = data["pos"][i0] + np.array([0.0, 0.0, 0.39])                 # q = identity: the CG is straight above
    data["hold_point"] = np.array([cg[0] + hold_offset[0], cg[1] + hold_offset[1], 0.0])
    ok = ~((t >= outage[0]) & (t < outage[1]))
    data["gps_ok"] = ok
    data["gps_sats"] = np.where(ok, 9, 0)
    meta = json.loads(str(data["meta"]))
    meta["gps_on"] = True
    meta["criteria"]["station_keeping_max_m"] = {"field": 3.0, "demo": 2.5}
    data["meta"] = np.array(json.dumps(meta))
    np.savez(path, **data)
    return path


@pytest.fixture()
def pair(tmp_path):
    b = cv.load_flight(make_log(str(tmp_path / "b.npz"), t_motor=2.0, t_td=15.0, drift_mps=0.4, h_bias=0.2))
    a = cv.load_flight(make_log(str(tmp_path / "a.npz"), t_motor=3.0, t_td=14.0, t_end=18.0, drift_mps=0.1))
    return b, a


# --------------------------------------------------------------------------- #
# alignment
# --------------------------------------------------------------------------- #
def test_parse_align():
    assert cv.parse_align("motor_on") == cv.Align("motor_on", None)
    assert cv.parse_align(" touchdown ").event == "touchdown"
    al = cv.parse_align("offset:1.5,-2")
    assert al.offsets == (1.5, -2.0) and al.event is None and "지정" in al.label
    assert cv.parse_align("motor_on").label == "모터 시동"
    assert "OFF" in cv.parse_align("cmd:off").label
    for bad in ("", "offset:1", "offset:a,b", "offset:1,nan"):
        with pytest.raises(ValueError):
            cv.parse_align(bad)


def test_event_time(pair):
    b, a = pair
    lg = b.log
    assert cv.event_time(lg, "motor_on") == pytest.approx(2.0)
    assert cv.event_time(lg, "liftoff") == pytest.approx(2.4)
    assert cv.event_time(lg, "touchdown") == pytest.approx(15.0)
    assert cv.event_time(lg, "landed") == pytest.approx(15.5)
    assert cv.event_time(lg, "hold") == pytest.approx(5.0)
    assert cv.event_time(lg, "land") == pytest.approx(11.0)
    assert cv.event_time(lg, "on_cmd") == pytest.approx(0.9)
    assert cv.event_time(lg, "cmd:OFF") == pytest.approx(10.8)
    assert cv.event_time(lg, "start") == 0.0
    assert cv.event_time(lg, "event:motor on") == pytest.approx(2.0)
    assert cv.event_time(lg, "UNPOWERED") == pytest.approx(15.5)          # free text, case-insensitive
    with pytest.raises(ValueError):
        cv.event_time(lg, "no such event")
    assert cv.align_offsets(b.log, a.log, cv.parse_align("motor_on")) == (pytest.approx(2.0), pytest.approx(3.0))
    assert cv.align_offsets(b.log, a.log, cv.parse_align("offset:4,5")) == (4.0, 5.0)


def test_touchdown_event_is_number_one_only(tmp_path):
    p = make_log(str(tmp_path / "x.npz"), td_text="touchdown #12 on the ground at v_z -0.1 m/s, horizontal 0.1 m/s")
    with pytest.raises(ValueError):
        cv.event_time(cv.load_flight(p).log, "touchdown")


def test_tau_window():
    spans, offs = [(0.0, 20.0), (0.0, 18.0)], [2.0, 3.0]
    assert cv.tau_window(spans, offs) == (pytest.approx(-2.0), pytest.approx(18.0))     # pre 2 s, the later end
    assert cv.tau_window(spans, offs, pre_s=1.0)[0] == pytest.approx(-1.0)
    assert cv.tau_window([(1.0, 20.0), (0.0, 18.0)], offs)[0] == pytest.approx(-1.0)   # a log that starts late
    assert cv.tau_window(spans, offs, start=5.0, end=7.0) == (5.0, 7.0)
    with pytest.raises(ValueError):
        cv.tau_window(spans, offs, start=5.0, end=5.0)


def test_map_side_freezes_and_flags():
    tau = np.array([-3.0, -1.0, 0.0, 10.0, 17.0, 20.0])
    tq, st = cv.map_side(tau, 2.0, 0.0, 18.0)
    np.testing.assert_allclose(tq, [0.0, 1.0, 2.0, 12.0, 18.0, 18.0])
    assert st.tolist() == [cv.PRE, cv.RUN, cv.RUN, cv.RUN, cv.ENDED, cv.ENDED]


def test_frame_grid_and_frozen_shorter_log(pair):
    b, a = pair
    offs = cv.align_offsets(b.log, a.log, cv.Align())
    t0, t1 = cv.tau_window([(float(f.t[0]), float(f.t[-1])) for f in (b, a)], offs)
    tau = sv.frame_times(t0, t1, 30.0, 1.0)
    tq_a, st_a = cv.map_side(tau, offs[1], float(a.t[0]), float(a.t[-1]))
    tq_b, st_b = cv.map_side(tau, offs[0], float(b.t[0]), float(b.t[-1]))
    assert np.all(st_b == cv.RUN)                       # the longer log runs to the end of the video
    ended = st_a == cv.ENDED
    assert ended.any() and np.all(tq_a[ended] == a.t[-1])
    fr = sv.resample(a.log, tq_a)
    assert np.allclose(fr.pos[ended], a.log.pos[-1])    # frozen at its last sample


# --------------------------------------------------------------------------- #
# series / resampling
# --------------------------------------------------------------------------- #
def test_channel_series(tmp_path, pair):
    b, _ = pair
    alt = cv.channel_series(b, "alt", 2.0)
    assert [s.role for s in alt] == ["main", "target"] and alt[1].step
    np.testing.assert_allclose(alt[0].tau, b.t - 2.0)
    assert b.height_src == "h_true"
    tilt = cv.channel_series(b, "tilt", 0.0)
    assert [s.role for s in tilt] == ["main", "est"]
    np.testing.assert_allclose(cv.channel_series(b, "dist", 0.0)[0].y, np.hypot(b.log.pos[:, 0], b.log.pos[:, 1]))
    nox = cv.load_flight(make_log(str(tmp_path / "n.npz"), extras=False))
    assert nox.height_src == "pos_z" and cv.channel_series(nox, "est_err", 0.0) == []
    assert [s.role for s in cv.channel_series(nox, "tilt", 0.0)] == ["main"]
    with pytest.raises(ValueError):
        cv.channel_series(b, "phase", 0.0)


def test_value_at_clamps_and_holds():
    s = cv.Series("main", np.array([0.0, 1.0, 2.0]), np.array([0.0, 10.0, 20.0]), "x")
    assert cv.value_at(s, 0.5) == pytest.approx(5.0)
    assert cv.value_at(s, 9.0) == pytest.approx(20.0) and cv.value_at(s, -1.0) == pytest.approx(0.0)
    st = cv.Series("target", np.array([0.0, 1.0, 2.0]), np.array([1.0, 2.0, 3.0]), "x", step=True)
    assert cv.value_at(st, 1.9) == 2.0
    assert math.isnan(cv.value_at(cv.Series("main", np.zeros(0), np.zeros(0), "x"), 1.0))


def test_phase_spans_and_limits():
    ph = np.array([0, 0, 2, 2, 3, 3, 3, 5])
    tau = np.arange(8.0)
    assert cv.phase_spans(ph, tau) == [(0.0, 2.0, 0), (2.0, 4.0, 2), (4.0, 7.0, 3), (7.0, 7.0, 5)]
    lo, hi = cv.series_limits([np.array([0.0, 2.0]), np.array([1.0, np.nan, 4.0])], floor0=True)
    assert lo == 0.0 and 4.0 < hi < 4.5
    lo, hi = cv.series_limits([np.array([5.0, 5.0])], min_span=1.0)
    assert hi - lo >= 1.0 and lo < 5.0 < hi


# --------------------------------------------------------------------------- #
# metrics
# --------------------------------------------------------------------------- #
def test_touchdowns_parse():
    lg = type("L", (), {})()
    lg.event_t = np.array([1.0, 2.0, 3.0])
    lg.event_text = ["touchdown #1 on the ground (z -0.11 m) at v_z -0.34 m/s, horizontal 7.66 m/s, tilt 7.5 deg, POWERED/LAND",
                     "lift-off again", "touchdown #2 at v_z -0.1 m/s, horizontal 0.2 m/s"]
    tds = cv.touchdowns(lg)
    assert [d["i"] for d in tds] == [1, 2]
    assert tds[0]["vz"] == -0.34 and tds[0]["vh"] == 7.66 and tds[0]["tilt"] == 7.5 and tds[1]["tilt"] is None


def test_extract_metrics(pair):
    b, a = pair
    m = cv.extract_metrics(b)
    assert m["td_vz"] == -0.40 and m["td_vh"] == pytest.approx(0.40) and m["td_tilt"] == 1.5
    assert m["td_dist"] == pytest.approx(0.4 * 13.0, abs=1e-6)            # x at the touchdown time
    assert m["hold_rms"] == pytest.approx(0.2, abs=1e-9)                     # h_true - h_ref = h_bias in HOLD
    assert m["est_err_rms"] == pytest.approx(2.0) and m["est_err_max"] == pytest.approx(2.0)
    assert m["t_land"] == pytest.approx(4.0) and m["t_flight"] == pytest.approx(13.5)
    assert m["drift_max"] == pytest.approx(5.2) and m["max_tilt"] == pytest.approx(1.0)
    assert m["n_touchdowns"] == 1 and m["landed_by"] == cv.LANDED_BY_KO[1] and m["reasons"] == []
    assert cv.extract_metrics(a)["hold_rms"] == pytest.approx(0.0, abs=1e-12)


def test_extract_metrics_without_extras(tmp_path):
    fl = cv.load_flight(make_log(str(tmp_path / "n.npz"), extras=False))
    m = cv.extract_metrics(fl)
    assert m["est_err_rms"] is None and m["hold_rms"] is None and m["landed_by"] is None and m["reasons"] is None
    assert m["td_vz"] == -0.40


def test_metrics_from_json_and_combine(tmp_path):
    js = {"scenarios": {"S": {"metrics": {"touchdown_vz": -0.5, "touchdown_vh": 1.0, "rms_tilt_est_err_deg": 3.0,
                                          "touchdown_t_s": 20.0, "t_land_s": 12.0, "t_landed_s": 22.0,
                                          "t_motor_start_s": 2.0, "landed_by": "floor", "interlocks": ["V10 x"]}}}}
    p = tmp_path / "m.json"
    p.write_text(json.dumps(js), encoding="utf-8")
    mj = cv.metrics_from_json(f"{p}#S")
    assert mj["td_vz"] == -0.5 and mj["est_err_rms"] == 3.0 and mj["t_land"] == 8.0 and mj["t_flight"] == 20.0
    assert mj["landed_by"] == cv.LANDED_BY_KO["floor"] and mj["reasons"] == ["V10 x"]
    assert cv.metrics_from_json("m.json#S", str(tmp_path))["td_vh"] == 1.0              # relative to base_dir
    with pytest.raises(ValueError):
        cv.metrics_from_json(f"{p}#NOPE")
    with pytest.raises(ValueError):
        cv.metrics_from_json(str(p))
    vals, src, warns = cv.combine_metrics(["td_vz", "td_vh", "est_err_rms", "hold_rms", "td_tilt"],
                                          {"td_vz": -0.4, "td_vh": 1.0, "est_err_rms": None, "hold_rms": None,
                                           "td_tilt": 2.0},
                                          mj, {"td_tilt": 9.9})
    assert (vals["td_vz"], src["td_vz"]) == (-0.4, "log")             # log first
    assert (vals["est_err_rms"], src["est_err_rms"]) == (3.0, "json")  # the log lacks it
    assert (vals["hold_rms"], src["hold_rms"]) == (None, "none")
    assert (vals["td_tilt"], src["td_tilt"]) == (9.9, "spec")          # the spec overrides
    assert any(w.startswith("td_vz") for w in warns) and not any(w.startswith("td_vh") for w in warns)


def test_fmt_and_change():
    assert cv.fmt_metric("td_vz", -0.344) == "-0.34 m/s"
    assert cv.fmt_metric("td_tilt", 7.46) == "7.5°"
    assert cv.fmt_metric("est_err_rms", None) == "기록 없음"
    assert cv.fmt_metric("reasons", []) == "없음" and cv.fmt_metric("reasons", ["a", "b"]) == "a, b"
    assert cv.fmt_metric("n_touchdowns", 2) == "2 회"
    txt, tone = cv.metric_change("td_vh", 4.69, 1.64)
    assert tone == "better" and txt.startswith("-3.05 m/s") and "(-65 %)" in txt
    assert cv.metric_change("td_vz", -0.39, -0.43)[1] == "worse"            # |v_z| grew
    assert cv.metric_change("td_vz", -0.40, 0.40)[1] == "same"
    assert cv.metric_change("t_land", 14.0, 13.0)[1] == "na"                # no better direction
    assert cv.metric_change("est_err_rms", None, 1.0) == ("비교 불가", "na")
    assert cv.metric_change("landed_by", "x", "x") == ("같음", "same")


def test_criteria_and_isolation_warnings():
    metas = [{"criteria": {"touchdown_vh_max_mps": 0.5}}, {"criteria": {"touchdown_vh_max_mps": 0.7}}]
    assert cv.criterion_limit("td_vh", metas) == 0.5 and cv.criterion_limit("td_dist", metas) is None
    same = {"params_sha256": "a", "seed": 1, "profile": "field"}
    assert cv.isolation_warnings("isolated", [same, dict(same)]) == []
    w = cv.isolation_warnings("isolated", [same, dict(same, params_sha256="b", seed=2)])
    assert len(w) == 2
    assert cv.isolation_warnings("control+model", [same, dict(same, params_sha256="b")]) == []


# --------------------------------------------------------------------------- #
# spec
# --------------------------------------------------------------------------- #
def test_parse_spec_minimal_and_defaults(tmp_path):
    sp = cv.parse_spec({"title": "변경 1: 테스트"}, str(tmp_path))
    assert sp.isolation == "unknown" and sp.channels == cv.DEFAULT_CHANNELS and sp.metrics == cv.DEFAULT_METRICS
    assert sp.note == cv.ISOLATION_NOTE["unknown"]
    assert sp.resolve("x.npz") == os.path.normpath(os.path.join(str(tmp_path), "x.npz"))
    assert sp.resolve(None) is None


def test_parse_spec_full_and_load(tmp_path):
    d = {"title": "T", "what": "한 줄", "why": ["a", "b"], "isolation": "control+model", "channels": "tilt,phase",
         "metrics": ["td_vz"], "metric_values": {"before": {"td_vz": -1.0}}, "metrics_json": {"after": "m.json#S"},
         "camera": {"sync": "shared", "framing": "ground", "d_max": 20}, "title_s": 3, "poster_tau": 5,
         "comment": "ignored"}
    p = tmp_path / "s.json"
    p.write_text(json.dumps(d, ensure_ascii=False), encoding="utf-8")
    sp = cv.load_spec(str(p))
    assert sp.what == ["한 줄"] and sp.channels == ("tilt", "phase") and sp.metric_values["before"]["td_vz"] == -1.0
    assert sp.camera["sync"] == "shared" and sp.poster_tau == 5.0 and sp.base_dir == str(tmp_path)
    assert "모델" in sp.note


@pytest.mark.parametrize("bad", [
    {}, {"title": ""}, {"title": "T", "isolation": "maybe"}, {"title": "T", "channels": ["alt", "nope"]},
    {"title": "T", "channels": ["phase"]}, {"title": "T", "metrics": ["nope"]},
    {"title": "T", "metric_values": {"middle": {}}}, {"title": "T", "metric_values": {"before": {"nope": 1}}},
    {"title": "T", "metrics_json": {"before": 3}}, {"title": "T", "camera": {"zoom": 2}},
    {"title": "T", "camera": {"sync": "x"}}, {"title": "T", "align": "offset:1"}, {"title": "T", "extra_key": 1},
    {"title": "T", "what": [{"a": 1}]}, {"title": "T", "title_s": -1}])
def test_parse_spec_rejects(bad):
    with pytest.raises(ValueError):
        cv.parse_spec(bad)


def test_project_specs_parse():
    d = os.path.join(_ROOT, "sim", "out", "compare", "specs")
    if not os.path.isdir(d):
        pytest.skip("no project specs")
    for name in sorted(os.listdir(d)):
        with open(os.path.join(d, name), encoding="utf-8") as f:
            js = json.load(f)
        if "paths" in js:
            sp = cv.parse_timing_spec(js)
            assert sp.paths
        else:
            sp = cv.load_spec(os.path.join(d, name))
            assert sp.title and sp.isolation in cv.ISOLATIONS


# --------------------------------------------------------------------------- #
# timing spec
# --------------------------------------------------------------------------- #
def test_timing_spec():
    d = {"title": "tick", "tick_us": 4000, "order": {"before": ["a", "b"], "after": ["b", "a"]},
         "paths": [{"name": "p", "limit_us": 4710,
                    "before": {"segments": [{"label": "x", "us": 4375, "kind": "derived"},
                                            {"label": "y", "us": 796, "kind": "measured"}], "total_kind": "measured"},
                    "after": {"segments": [{"label": "z", "us": None, "kind": "unmeasured"}]}}]}
    sp = cv.parse_timing_spec(d)
    p = sp.paths[0]
    assert p.before.total_us == pytest.approx(5171.0) and p.after.total_us is None
    assert cv.total_qualifier(p.before) == "측정"
    assert cv.total_qualifier(cv.TimingBar([cv.TimingSegment("a", 1.0, "derived"), cv.TimingSegment("b", 1.0, "measured")])) == "혼합"
    assert cv.total_qualifier(cv.TimingBar([cv.TimingSegment("a", 1.0, "assumed")])) == cv.KIND_KO["assumed"]
    assert sp.axis_max_us == 6000.0                          # 1.12 x the longest total, rounded up to 1 ms
    assert cv.sweep_us(0, 30.0, 10.0, 6000.0) == 0.0 and cv.sweep_us(300, 30.0, 10.0, 6000.0) == 6000.0
    assert cv.sweep_us(150, 30.0, 10.0, 6000.0) == pytest.approx(3000.0)
    for bad in ({}, {"title": "t", "paths": []},
                {"title": "t", "paths": [{"name": "p", "before": {"segments": [{"label": "a", "kind": "measured"}]}}]},
                {"title": "t", "paths": [{"name": "p", "before": {"segments": [{"label": "a", "us": 1, "kind": "guess"}]}}]},
                {"title": "t", "paths": [{"name": "p", "before": {"segments": [], "total_kind": "x"}}]}):
        with pytest.raises(ValueError):
            cv.parse_timing_spec(bad)


def test_timing_renderer_frames():
    d = {"title": "tick", "paths": [{"name": "p", "before": {"segments": [{"label": "x", "us": 5000}]},
                                     "after": {"segments": [{"label": "y", "us": 4000, "kind": "assumed"}], "note": "n"}}],
         "sweep_s": 1, "hold_s": 1}
    r = cv.TimingRenderer(cv.parse_timing_spec(d), fps=10.0, size=(960, 540))
    assert r.n_body == 20
    for k in (0, 5, 19):
        assert r.frame(k).shape == (540, 960, 3)
    assert r.title_card().shape == r.end_card().shape == (540, 960, 3)


# --------------------------------------------------------------------------- #
# drawing helpers, camera helpers
# --------------------------------------------------------------------------- #
def test_safe_glyphs():
    assert cv.safe("−0.3 ≈ 1") == "-0.3 ~ 1"
    assert cv.safe("→ τ °") == "→ τ °"


def test_shared_distance_and_reaim():
    a = np.full(60, 10.0)
    b = np.full(60, 6.0)
    b[20:30] = 14.0
    d = cv.shared_distance([a, b], 30.0, 1.0)
    assert np.all(d >= 10.0 - 1e-9) and d.max() <= 14.0 + 1e-9
    assert cv.shared_distance([a, a], 30.0, 1.0) == pytest.approx(a)
    # re-aiming keeps the framed point at the same place in the box when the distance grows
    box = (-0.62, 0.86, -0.56, 0.62)
    az, el, va, asp = np.full(1, 90.0), 12.0, 38.0, 936 / 556
    c = np.array([1.0, 2.0, 5.0])
    d_old, d_new = np.array([8.0]), np.array([13.0])
    th_v = math.tan(math.radians(va) / 2.0)
    _, r, u = sv.camera_basis(90.0, el)
    bcx, bcy = 0.5 * (box[0] + box[1]), 0.5 * (box[2] + box[3])
    foc_old = c - r * bcx * d_old[0] * th_v * asp - u * bcy * d_old[0] * th_v
    foc_new = cv.reaim(foc_old[None, :], d_old, d_new, az, el, va, asp, box)[0]
    f, _, _ = sv.camera_basis(90.0, el)
    for foc, dd in ((foc_old, d_old[0]), (foc_new, d_new[0])):
        cam = sv.CamView(foc - dd * f, foc, u, va)
        q, _ = sv._ndc(c[None, :], cam, asp)
        assert q[0] == pytest.approx([bcx, bcy], abs=1e-9)
    np.testing.assert_allclose(cv.reaim(foc_old[None, :], d_old, d_old, az, el, va, asp, box)[0], foc_old)


def test_shared_rings_restores():
    orig = sv.Scene.__dict__["_ring_radii"]
    with cv.shared_rings([1.0, 5.0]):
        assert sv.Scene._ring_radii(None) == [1.0, 5.0]
    assert sv.Scene.__dict__["_ring_radii"] is orig


def test_topview_scale_shared():
    t1 = cv.TopView(74.8, 232, 1.0, 90.0, cv.BEFORE_C)
    t2 = cv.TopView(74.8, 232, 1.0, 90.0, cv.AFTER_C)
    assert t1.R == t2.R == 100.0 and t1.k == t2.k
    im = t1.frame(np.array([[0.0, 0.0], [10.0, 0.0], [20.0, 5.0]]))
    assert im.size == (t1.W, t1.H)


# --------------------------------------------------------------------------- #
# real logs (present on the development PC; skipped elsewhere)
# --------------------------------------------------------------------------- #
@pytest.mark.skipif(not os.path.exists(os.path.join(HIST, "V_nominal.npz")), reason="v0.7.0 archive not present")
def test_metrics_match_sim_on_v070_log():
    fl = cv.load_flight(os.path.join(HIST, "V_nominal.npz"))
    m = cv.extract_metrics(fl)
    mj = cv.metrics_from_json(os.path.join(HIST, "vtol_metrics.json") + "#V_nominal")
    for k in ("td_dist", "hold_rms", "drift_max", "max_tilt", "t_land", "t_flight"):
        assert m[k] == pytest.approx(mj[k], rel=1e-6, abs=0.3 if k == "td_dist" else 1e-6), k
    for k in ("td_vz", "td_vh", "td_tilt"):                  # the event text carries them rounded
        assert m[k] == pytest.approx(mj[k], abs=0.051), k
    assert m["est_err_rms"] is None                           # v0.7.0 logs have no tilt_est_err_deg
    assert cv.event_time(fl.log, "motor_on") == pytest.approx(3.704)


# --------------------------------------------------------------------------- #
# render smoke test
# --------------------------------------------------------------------------- #
@pytest.mark.skipif(not HAVE_PV, reason="pyvista / imageio-ffmpeg not installed")
def test_render_smoke(tmp_path, pair):
    b, a = pair
    spec = cv.parse_spec({"title": "변경 테스트: 합성 패턴", "isolation": "isolated", "what": ["a"], "why": ["b"],
                          "channels": ["alt", "tilt", "est_err", "dist", "phase", "touchdown"]}, str(tmp_path))
    r = cv.CompareRenderer(b, a, spec, cv.parse_align("motor_on"), spec.channels, fps=10.0, size=(960, 540),
                           style="schematic", aa="none", asset_dir=None)
    try:
        assert r.offsets == (pytest.approx(2.0), pytest.approx(3.0))
        assert r.sides[0].rnd.cam is r.plans[0]
        assert np.allclose(r.plans[0].dist, r.plans[1].dist)            # one camera distance for both sides
        assert r.camera_info["in_view_fraction"] == [1.0, 1.0]
        ks = [0, r.tau.size // 2, r.tau.size - 1]
        views: list[list[np.ndarray]] = [[], []]
        for side in (0, 1):
            r.render_pass(side, ks, lambda j, img, side=side: views[side].append(img))
        for j, k in enumerate(ks):
            img = r.frame(k, (views[0][j], views[1][j]))
            assert img.shape == (540, 960, 3)
        assert r.maps[1][1][-1] == cv.ENDED and r.maps[0][1][-1] == cv.RUN    # the shorter log is frozen
        tag = img[0:r.lay.banner_h, r.lay.W // 2:]                            # the simulation tag (red) is drawn
        assert np.any((tag[..., 0] > 180) & (tag[..., 1] < 80) & (tag[..., 2] < 80))
        assert r.title_card().shape == r.end_card().shape == (540, 960, 3)
    finally:
        r.close()


# --------------------------------------------------------------------------- #
# v0.8: station keeping channels / metrics, the shared scene
# --------------------------------------------------------------------------- #
@pytest.fixture()
def gps_pair(tmp_path):
    """before = a v0.7.1-style log (no GPS keys, drifts 0.4 m/s), after = the same pattern drifting 0.02 m/s with the
    v0.8 keys (hold point, GPS outage 8-9 s)."""
    b = cv.load_flight(make_log(str(tmp_path / "b.npz"), t_motor=2.0, t_td=15.0, drift_mps=0.4))
    a = cv.load_flight(add_gps(make_log(str(tmp_path / "a.npz"), t_motor=2.0, t_td=15.0, drift_mps=0.02)))
    return b, a


def test_hold_reference_logged_and_derived(gps_pair):
    b, a = gps_pair
    hb, sb = b.hold
    ha, sa = a.hold
    assert (sb, sa) == ("derived", "log")
    t = b.t
    i0 = int(np.flatnonzero(b.log.phase == cv.PHASE_HOLD)[0])
    assert np.all(np.isnan(hb[:i0, 0])) and np.allclose(hb[i0:], b.log.pos[i0, :2])     # the v0.8 rule on the log
    hd = b.hold_dist()
    k = int(np.searchsorted(t, 9.0))
    assert hd[k] == pytest.approx(0.4 * (t[k] - t[i0]), abs=1e-6)                       # drift since the HOLD start
    assert cv.first_hold_xy(ha) == pytest.approx(a.log.pos[i0, :2])
    assert cv.hold_reference(cv.load_flight(make_log(str(os.path.dirname(b.path) + "/n.npz"), t_motor=2.0, t_td=4.0)).log) \
        == (None, "none")                                       # no HOLD phase: no reference


def test_v08_channel_series(gps_pair):
    b, a = gps_pair
    s = cv.channel_series(b, "hold_dist", 2.0)
    assert len(s) == 1 and "계산" in s[0].label and np.isnan(s[0].y[0])
    assert cv.channel_series(a, "hold_dist", 2.0)[0].label == "유지점 거리"
    hs = cv.channel_series(b, "hspeed", 0.0)[0]
    np.testing.assert_allclose(hs.y, np.hypot(b.log.vel[:, 0], b.log.vel[:, 1]))
    g = cv.channel_series(a, "gps", 0.0)
    assert len(g) == 1 and g[0].step and set(np.unique(g[0].y)) == {0.0, 9.0}
    assert cv.channel_series(b, "gps", 0.0) == []                 # no GPS in the v0.7.1-style log
    assert cv.parse_channels(cv.V08_CHANNELS) == cv.V08_CHANNELS


def test_v08_metrics_and_profile_criteria(gps_pair):
    b, a = gps_pair
    mb, ma = cv.extract_metrics(b), cv.extract_metrics(a)
    hold = a.log.phase == cv.PHASE_HOLD
    hd = a.hold_dist()
    assert ma["sk_rms"] == pytest.approx(float(np.sqrt(np.mean(hd[hold] ** 2))))
    assert ma["sk_max"] == pytest.approx(float(hd[hold].max())) and mb["sk_max"] > 10.0 * ma["sk_max"]
    assert ma["td_hold"] == pytest.approx(0.02 * (15.0 - 5.0), abs=1e-6)            # drift from the HOLD start
    dt = np.diff(np.append(a.t, a.t[-1]))
    P = a.log.state == sv.STATE_POWERED
    assert ma["gps_avail"] == pytest.approx(100.0 * np.sum(dt[P & a.log.gps_ok]) / np.sum(dt[P]))
    assert mb["gps_avail"] is None
    assert cv.criterion_limit("sk_max", [a.log.meta]) == 3.0                          # the profile 'field' entry
    assert cv.criterion_limit("sk_max", [dict(a.log.meta, profile="demo")]) == 2.5
    assert cv.fmt_metric("gps_avail", 91.7) == "92 %"
    assert cv.metric_change("sk_max", 4.0, 1.0)[1] == "better"


def test_parse_spec_layout():
    sp = cv.parse_spec({"title": "T", "layout": "shared", "ghost_align": "hold"})
    assert sp.layout == "shared" and sp.ghost_align == "hold"
    assert cv.parse_spec({"title": "T"}).layout == "split"
    for bad in ({"title": "T", "layout": "both"}, {"title": "T", "ghost_align": "nose"}):
        with pytest.raises(ValueError):
            cv.parse_spec(bad)
    lay = cv.CompareLayout.for_size(1920, 1080, shared=True)
    assert lay.view_x == (16, 16) and lay.col_x == 16 + lay.view_w + 16 and lay.col_x + lay.col_w == 1920 - 16
    assert cv.CompareLayout.for_size(1920, 1080) == cv.CompareLayout.for_size(1920, 1080, shared=False)


def test_topview_hold_and_clipped_other():
    tv = cv.TopView(3.0, 232, 1.0, 90.0, cv.AFTER_C)
    far = np.array([[0.0, 0.0], [10.0, 0.0], [40.0, 5.0]])
    im = tv.frame(np.array([[0.0, 0.0], [0.5, 0.2]]), hold=np.array([0.3, 0.1]), hold_d=0.22,
                  others=[(far, sv.hex_rgb(cv.BEFORE_C), "변경 전")])
    a = np.asarray(im)
    assert im.size == (tv.W, tv.H)
    # the far track is clipped to the map: nothing of it is drawn in the header band (where unclipped points would land)
    hold_px = tv.px(np.array([0.3, 0.1]))
    col = np.array(sv.hex_rgb(sv.HOLD_C))
    near = a[int(hold_px[1]) - 8:int(hold_px[1]) + 9, int(hold_px[0]) - 8:int(hold_px[0]) + 9, :3].astype(int)
    assert np.any(np.abs(near - col).sum(axis=2) < 60)               # the hold mark is drawn


@pytest.mark.skipif(not HAVE_PV, reason="pyvista / imageio-ffmpeg not installed")
def test_render_shared_layout(tmp_path, gps_pair):
    b, a = gps_pair
    spec = cv.parse_spec({"title": "한 장면 비교 테스트: 합성 패턴", "isolation": "isolated", "layout": "shared",
                          "ghost_align": "hold", "channels": list(cv.V08_CHANNELS),
                          "metrics": list(cv.V08_METRICS)}, str(tmp_path))
    r = cv.CompareRenderer(b, a, spec, cv.parse_align("motor_on"), spec.channels, fps=10.0, size=(960, 540),
                           style="schematic", aa="none", asset_dir=None)
    try:
        assert r.layout == "shared" and len(r.sides) == 1 and isinstance(r.sides[0], cv.SharedSide)
        # ghost_align hold: the before log moved so its (derived) hold point lies on the after hold point
        hb, ha = cv.first_hold_xy(b.hold[0]), cv.first_hold_xy(a.hold[0])
        assert r.shift_xy == pytest.approx(ha - hb)
        sd = r.sides[0]
        k = r.tau.size // 2
        assert sd.ghost_fr.pos[k, :2] == pytest.approx(sv.interp_linear(b.t, b.log.pos, np.array([sd.ghost_tq[k]]))[0, :2]
                                                      + r.shift_xy)
        ks = [0, k, r.tau.size - 1]
        views: list[np.ndarray] = []
        r.render_pass(0, ks, lambda j, img: views.append(img))
        for j, kk in enumerate(ks):
            img = r.frame(kk, (views[j],))
            assert img.shape == (540, 960, 3)
        assert r.camera_info["layout"] == "shared" and len(r.camera_info["in_view_fraction"]) == 2
        assert any("계산" in nt for nt in r.auto_notes)                         # the before hold point is derived
        assert r.title_card().shape == r.end_card().shape == (540, 960, 3)
        assert r.metrics[1]["values"]["sk_max"] is not None
    finally:
        r.close()


@pytest.mark.skipif(not HAVE_PV, reason="pyvista / imageio-ffmpeg not installed")
def test_render_compare_cli_shared_preview(tmp_path, gps_pair):
    b, a = gps_pair
    sp = tmp_path / "s.json"
    sp.write_text(json.dumps({"title": "CLI 테스트", "before": b.path, "after": a.path, "channels": ["alt", "hold_dist"]},
                             ensure_ascii=False), encoding="utf-8")
    rc = cv.main(["--spec", str(sp), "--out", str(tmp_path / "o.mp4"), "--preview", "2", "--size", "960x540",
                  "--style", "schematic", "--aa", "none", "--layout", "shared"])
    assert rc == 0
    with open(tmp_path / "o_frames.json", encoding="utf-8") as f:
        side = json.load(f)
    assert side["layout"] == "shared" and side["hold_reference"] == {"before": "derived", "after": "log"}
    assert side["frames"] == 2 and len(side["pass_s"]) == 1
