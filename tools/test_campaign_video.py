"""Tests for tools/campaign_video.py: the tracks contract loader, run status / counters, interpolation, the radial
scale, the summary numbers, frame determinism (a preview frame equals the video's frame at that time) and a small
smoke render.  The tracks used here are the module's synthetic KINEMATIC TEST PATTERN (labelled so in its meta);
nothing here is a simulation result.

Run: ``.venv/Scripts/python.exe -m pytest tools/test_campaign_video.py -q``
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import campaign_video as cv  # noqa: E402

try:
    import importlib.util
    HAVE_FFMPEG = importlib.util.find_spec("imageio_ffmpeg") is not None
except ImportError:  # pragma: no cover
    HAVE_FFMPEG = False


@pytest.fixture(scope="module")
def synth(tmp_path_factory):
    d = tmp_path_factory.mktemp("camp")
    return cv.write_synthetic_tracks(str(d / "tracks.npz"), runs=40, seed=3)


def _minimal(path: str, **over) -> str:
    T, R = 5, 2
    data = {"t": np.arange(T, dtype=float), "pos": np.zeros((R, T, 3), dtype=np.float32), "alive": np.ones((R, T), bool),
            "phase": np.zeros((R, T), np.uint8), "state": np.zeros((R, T), np.uint8), "outcome": np.array([0, 1], np.uint8),
            "outcome_names": np.array(["pass", "landed_fail"]), "meta": np.array(json.dumps({"title": "x"}))}
    data.update(over)
    for k in [k for k, v in data.items() if v is None]:
        data.pop(k)
    np.savez(path, **data)
    return path


# --------------------------------------------------------------------------- #
# loading
# --------------------------------------------------------------------------- #
def test_load_synthetic(synth):
    trk = cv.load_tracks(os.path.dirname(synth))                  # a campaign directory works too
    assert trk.R == 40 and trk.T == trk.t.size and trk.pos.shape == (40, trk.T, 3)
    assert trk.warnings == []
    assert np.all(np.isfinite(trk.pos))                          # NaN after the end is forward-filled
    assert trk.outcome_names[:4] == ["pass", "landed_fail", "abort", "not_landed"]
    assert trk.label(0, "ko") == "착지 - 모든 기준 통과" and trk.label(2, "en") == "abort"
    assert trk.color(2) == cv.OUTCOME_COLORS["abort"]
    assert "wind_xy" in trk.opt and trk.opt["wind_xy"].shape == (40, trk.T, 2)
    assert len(trk.sha256) == 64 and "합성 테스트 패턴" in trk.meta["title_ko"]
    e = trk.end_index()
    assert np.all(trk.alive[np.arange(trk.R), e])
    assert np.all(~trk.alive[np.arange(trk.R)[:, None], np.arange(trk.T)[None, :]] | (np.arange(trk.T)[None, :] <= e[:, None]))


def test_load_rejects_bad(tmp_path):
    with pytest.raises(ValueError, match="'alive'"):
        cv.load_tracks(_minimal(str(tmp_path / "a.npz"), alive=None))
    with pytest.raises(ValueError, match="outcome code 3"):
        cv.load_tracks(_minimal(str(tmp_path / "b.npz"), outcome=np.array([0, 3], np.uint8)))
    with pytest.raises(ValueError, match="strictly increasing"):
        cv.load_tracks(_minimal(str(tmp_path / "c.npz"), t=np.array([0.0, 1.0, 1.0, 2.0, 3.0])))
    with pytest.raises(ValueError, match="pos"):
        cv.load_tracks(_minimal(str(tmp_path / "d.npz"), pos=np.zeros((2, 4, 3))))
    p = np.zeros((2, 5, 3))
    p[1] = np.nan
    with pytest.raises(ValueError, match="no finite position"):
        cv.load_tracks(_minimal(str(tmp_path / "e.npz"), pos=p))
    trk = cv.load_tracks(_minimal(str(tmp_path / "f.npz"), outcome_names=np.array(["pass", "weird"])))
    assert any("weird" in w for w in trk.warnings) and trk.color(1) == cv.OUTCOME_COLORS["other"]


# --------------------------------------------------------------------------- #
# status / counters
# --------------------------------------------------------------------------- #
def test_run_status_sequences():
    S, A, P, U, AB = cv.STATE_SAFE, cv.STATE_ARMED, cv.STATE_POWERED, 3, cv.STATE_ABORT
    #            0  1  2  3  4  5  6  7  8
    state = np.array([[S, A, P, P, U, S, A, P, P],          # hop, landed, safe, armed again (a second hop)
                      [S, S, A, P, P, AB, 4, 4, 4],         # abort, release: latched
                      [S, A, P, P, P, P, P, P, P],          # still flying when its simulation ends at 5
                      [S, S, S, S, S, S, S, S, S]], dtype=np.uint8)
    phase = np.array([[0, 1, 2, 3, 5, 0, 1, 2, 3],
                      [0, 0, 1, 2, 3, 0, 0, 0, 0],
                      [0, 1, 2, 3, 4, 4, 4, 4, 4],
                      [0] * 9], dtype=np.uint8)
    alive = np.ones((4, 9), dtype=bool)
    alive[2, 6:] = False
    alive[3, :] = False
    st = cv.run_status(phase, state, alive)
    W, F, L, AB_, E = cv.WAITING, cv.FLYING, cv.LANDED, cv.ABORTED, cv.ENDED
    assert st[0].tolist() == [W, F, F, F, L, L, F, F, F]
    assert st[1].tolist() == [W, W, F, F, F, AB_, AB_, AB_, AB_]
    assert st[2].tolist() == [W, F, F, F, F, F, E, E, E]
    assert st[3].tolist() == [W] * 9
    c = cv.status_counts(st)
    assert c.shape == (9, 5) and np.all(c.sum(axis=1) == 4)


def test_counts_on_synthetic(synth):
    trk = cv.load_tracks(synth)
    st = cv.run_status(trk.phase, trk.state, trk.alive)
    c = cv.status_counts(st)
    assert np.all(c.sum(axis=1) == trk.R)
    names = trk.outcome_names
    fin = st[:, -1]
    for r in range(trk.R):
        oc = names[int(trk.outcome[r])]
        expect = {"pass": cv.LANDED, "landed_fail": cv.LANDED, "abort": cv.ABORTED, "not_landed": cv.ENDED}[oc]
        assert fin[r] == expect, (r, oc, fin[r])


def test_interp_runs_clamps_at_end():
    t = np.array([0.0, 1.0, 2.0, 3.0])
    y = np.array([[[0.0], [1.0], [2.0], [3.0]], [[0.0], [10.0], [99.0], [99.0]]])
    end = np.array([3, 1])
    v = cv.interp_runs(t, y, 1.5, end)
    assert v[0, 0] == pytest.approx(1.5) and v[1, 0] == pytest.approx(10.0)
    assert cv.interp_runs(t, y, 9.0, end)[0, 0] == pytest.approx(3.0)


def test_radial_scale():
    mode, R = cv.radial_mode(np.array([1.0, 2.0, 3.0, 2.5]))
    assert mode == "linear" and R >= 3.0
    mode, R = cv.radial_mode(np.array([1.0, 2.0, 2.0, 2.0, 300.0]))
    assert mode == "sqrt" and R >= 300.0
    r = cv.radial_px(np.array([0.0, R / 4, R]), R, 400.0, "sqrt")
    assert r[0] == 0.0 and r[1] == pytest.approx(200.0) and r[2] == pytest.approx(400.0)
    assert cv.ring_values(500.0, "sqrt")[-1] == 500.0
    assert cv.ring_values(20.0, "linear") == [5.0, 10.0, 15.0, 20.0]


def test_summary_stats(synth):
    trk = cv.load_tracks(synth)
    sm = cv.summary_stats(trk)
    assert sm["runs"] == 40 and sum(sm["outcomes"].values()) == 40
    assert sm["touchdown_abs_vz"]["n"] == int(np.count_nonzero(np.isfinite(trk.opt["touchdown_vz"])))
    assert sm["drift_max_m"]["max"] == pytest.approx(float(np.nanmax(trk.opt["drift_max_m"])))
    assert isinstance(sm["failed_criteria"], dict)
    assert cv.sampling_items({"wind_mps": [0.0, 6.0], "x": 3}, "ko") == ["평균 풍속 [m/s] 0~6", "x 3"]


# --------------------------------------------------------------------------- #
# rendering
# --------------------------------------------------------------------------- #
def test_preview_frame_equals_sequential_frame(synth):
    """Frames are a pure function of time: the incremental trail layers drawn in sequence give the same picture as a
    renderer that jumps straight to that time (it draws the history at the video's frame spacing)."""
    trk = cv.load_tracks(synth)
    r1 = cv.CampaignRenderer(trk, fps=5.0, speed=4.0, size=(640, 360))
    k = 12
    for tq in r1.tq[:k]:
        r1.frame(float(tq))
    a = r1.frame(float(r1.tq[k]))
    r2 = cv.CampaignRenderer(trk, fps=5.0, speed=4.0, size=(640, 360))
    b = r2.frame(float(r1.tq[k]))
    assert a.shape == (360, 640, 3) and np.array_equal(a, b)
    c = r2.frame(float(r1.tq[3]))                   # going back restarts the layers
    r3 = cv.CampaignRenderer(trk, fps=5.0, speed=4.0, size=(640, 360))
    assert np.array_equal(c, r3.frame(float(r1.tq[3])))


@pytest.mark.skipif(not HAVE_FFMPEG, reason="imageio-ffmpeg not installed")
def test_smoke_render(synth, tmp_path):
    import imageio_ffmpeg
    out = str(tmp_path / "v" / "camp.mp4")
    info = cv.render_campaign(synth, out, fps=6.0, size=(480, 270), speed=10.0, title_s=0.5, end_s=0.5, quiet=True)
    n, _ = imageio_ffmpeg.count_frames_and_secs(out)
    assert n == info["frames"] == 3 + info["body_frames"] + 3
    trk = cv.load_tracks(synth)
    t_last = float(trk.t[int(trk.end_index().max())])                 # the video ends with the longest run
    assert info["body_frames"] == cv.sv.frame_times(float(trk.t[0]), t_last, 6.0, 10.0).size
    for suffix in ("_poster.png", "_summary.png", ".json"):
        assert os.path.exists(os.path.join(tmp_path, "v", "camp" + suffix))
    assert sum(info["final_status_counts"].values()) == 40 and info["s_per_frame"] > 0
    info2 = cv.render_campaign(synth, str(tmp_path / "p.mp4"), size=(480, 270), at=[3.0, 30.0], lang="en", quiet=True)
    assert len(info2["files"]) == 4 and all(os.path.exists(f) for f in info2["files"])


def test_cli_synthetic(tmp_path, capsys):
    rc = cv.main(["--synthetic", str(tmp_path / "s"), "--runs", "12"])
    assert rc == 0 and os.path.exists(tmp_path / "s" / "tracks.npz")
    assert cv.load_tracks(str(tmp_path / "s")).R == 12


# --------------------------------------------------------------------------- #
# v0.8: hold-point-relative tracks
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def synth_hold(tmp_path_factory):
    d = tmp_path_factory.mktemp("camp_hold")
    return cv.write_synthetic_tracks(str(d / "tracks.npz"), runs=30, seed=5, with_hold=True)


def test_hold_pattern_and_origin(synth, synth_hold):
    trk = cv.load_tracks(synth_hold)
    assert trk.opt["hold_xy"].shape == (30, 2) and np.all(np.isfinite(trk.opt["hold_xy"]))
    for k in cv.V08_SCALARS:
        assert k in trk.opt
    assert cv.resolve_origin(trk) == "hold" and cv.resolve_origin(trk, "pad") == "pad"
    plain = cv.load_tracks(synth)
    assert cv.resolve_origin(plain) == "pad"
    with pytest.raises(ValueError):
        cv.resolve_origin(plain, "hold")
    with pytest.raises(ValueError):
        cv.resolve_origin(plain, "moon")
    off = cv.map_offsets(trk, "hold")
    np.testing.assert_allclose(off, trk.opt["hold_xy"])
    assert np.all(cv.map_offsets(trk, "pad") == 0.0)
    # relative distances: in HOLD the pattern stays near its hold point, from the pad it is far
    d_hold = cv.max_dist_from(trk, off)
    assert np.median(cv.max_drift(trk)) > np.median(trk.opt["sk_max_m"])
    assert np.all(d_hold >= np.nan_to_num(trk.opt["sk_max_m"]) - 1e-6)
    sm = cv.summary_stats(trk)
    assert sm["runs_with_hold_point"] == 30 and sm["sk_max_m"]["n"] == 30


def test_hold_relative_render_and_pad_unchanged(synth, synth_hold):
    trk = cv.load_tracks(synth_hold)
    r = cv.CampaignRenderer(trk, fps=5.0, speed=4.0, size=(640, 360))
    assert r.origin == "hold" and r.sk_crit == 3.0 and r.n_no_hold == 0
    j = int(np.searchsorted(trk.t, 20.0))
    np.testing.assert_allclose(r.pos_map[:, j, :2], trk.pos[:, j, :2] - trk.opt["hold_xy"])
    a = r.frame(20.0)
    assert a.shape == (360, 640, 3)
    r2 = cv.CampaignRenderer(trk, fps=5.0, speed=4.0, size=(640, 360), origin="pad")
    assert r2.origin == "pad" and np.array_equal(r2.pos_map, trk.pos)
    assert not np.array_equal(a, r2.frame(20.0))
    end = r.end_card()
    assert end.shape == (360, 640, 3)
    # a run without a hold point stays relative to the pad and is counted
    trk.opt["hold_xy"] = trk.opt["hold_xy"].copy()
    trk.opt["hold_xy"][0] = np.nan
    r3 = cv.CampaignRenderer(trk, fps=5.0, speed=4.0, size=(640, 360))
    assert r3.n_no_hold == 1 and np.array_equal(r3.pos_map[0], trk.pos[0])


def test_write_tracks_hold_xy(tmp_path):
    t = np.arange(0.0, 3.0, 0.1)
    run = {"t": t, "pos": np.zeros((t.size, 3)), "phase": np.zeros(t.size), "state": np.zeros(t.size), "outcome": 0}
    p = cv.write_tracks(str(tmp_path / "w.npz"), [dict(run, hold_xy=[1.0, 2.0], sk_max_m=0.7), dict(run)], {"title": "x"})
    trk = cv.load_tracks(p)
    assert np.allclose(trk.opt["hold_xy"][0], [1.0, 2.0]) and np.all(np.isnan(trk.opt["hold_xy"][1]))
    assert trk.opt["sk_max_m"][0] == pytest.approx(0.7) and np.isnan(trk.opt["sk_max_m"][1])
