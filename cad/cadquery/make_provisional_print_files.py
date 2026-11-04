"""v0.8 / v0.8.1 provisional print files of the parts the GPS pod and the GNS board change, made from the CadQuery twin's STLs.

Invariant: the official print set (cad/exports/print, tools/make_print_files.py) is made only from a Fusion 360 run of the
current params.json and script; that run needs the Fusion GUI, which this machine's agents cannot drive.  Until it
exists, this script writes a clearly separate PROVISIONAL set - cad/exports/print/provisional_v081_cadquery/ (v0.8.1;
the v0.8.0 set in provisional_v080_cadquery/ is superseded) - for the three printed parts v0.8 / v0.8.1 change or add
(payload_module with the GPS pod, avionics_module with the lead entry hole and the four GNS-board bosses, the gps_pod_lid;
v0.8.1 also thins the pod walls / floor / lid to 1.2 mm) from cad/exports/cadquery/stl, oriented and sliced exactly as make_print_files does (its orient /
slice_one: P2S, PLA Matte, 5 walls, 15 % gyroid, its support rules), every number taken from Bambu Studio's own slice.
The twin and the Fusion script's construction agree on these parts (cad/fusion360/replay_x3.py: boolean differences
0.000 mm^3), but no Fusion kernel has built them yet: print_plan_provisional.json records that, and the set is superseded
by make_print_files.py as soon as the Fusion run exists.  Every other part keeps its v0.6.3 print file (unchanged solid).

Usage: python cad/cadquery/make_provisional_print_files.py [--no-slice]
Outputs: cad/exports/print/provisional_v081_cadquery/<part>.stl (oriented), <part>.3mf (Bambu Studio sliced project),
print_plan_provisional.{json,md}
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import make_print_files as mpf  # noqa: E402
import trimesh  # noqa: E402

SRC = os.path.join(ROOT, "cad", "exports", "cadquery", "stl")
OUT = os.path.join(ROOT, "cad", "exports", "print", "provisional_v081_cadquery")
CQ_REPORT = os.path.join(ROOT, "cad", "exports", "cadquery_report.json")
# the two changed modules keep make_print_files' orientation and support rule; the new lid prints flat, top face down
# Bambu Studio (CLI, P2S profile) refuses to slice the v0.8 payload module with make_print_files' tree(auto) supports
# ("One of the plate is empty or has no object fully inside it", every Z rotation and arrange setting tried, tree_organic
# and support_on_build_plate_only too); the same part slices without supports and with normal(auto), as the launch pad
# and the intake already use.  Measured (support extrusions mapped back to the vehicle frame): with normal(auto) the pod
# sector gets 1.2 m of support path (the connector pocket's ceiling), the bore under the bulkhead 730 m as before (754 m
# for the v0.7.1 module); so the pod itself needs none.
SUPPORT_OVERRIDES = {"payload_module": {"support_type": "normal(auto)"}}
PARTS = {
    "payload_module": mpf.PARTS["payload_module"][:3] + (
        mpf.PARTS["payload_module"][3] + "; v0.8: the GPS pod's floor rises at 45 deg and its raceway is a vertical "
        "tunnel (no support there; 1.2 m of support in the connector pocket's ceiling), its corner bosses bridge 5 mm; "
        "normal(auto) supports (the CLI refuses tree(auto) on this part); v0.8.1: pod walls / floor 1.2 mm",),
    "avionics_module": mpf.PARTS["avionics_module"][:3] + (mpf.PARTS["avionics_module"][3] + "; v0.8: + the GPS lead hole; "
                                                           "v0.8.1: + four 45-deg diamond GNS bosses on the -x wall",),
    "gps_pod_lid": (1, [[("x", 180.0)]], False, "flat: lid top face on the bed, the three ground-plane posts up"),
}


def sha256(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--no-slice", action="store_true")
    a = ap.parse_args(argv)
    with open(CQ_REPORT, "r", encoding="utf-8") as f:
        rep = json.load(f)
    params_sha = sha256(os.path.join(ROOT, "params.json"))
    if rep.get("params_sha256") != params_sha:
        print("REFUSED: cad/exports/cadquery_report.json is not from the current params.json - rebuild the twin first")
        return 2
    L = mpf.layout.build()
    if not getattr(L, "has_gps_pod", False):
        print("REFUSED: params carry no GPS pod")
        return 2
    rho = float(L.p["printer"]["density_g_cm3"])
    os.makedirs(OUT, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix="cansat_prov_slice_")
    plan = {"provisional": True,
            "source": {"kind": "CadQuery twin STL (cad/exports/cadquery/stl) - NOT the Fusion 360 path",
                       "why": "make_print_files.py makes print files only from a Fusion run of the current params.json and "
                              "script; the v0.8 Fusion GUI run has not been made",
                       "replaced_by": "python tools/make_print_files.py after the Fusion run of CanSatEDFRocket.py "
                                      f"{mpf.repo_script_version()} (then delete this folder)",
                       "construction_check": "cad/fusion360/replay_x3.py: the Fusion script's recorded feature calls, rebuilt "
                                             "in OCC, equal these solids (boolean differences < 1 mm^3)",
                       "params_sha256": params_sha, "cadquery_report_params_sha256": rep.get("params_sha256"),
                       "preset": L.p["meta"]["preset"], "stl_sha256": {}},
            "machine": mpf.MACHINE, "process_base": mpf.PROCESS, "filament": mpf.FILAMENT, "parts": {}}
    try:
        for name, (qty, cands, sup, why) in PARTS.items():
            src = os.path.join(SRC, f"{name}.stl")
            mesh = trimesh.load(src, force="mesh")
            plan["source"]["stl_sha256"][name] = sha256(src)
            rots = cands[0]
            m = mpf.orient(mesh, rots)
            out_stl = os.path.join(OUT, f"{name}.stl")
            m.export(out_stl)
            entry = {"rotations": rots, "extents_mm": [round(float(x), 1) for x in m.extents], "print_file_stl": f"{name}.stl"}
            if not a.no_slice and os.path.exists(mpf.BAMBU):
                cand = os.path.join(tmp, f"{name}.stl")
                m.export(cand)
                res = mpf.slice_one(cand, sup, tmp, rho, {**(mpf.PART_OVERRIDES.get(name) or {}),
                                                          **SUPPORT_OVERRIDES.get(name, {})})
                if res.get("ok"):
                    shutil.copyfile(res["_project"], os.path.join(OUT, f"{name}.3mf"))
                    entry["print_file"] = f"{name}.3mf"
                entry.update({k: v for k, v in res.items() if not k.startswith("_")})
            used = mpf.used_support_type(name, sup, entry)
            plan["parts"][name] = {"quantity": qty, "supports": sup, "support_type_used": used, "why": why, **entry}
            print(f"{name}: {entry['extents_mm']} "
                  + (f"{mpf.fmt_t(entry['time_s'])}, {entry['filament_g']:.1f} g, support {used}" if entry.get("ok")
                     else entry.get("error", "not sliced")))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    ok = [v for v in plan["parts"].values() if v.get("ok")]
    plan["totals"] = {"time_s": sum(v["time_s"] * v["quantity"] for v in ok),
                      "filament_g": sum(v["filament_g"] * v["quantity"] for v in ok)}
    with open(os.path.join(OUT, "print_plan_provisional.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(plan, f, indent=2, ensure_ascii=False)
    lines = ["# 임시 출력 파일 (v0.8 GPS 포드 + v0.8.1 GNS 보드) - CadQuery 트윈에서 만든 것", "",
             "**주의: 임시본입니다.** 공식 출력 파일(`cad/exports/print`)은 Fusion 360에서 스크립트를 실행한 결과로만 만듭니다. "
             "v0.8.1 Fusion 실행 전까지 GPS 포드·GNS 보드 때문에 바뀐 세 부품(페이로드 모듈: GPS 포드, 벽·바닥 1.2 mm; 항공전자 "
             "모듈: GPS 선 구멍 + GNS 보스 4개; GPS 포드 뚜껑 1.2 mm)만 CadQuery 트윈 STL에서 같은 방식(방향, 슬라이서 설정)으로 "
             "만들었습니다. Fusion 스크립트의 피처 호출을 OCC로 재현한 형상이 이 형상과 같다는 것은 확인했습니다 "
             "(`cad/fusion360/replay_x3.py`, 불리언 차이 0 mm³). Fusion 실행 뒤 `python tools/make_print_files.py`가 만든 "
             "공식 파일로 바꾸고 이 폴더는 지웁니다. 나머지 부품은 v0.6.3 출력 파일을 그대로 씁니다 (형상 변화 없음).", "",
             f"프린터 `{mpf.MACHINE}`, 공정 `{mpf.PROCESS}` 기반 + 벽 5겹, 15 % 자이로이드, 필라멘트 `{mpf.FILAMENT}` "
             f"(Bambu PLA Matte 검정, 밀도 {rho} g/cm³).", "",
             "| 부품 | 출력 파일 | 수량 | 출력 방향 | 서포트 | 크기 (mm) | 1개 출력 시간 | 1개 필라멘트 |", "|---|---|---|---|---|---|---|---|"]
    for name, v in plan["parts"].items():
        sl = mpf.support_label(v["support_type_used"], (v.get("settings_in_gcode") or {}).get("support_style"))
        lines.append(f"| {name} | `{v.get('print_file', v['print_file_stl'])}` | {v['quantity']} | {v['why']} | {sl} | "
                     f"{' x '.join(str(x) for x in v['extents_mm'])} | "
                     + (f"{mpf.fmt_t(v['time_s'])} | {v['filament_g']:.1f} g |" if v.get("ok") else "슬라이스 안 함 | - |"))
    lines += ["", f"**합계**: {mpf.fmt_t(plan['totals']['time_s'])}, PLA {plan['totals']['filament_g']:.0f} g", ""]
    with open(os.path.join(OUT, "print_plan_provisional.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines))
    print("\n".join(lines[-2:]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
