"""Download the CC0 Poly Haven assets the Blender scenes use (reproducible, verified, idempotent).

Run with the project venv:  python render/fetch_assets.py [--check]

Every asset lands in ``render/assets/polyhaven/<id>/`` (git-ignored).  Only api.polyhaven.com (file
lists, md5 sums, metadata) and dl.polyhaven.org (the files) are contacted.  A file is downloaded only
when it is missing or its size / md5 does not match the API listing, so a second run is a no-op.  The
manifest ``render/assets/manifest.json`` records id, resolution, files, bytes and md5; CREDITS.md (Korean
+ English, for the presentation credits) is written next to it.

Invariant: nothing outside the ASSETS list is fetched; the total stays under BUDGET_MB.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
import urllib.request
from typing import Any

ROOT = os.path.dirname(os.path.abspath(__file__))
ASSET_DIR = os.path.join(ROOT, "assets")
PH_DIR = os.path.join(ASSET_DIR, "polyhaven")
API = "https://api.polyhaven.com"
ALLOWED_HOSTS = ("api.polyhaven.com", "dl.polyhaven.org")
UA = {"User-Agent": "CanSat-EDF-render/1.0 (asset fetch, CC0)"}
BUDGET_MB = 200.0

# (id, kind, resolution, which maps / format, why)
ASSETS: list[dict[str, Any]] = [
    {"id": "alps_field", "kind": "hdri", "res": "4k", "fmt": "hdr",
     "why": "outdoor field: open mown meadow to the horizon, mountains, clear sky with a visible sun (~30 deg) for crisp shadows"},
    {"id": "meadow_2", "kind": "hdri", "res": "4k", "fmt": "hdr",
     "why": "outdoor alternative: lush summer meadow with trees, partly cloudy, visible sun (~40 deg)"},
    {"id": "school_hall", "kind": "hdri", "res": "4k", "fmt": "hdr",
     "why": "indoor demo hall: school auditorium with a light wooden floor, stage and ceiling lights (projected onto the hall box)"},
    {"id": "studio_small_09", "kind": "hdri", "res": "2k", "fmt": "hdr",
     "why": "studio softboxes for the vehicle-reveal turntable reflections"},
    {"id": "leafy_grass", "kind": "texture", "res": "2k", "maps": ("Diffuse", "nor_gl", "Rough"), "fmt": "jpg",
     "why": "lawn soil / grass base under the instanced grass blades"},
    {"id": "concrete_floor_worn_001", "kind": "texture", "res": "2k", "maps": ("Diffuse", "nor_gl", "Rough"), "fmt": "jpg",
     "why": "concrete launch slab under the pad (outdoor)"},
    {"id": "plywood", "kind": "texture", "res": "2k", "maps": ("Diffuse", "nor_gl", "Rough"), "fmt": "jpg",
     "why": "plywood blast board under the pad (indoor demo)"},
    {"id": "rectangular_parquet", "kind": "texture", "res": "2k", "maps": ("Diffuse", "nor_gl", "Rough"), "fmt": "jpg",
     "why": "sports-hall maple strip floor"},
    {"id": "korean_fire_extinguisher_01", "kind": "model", "res": "2k", "fmt": "gltf",
     "why": "Korean ABC fire extinguisher beside the launch area (safety cue)"},
    {"id": "plastic_crate_01", "kind": "model", "res": "1k", "fmt": "gltf",
     "why": "field-gear crate near the operator"},
]


def _get(url: str, timeout: float = 120.0) -> bytes:
    host = url.split("/")[2]
    if host not in ALLOWED_HOSTS:
        raise ValueError(f"refusing to contact {host} (allowed: {ALLOWED_HOSTS})")
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read()


def _md5(path: str) -> str:
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def plan_files(a: dict[str, Any], files: dict[str, Any]) -> list[dict[str, Any]]:
    """The (relative path, url, size, md5) list of one asset from its /files listing."""
    out: list[dict[str, Any]] = []
    if a["kind"] == "hdri":
        e = files["hdri"][a["res"]][a["fmt"]]
        out.append({"rel": os.path.basename(e["url"]), "url": e["url"], "size": e["size"], "md5": e.get("md5")})
    elif a["kind"] == "texture":
        for m in a["maps"]:
            e = files[m][a["res"]][a["fmt"]]
            out.append({"rel": os.path.basename(e["url"]), "url": e["url"], "size": e["size"], "md5": e.get("md5")})
    elif a["kind"] == "model":
        g = files["gltf"][a["res"]]["gltf"]
        out.append({"rel": os.path.basename(g["url"]), "url": g["url"], "size": g["size"], "md5": g.get("md5")})
        for rel, e in sorted(g.get("include", {}).items()):
            out.append({"rel": rel.replace("\\", "/"), "url": e["url"], "size": e["size"], "md5": e.get("md5")})
    else:
        raise ValueError(a["kind"])
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true", help="verify the files on disk, download nothing")
    ap.add_argument("--credits-only", action="store_true", help="rewrite CREDITS.md from manifest.json (offline)")
    args = ap.parse_args(argv)
    if args.credits_only:
        with open(os.path.join(ASSET_DIR, "manifest.json"), "r", encoding="utf-8") as f:
            man = json.load(f)
        write_credits(man["assets"], int(man["total_bytes"]))
        return 0
    os.makedirs(PH_DIR, exist_ok=True)
    manifest: list[dict[str, Any]] = []
    total = 0
    for a in ASSETS:
        info = json.loads(_get(f"{API}/info/{a['id']}"))
        files = json.loads(_get(f"{API}/files/{a['id']}"))
        d = os.path.join(PH_DIR, a["id"])
        os.makedirs(d, exist_ok=True)
        flist = plan_files(a, files)
        for fe in flist:
            p = os.path.join(d, *fe["rel"].split("/"))
            ok = os.path.exists(p) and os.path.getsize(p) == fe["size"] and (fe["md5"] is None or _md5(p) == fe["md5"])
            if not ok:
                if args.check:
                    print(f"MISSING/BAD {a['id']}/{fe['rel']}")
                    continue
                os.makedirs(os.path.dirname(p), exist_ok=True)
                t0 = time.time()
                blob = _get(fe["url"], timeout=600.0)
                if len(blob) != fe["size"]:
                    raise RuntimeError(f"{fe['url']}: {len(blob)} bytes, listing says {fe['size']}")
                if fe["md5"] and hashlib.md5(blob).hexdigest() != fe["md5"]:
                    raise RuntimeError(f"{fe['url']}: md5 mismatch")
                with open(p, "wb") as f:
                    f.write(blob)
                print(f"  got {a['id']}/{fe['rel']}  {len(blob) / 1e6:.1f} MB  {time.time() - t0:.1f} s", flush=True)
            total += fe["size"]
        manifest.append({"id": a["id"], "kind": a["kind"], "res": a["res"], "name": info.get("name", a["id"]),
                         "authors": sorted((info.get("authors") or {}).keys()), "license": "CC0 1.0",
                         "page": f"https://polyhaven.com/a/{a['id']}", "why": a["why"], "dimensions_mm": info.get("dimensions"),
                         "files": [{"rel": fe["rel"], "bytes": fe["size"], "md5": fe["md5"]} for fe in flist],
                         "bytes": sum(fe["size"] for fe in flist)})
        if total / 1e6 > BUDGET_MB:
            raise RuntimeError(f"asset budget exceeded: {total / 1e6:.1f} MB > {BUDGET_MB} MB")
    with open(os.path.join(ASSET_DIR, "manifest.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump({"total_bytes": total, "assets": manifest}, f, indent=1, ensure_ascii=False)
    write_credits(manifest, total)
    print(f"total {total / 1e6:.1f} MB in {len(manifest)} assets")
    return 0


KIND_KO = {"hdri": "HDRI 환경광", "texture": "PBR 텍스처", "model": "3D 모델"}
# CC0 sources not fetched by this script (listed in the credits when present on disk)
OTHER_CC0 = [
    {"path": ("render", "human", "human_rigged.blend"),
     "ko": "인체 모델: Blender Studio Human Base Meshes v1.4.1 (CC0) — 리그·포즈는 팀 제작 (render/human)",
     "en": "Human figure: Blender Studio Human Base Meshes v1.4.1 (CC0); rig and poses by the team (render/human)"},
]


def write_credits(manifest: list[dict[str, Any]], total: int) -> None:
    project = os.path.dirname(ROOT)
    other = [o for o in OTHER_CC0 if os.path.exists(os.path.join(project, *o["path"]))]
    lines = ["# Asset credits / 에셋 출처", "",
             "All third-party assets below are from Poly Haven (https://polyhaven.com) under CC0 1.0 (public domain);",
             "attribution is not required but given here.  아래 외부 에셋은 모두 Poly Haven의 CC0 1.0(퍼블릭 도메인) 자료입니다.",
             "", "Presentation one-liners / 발표 자막용 한 줄:", "",
             "- KO: 배경 HDRI·텍스처·소품: Poly Haven (CC0) — " + ", ".join(m["name"] for m in manifest),
             "- EN: HDRIs, textures and props: Poly Haven (CC0) — " + ", ".join(m["name"] for m in manifest),
             *[f"- KO: {o['ko']}" for o in other], *[f"- EN: {o['en']}" for o in other],
             "- KO: 기체 형상: 팀 CAD(CadQuery STL), 비행 데이터: 6자유도 시뮬레이션 로그, 렌더: Blender (Cycles)",
             "- EN: Vehicle geometry: team CAD (CadQuery STLs); motion: 6-DOF simulation log; rendered with Blender (Cycles)",
             "- KO: 모든 영상에는 \"시뮬레이션 (CG)\" 표기가 들어갑니다 (분석판: \"시뮬레이션 - 실제 비행 영상 아님\").",
             "- EN: Every video carries a \"시뮬레이션 (CG)\" (simulation, CG) tag; analysis versions say \"not flight footage\".",
             "", "| id | 종류 / kind | res | authors | MB | used for |", "|---|---|---|---|---|---|"]
    for m in manifest:
        lines.append(f"| [{m['id']}]({m['page']}) | {KIND_KO.get(m['kind'], m['kind'])} / {m['kind']} | {m['res']} | "
                     f"{', '.join(m['authors'])} | {m['bytes'] / 1e6:.1f} | {m['why']} |")
    lines += ["", f"Total on disk: {total / 1e6:.1f} MB (render/assets/polyhaven, git-ignored; re-create with "
              "`python render/fetch_assets.py`).", ""]
    with open(os.path.join(ASSET_DIR, "CREDITS.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    sys.exit(main())
