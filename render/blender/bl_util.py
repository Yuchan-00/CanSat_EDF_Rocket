"""Small bpy helpers shared by the scene builders (run inside Blender only).

Invariant: everything here is deterministic - no random state, no time, no uuid; meshes are built
from numpy arrays with foreach_set and keyframes are written in bulk, so the same inputs give the same
.blend data and the same pixels for a given engine / sample count.
"""
from __future__ import annotations

import math
import os
import struct
from typing import Any, Iterable, Sequence

import bpy
import numpy as np

INTERP = {"CONSTANT": 0, "LINEAR": 1, "BEZIER": 2}


# --------------------------------------------------------------------------- #
# collections / objects
# --------------------------------------------------------------------------- #
def collection(name: str, parent: Any = None) -> Any:
    c = bpy.data.collections.get(name)
    if c is None:
        c = bpy.data.collections.new(name)
        (parent or bpy.context.scene.collection).children.link(c)
    return c


def link(obj: Any, coll: Any = None) -> Any:
    (coll or bpy.context.scene.collection).objects.link(obj)
    return obj


def empty(name: str, coll: Any = None, parent: Any = None, loc: Sequence[float] = (0, 0, 0), size: float = 0.1,
          kind: str = "PLAIN_AXES") -> Any:
    o = bpy.data.objects.new(name, None)
    o.empty_display_type = kind
    o.empty_display_size = size
    o.location = loc
    if parent is not None:
        o.parent = parent
    return link(o, coll)


def mesh_object(name: str, verts: np.ndarray, faces: np.ndarray | Sequence[Sequence[int]], coll: Any = None,
                parent: Any = None, smooth_angle_deg: float | None = 30.0, mat: Any = None) -> Any:
    me = mesh_from_arrays(name, verts, faces)
    if smooth_angle_deg is not None:
        smooth(me, smooth_angle_deg)
    o = bpy.data.objects.new(name, me)
    if parent is not None:
        o.parent = parent
    if mat is not None:
        o.data.materials.append(mat)
    return link(o, coll)


def mesh_from_arrays(name: str, verts: np.ndarray, faces: Any) -> Any:
    """A mesh from (V,3) vertices and faces: an (F,k) int array (all faces k-gons) or a list of index lists."""
    verts = np.asarray(verts, dtype=np.float32).reshape(-1, 3)
    me = bpy.data.meshes.new(name)
    if isinstance(faces, np.ndarray) and faces.ndim == 2:
        fl = faces.astype(np.int32)
        sizes = np.full(fl.shape[0], fl.shape[1], dtype=np.int32)
        flat = fl.reshape(-1)
    else:
        sizes = np.array([len(f) for f in faces], dtype=np.int32)
        flat = np.array([i for f in faces for i in f], dtype=np.int32)
    starts = np.concatenate([[0], np.cumsum(sizes)[:-1]]).astype(np.int32)
    me.vertices.add(verts.shape[0])
    me.vertices.foreach_set("co", verts.reshape(-1))
    me.loops.add(flat.size)
    me.loops.foreach_set("vertex_index", flat)
    me.polygons.add(sizes.size)
    me.polygons.foreach_set("loop_start", starts)          # face sizes follow from the next loop_start (Blender >= 4)
    me.update(calc_edges=True)
    me.validate(clean_customdata=False)
    return me


def smooth(me: Any, angle_deg: float = 30.0) -> None:
    """Smooth shading with edges sharper than ``angle_deg`` kept sharp (CAD tessellation)."""
    me.polygons.foreach_set("use_smooth", np.ones(len(me.polygons), dtype=bool))
    if hasattr(me, "set_sharp_from_angle"):
        me.set_sharp_from_angle(angle=math.radians(angle_deg))
    me.update()


def read_stl(path: str, scale: float = 1e-3, weld: float = 1e-6) -> tuple[np.ndarray, np.ndarray]:
    """Binary STL -> (welded vertices (V,3) in metres, triangles (F,3)).  Degenerate triangles dropped."""
    with open(path, "rb") as f:
        raw = f.read()
    n = struct.unpack("<I", raw[80:84])[0]
    if 84 + 50 * n != len(raw):
        raise ValueError(f"{path}: not a binary STL")
    a = np.frombuffer(raw[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    v = a["v"].reshape(-1, 3).astype(np.float64) * scale
    key = np.round(v / weld).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    verts = np.zeros((uniq.shape[0], 3))
    np.add.at(verts, inv.reshape(-1), v)
    cnt = np.bincount(inv.reshape(-1), minlength=uniq.shape[0])[:, None]
    verts /= cnt
    tris = inv.reshape(-1, 3)
    ok = (tris[:, 0] != tris[:, 1]) & (tris[:, 1] != tris[:, 2]) & (tris[:, 0] != tris[:, 2])
    return verts, tris[ok]


# --------------------------------------------------------------------------- #
# keyframes in bulk
# --------------------------------------------------------------------------- #
def fcurve(idb: Any, data_path: str, index: int = 0) -> Any:
    ad = idb.animation_data or idb.animation_data_create()
    if ad.action is None:
        ad.action = bpy.data.actions.new(f"{idb.name}_act")
    act = ad.action
    if hasattr(act, "fcurve_ensure_for_datablock"):
        return act.fcurve_ensure_for_datablock(idb, data_path, index=index)
    fc = act.fcurves.find(data_path, index=index)
    return fc or act.fcurves.new(data_path, index=index)


def key_array(idb: Any, data_path: str, frames: np.ndarray, values: np.ndarray, const_frames: Iterable[int] = (),
              interp: str = "LINEAR") -> None:
    """Key ``data_path`` at ``frames`` (N,) with ``values`` (N,) or (N,k) (k array components).  Keys whose frame is
    in ``const_frames`` get CONSTANT interpolation (a cut: no blur / no blend into the next shot)."""
    frames = np.asarray(frames, dtype=np.float64)
    values = np.asarray(values, dtype=np.float64)
    if values.ndim == 1:
        values = values[:, None]
    cf = set(int(c) for c in const_frames)
    ip = np.array([INTERP["CONSTANT"] if int(f) in cf else INTERP[interp] for f in frames], dtype=np.int32)
    for j in range(values.shape[1]):
        fc = fcurve(idb, data_path, j)
        fc.keyframe_points.clear()
        fc.keyframe_points.add(frames.size)
        co = np.empty(2 * frames.size, dtype=np.float64)
        co[0::2] = frames
        co[1::2] = values[:, j]
        fc.keyframe_points.foreach_set("co", co)
        fc.keyframe_points.foreach_set("interpolation", ip)
        if hasattr(fc, "extrapolation"):
            fc.extrapolation = "CONSTANT"
        fc.update()


# --------------------------------------------------------------------------- #
# shader node helpers
# --------------------------------------------------------------------------- #
class NodeBuilder:
    """Terse node-tree building: nb.n('ShaderNodeMath', operation='MULTIPLY', inputs={1: 2.0})."""

    def __init__(self, tree: Any, clear: bool = True):
        self.tree = tree
        if clear:
            for nd in list(tree.nodes):
                tree.nodes.remove(nd)
        self.x = 0

    def n(self, kind: str, name: str | None = None, inputs: dict | None = None, **props: Any) -> Any:
        nd = self.tree.nodes.new(kind)
        if name:
            nd.name = name
            nd.label = name
        for k, v in props.items():
            setattr(nd, k, v)
        for k, v in (inputs or {}).items():
            sock = nd.inputs[k]
            sock.default_value = v
        nd.location = (self.x, 0)
        self.x += 40
        return nd

    def l(self, a: Any, b: Any) -> None:
        self.tree.links.new(a, b)

    def math(self, op: str, a: Any, b: Any = None, clamp: bool = False) -> Any:
        nd = self.n("ShaderNodeMath", operation=op, use_clamp=clamp)
        for i, v in enumerate((a, b)):
            if v is None:
                continue
            if isinstance(v, (int, float)):
                nd.inputs[i].default_value = float(v)
            else:
                self.l(v, nd.inputs[i])
        return nd.outputs[0]

    def mix_rgb(self, fac: Any, a: Any, b: Any, blend: str = "MIX") -> Any:
        nd = self.n("ShaderNodeMix", data_type="RGBA", blend_type=blend)
        for sock, v in ((nd.inputs["Factor"], fac), (nd.inputs[6], a), (nd.inputs[7], b)):
            if isinstance(v, (int, float)):
                sock.default_value = float(v)
            elif isinstance(v, tuple):
                sock.default_value = v
            else:
                self.l(v, sock)
        return nd.outputs[2]


def principled(nb: NodeBuilder, **inputs: Any) -> Any:
    p = nb.n("ShaderNodeBsdfPrincipled")
    for k, v in inputs.items():
        name = k.replace("_", " ")
        if isinstance(v, (int, float)):
            p.inputs[name].default_value = float(v)
        elif isinstance(v, tuple):
            p.inputs[name].default_value = v
        else:
            nb.l(v, p.inputs[name])
    return p


def new_material(name: str) -> tuple[Any, NodeBuilder, Any]:
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    nb = NodeBuilder(m.node_tree)
    out = nb.n("ShaderNodeOutputMaterial")
    return m, nb, out


def srgb(hexstr: str, a: float = 1.0) -> tuple[float, float, float, float]:
    """'#rrggbb' -> linear RGBA (the sRGB transfer removed), for Base Color sockets."""
    h = hexstr.lstrip("#")
    c = [int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]
    lin = [x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c]
    return (lin[0], lin[1], lin[2], a)


def image(path: str, colorspace: str = "sRGB") -> Any:
    name = os.path.basename(path)
    img = bpy.data.images.get(name)
    if img is None:
        img = bpy.data.images.load(path, check_existing=True)
    try:
        img.colorspace_settings.name = colorspace
    except TypeError:
        img.colorspace_settings.name = "Non-Color" if colorspace != "sRGB" else "sRGB"
    return img


BOX_FACES_OUT = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]]


def box_verts(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> np.ndarray:
    """8 corners, index = ix + 2 iy + 4 iz (use with BOX_FACES_OUT, outward normals)."""
    return np.array([[x, y, z] for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)], dtype=float)


def box_faces(inward: bool = False) -> list[list[int]]:
    return [f[::-1] for f in BOX_FACES_OUT] if inward else [list(f) for f in BOX_FACES_OUT]


def look_at_quat(direction: Sequence[float], up: Sequence[float] = (0, 0, 1), axis: str = "-Z") -> Any:
    from mathutils import Vector
    return Vector(direction).to_track_quat(axis, "Y" if axis in ("-Z", "Z") else "Z")
