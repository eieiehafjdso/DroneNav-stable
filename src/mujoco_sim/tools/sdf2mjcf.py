#!/usr/bin/env python3
"""SDF (Gazebo / Ignition / gz-sim) 静态世界 -> MJCF (MuJoCo)

支持: world / model / include(model://) / link / 几何(box cylinder sphere capsule
plane mesh) / 材质 / directional 与 point 灯光 / pose 逐级叠加。
不支持: 关节、传感器、Gazebo 插件(裁判系统、弹丸、装甲命中判定等逻辑
        没有 MuJoCo 对应物, 需要自己在 Python 里重写)。

用法:
  python sdf2mjcf.py rmuc_2025_world.sdf -o rmuc_2025.xml \
      --search <gz 资源目录(含 models/)> --floor

注意: MuJoCo 的碰撞网格会被简化成**凸包**, 所以大场景的网格默认只做显示,
      碰撞体请用 box/cylinder 手工拼(见生成的 XML 里的提示)。
"""
from __future__ import annotations

import argparse
import math
import os
import re
import shutil
import struct
import sys
import xml.etree.ElementTree as ET

# ---------------------------------------------------------------- 线性代数
def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mv(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def quat_from_R(R):
    t = R[0][0] + R[1][1] + R[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2.0
        return (0.25 * s, (R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s)
    if R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
        return ((R[2][1] - R[1][2]) / s, 0.25 * s, (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s)
    if R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
        return ((R[0][2] - R[2][0]) / s, (R[0][1] + R[1][0]) / s, 0.25 * s, (R[1][2] + R[2][1]) / s)
    s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
    return ((R[1][0] - R[0][1]) / s, (R[0][2] + R[2][0]) / s, (R[1][2] + R[2][1]) / s, 0.25 * s)


class Xf:
    """刚体变换: x' = R x + t"""

    def __init__(self, R=None, t=(0.0, 0.0, 0.0)):
        self.R = R or [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        self.t = tuple(float(x) for x in t)

    @staticmethod
    def pose(p):
        return Xf(rpy_matrix(*p[3:]), p[:3])

    def __matmul__(self, o):
        return Xf(mm(self.R, o.R), [a + b for a, b in zip(mv(self.R, o.t), self.t)])

    def apply(self, pt):
        return [a + b for a, b in zip(mv(self.R, pt), self.t)]


def fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


# ---------------------------------------------------------------- SDF 小工具
def num_list(elem, path, default=None):
    if elem is None:
        return default
    e = elem.find(path)
    if e is None or not (e.text or "").strip():
        return default
    return [float(x) for x in e.text.split()]


def text(elem, path, default=None):
    e = elem.find(path)
    if e is None or e.text is None:
        return default
    return e.text.strip()


def parse_pose(elem):
    if elem is None or not (elem.text or "").strip():
        return (0.0,) * 6
    v = [float(x) for x in elem.text.split()]
    if len(v) != 6:
        raise ValueError(f"<pose> 需要 6 个数, 得到 {elem.text!r}")
    if elem.get("degrees") == "true":
        v[3:] = [math.radians(x) for x in v[3:]]
    return tuple(v)


def stl_bbox(path):
    """二进制 STL 包围盒; 失败返回 None"""
    try:
        with open(path, "rb") as f:
            head = f.read(84)
            if len(head) < 84:
                return None
            if head[:5].lower() == b"solid":
                return None  # ASCII, 交给 MuJoCo 自己处理
            n = struct.unpack("<I", head[80:84])[0]
            data = f.read(n * 50)
            if len(data) != n * 50:
                return None
            lo = [math.inf] * 3
            hi = [-math.inf] * 3
            for tri in struct.iter_unpack("<12fH", data):
                for k in range(3):
                    for c in range(3):
                        v = tri[3 + 3 * k + c]
                        if v < lo[c]:
                            lo[c] = v
                        if v > hi[c]:
                            hi[c] = v
            return (lo, hi)
    except Exception:
        return None


def bbox_corners(lo, hi):
    return [[hi[0] if (i & 1) else lo[0],
             hi[1] if (i & 2) else lo[1],
             hi[2] if (i & 4) else lo[2]] for i in range(8)]


# ---------------------------------------------------------------- 转换器
class Builder:
    def __init__(self, args):
        self.args = args
        self.out_dir = os.path.dirname(os.path.abspath(args.output)) or "."
        self.meshdir = os.path.join(self.out_dir, args.meshdir)
        os.makedirs(self.meshdir, exist_ok=True)
        self.assets = []
        self.mesh_names = {}     # (path, scale) -> name
        self.materials = set()
        self.bodies = []         # (body_str, bbox)
        self.lights = []
        self.warnings = []
        self.mesh_bbox = None    # (lo, hi) 世界坐标
        self.excludes = set(args.exclude or [])
        self.mesh_worlds = []      # [(path, Xf, scale)] 用于生成碰撞体

    # ---------- 资源 ----------
    def resolve(self, uri):
        if uri.startswith("model://"):
            rest = uri[len("model://"):]
            name, _, rel = rest.partition("/")
            for d in self.args.search:
                cand = os.path.join(d, name)
                if os.path.isdir(cand):
                    return os.path.join(cand, rel) if rel else cand
            raise SystemExit(f"找不到 {uri}, 用 --search 指定包含 {name}/ 的目录")
        if uri.startswith("file://"):
            return uri[len("file://"):]
        if os.path.isabs(uri):
            return uri
        return os.path.abspath(os.path.join(os.path.dirname(self.args.world), uri))

    def model_file(self, uri):
        p = self.resolve(uri)
        if os.path.isdir(p):
            cand = os.path.join(p, "model.sdf")
            if not os.path.exists(cand):
                raise SystemExit(f"{p} 下没有 model.sdf")
            return cand
        return p

    # ---------- 资产 ----------
    def add_mesh(self, path, scale):
        ext = os.path.splitext(path)[1].lower()
        if ext == ".dae":
            path = self.dae_to_obj(path)
            ext = ".obj"
        if self.args.texture and ext == ".stl":
            path = self.stl_uv_obj(path)
            ext = ".obj"
        if ext == ".stl":
            nf = self.stl_faces(path)
            if nf > 200000:
                path = self.decimate_stl(path, 200000)
        if ext not in (".stl", ".obj"):
            raise SystemExit(
                f"MuJoCo 只支持 OBJ/STL 网格, 不支持 {ext}: {path}\n"
                f"  转换方法:  sudo apt install assimp-utils && assimp export in{ext} out.obj")
        key = (os.path.abspath(path), tuple(scale))
        if key in self.mesh_names:
            return self.mesh_names[key], stl_bbox(path) if ext == ".stl" else None
        base = re.sub(r"[^0-9A-Za-z_]", "_", os.path.splitext(os.path.basename(path))[0])
        name, i = base, 0
        while name in self.mesh_names.values():
            i += 1
            name = f"{base}_{i}"
        target = os.path.join(self.meshdir, name + ext)
        if not os.path.exists(target):
            try:
                os.symlink(os.path.abspath(path), target)
            except OSError:
                shutil.copy2(path, target)
        attrs = f'name="{name}" file="{name + ext}"'
        if tuple(scale) != (1.0, 1.0, 1.0):
            attrs += f' scale="{fmt(scale)}"'
        self.assets.append(f"    <mesh {attrs}/>")
        self.mesh_names[key] = name
        return name, (stl_bbox(path) if ext == ".stl" else None)

    @staticmethod
    def stl_faces(path):
        try:
            with open(path, "rb") as f:
                head = f.read(84)
                if len(head) < 84 or head[:5].lower() == b"solid":
                    return 0
                return struct.unpack("<I", head[80:84])[0]
        except OSError:
            return 0

    def stl_uv_obj(self, path):
        """STL 没有 UV; 用 XY 平面投影生成 vt 并导出 OBJ, 让 MuJoCo 能贴纹理"""
        out = os.path.join(self.meshdir, os.path.splitext(os.path.basename(path))[0] + "_uv.obj")
        if os.path.exists(out):
            return out
        try:
            import numpy as np
            import trimesh
            mesh = trimesh.load(path)
            v = np.asarray(mesh.vertices)
            lo, hi = v[:, :2].min(0), v[:, :2].max(0)
            span = np.maximum(hi - lo, 1e-9)
            uv = (v[:, :2] - lo) / span
            mesh.visual = trimesh.visual.TextureVisuals(uv=uv.astype(np.float64))
            # 关键：不合并顶点，否则 MuJoCo 会做平滑法线，模型看起来像"融化"了
            mesh.unmerge_vertices()
            mesh.export(out)
        except Exception as e:
            raise SystemExit(f"生成纹理坐标失败: {e}")
        print(f"  [UV] {os.path.basename(path)} -> {os.path.basename(out)}")
        return out

    def decimate_stl(self, path, target):
        """MuJoCo 单个网格上限 20 万三角面, 超过会用 trimesh 抽稀"""
        out = os.path.join(self.meshdir, os.path.splitext(os.path.basename(path))[0] + f"_dec{target}.stl")
        if os.path.exists(out):
            return out
        try:
            import trimesh
            import fast_simplification
            mesh = trimesh.load(path)
            nf = len(mesh.faces)
            v, f = fast_simplification.simplify(
                mesh.vertices.astype("float32"), mesh.faces.astype("int32"),
                target_reduction=max(0.0, min(0.99, 1.0 - target / nf)))
            mesh = trimesh.Trimesh(v, f, process=False)
            mesh.export(out)
        except Exception as e:
            raise SystemExit(
                f"网格 {os.path.basename(path)} 有 {self.stl_faces(path)} 个三角面, "
                f"超过 MuJoCo 上限 200000, 自动抽稀失败: {e}\n"
                f"  手动处理: pip install fast-simplification trimesh, 或在 Blender/MeshLab 里减面")
        print(f"  [抽稀] {os.path.basename(path)}: {self.stl_faces(path)} -> {len(mesh.faces)} 面")
        return out

    def dae_to_obj(self, path):
        """MuJoCo 不支持 DAE, 用 trimesh(+pycollada) 转成 OBJ"""
        out = os.path.join(self.meshdir,
                           re.sub(r"[^0-9A-Za-z_]", "_", os.path.splitext(os.path.basename(path))[0]) + ".obj")
        try:
            import trimesh
            scene = trimesh.load(path, force="scene")
            scene.export(out)
        except Exception as e:
            raise SystemExit(f"DAE 转 OBJ 失败: {e}\n  pip install pycollada 后再试, 或 "
                             f"sudo apt install assimp-utils && assimp export {path} out.obj")
        print(f"  [转换] DAE -> OBJ: {os.path.basename(out)}")
        return out

    def split_parts(self, path, world, scale, base_name):
        """按法线和半场把网格拆成 floor_red / floor_blue / wall / slope 四块"""
        import numpy as np
        import trimesh

        mesh = trimesh.load(path, force="mesh", process=False)
        v = np.asarray(mesh.vertices) * np.asarray(scale)
        R, t = np.asarray(world.R), np.asarray(world.t)
        vw = v @ R.T + t
        fn = np.asarray(mesh.face_normals) @ R.T
        tri = vw[np.asarray(mesh.faces)]
        cz = tri[:, :, 2].mean(1)
        cy = tri[:, :, 1].mean(1)
        nz = np.abs(fn[:, 2])

        masks = {
            "wall": nz < 0.5,
            "slope": (nz >= 0.5) & (nz < 0.92),
            "floor_red": (nz >= 0.92) & (cy < 0),
            "floor_blue": (nz >= 0.92) & (cy >= 0),
        }
        colors = {
            "floor_red": "1.0 0.58 0.55 1",
            "floor_blue": "0.58 0.68 1.0 1",
            "wall": "0.78 0.78 0.80 1",
            "slope": "0.90 0.90 0.92 1",
        }
        # STL 没有 UV: 用局部 XY 平面投影生成, 归一化到包围盒
        vloc = np.asarray(mesh.vertices)
        lo_v, hi_v = vloc[:, :2].min(0), vloc[:, :2].max(0)
        span_v = np.maximum(hi_v - lo_v, 1e-9)
        uv = (vloc[:, :2] - lo_v) / span_v

        out = []
        for pname, mask in masks.items():
            if not mask.any():
                continue
            sub = mesh.submesh([np.where(mask)[0]], append=True) if hasattr(mesh, "submesh") else None
            if sub is None:
                continue
            sub = trimesh.Trimesh(vertices=np.asarray(mesh.vertices),
                                  faces=np.asarray(mesh.faces)[mask], process=False)
            sub.visual = trimesh.visual.TextureVisuals(uv=uv)
            fname = f"{re.sub(r'[^0-9A-Za-z_]', '_', base_name)}_{pname}.obj"
            fpath = os.path.join(self.meshdir, fname)
            sub.unmerge_vertices()
            sub.export(fpath)
            mname = fname[:-4]
            self.assets.append(f'    <mesh name="{mname}" file="{fname}"/>')
            tex = ' texture="field_tex"' if self.args.texture else ''
            self.assets.append(f'    <material name="part_{pname}"{tex} rgba="{colors[pname]}" '
                               f'specular="0.08" shininess="0.2"/>')
            out.append((pname, mname))
        self.mesh_names[(os.path.abspath(path), tuple(scale), "parts")] = [m for _, m in out]
        return out

    def material(self, mat):
        if mat is None:
            return None
        diffuse = num_list(mat, "diffuse", [0.7, 0.7, 0.7, 1.0])
        specular = num_list(mat, "specular", [0.0, 0.0, 0.0, 1.0])
        if len(diffuse) == 3:
            diffuse = diffuse + [1.0]
        name = mat.get("name") or "mat_" + """_""".join(f"{x:.3g}".replace(".", "p").replace("-", "m") for x in diffuse)
        if name in self.materials:
            return name
        self.assets.append(f'    <material name="{name}" rgba="{fmt(diffuse)}" '
                           f'specular="{specular[0]:.3g}" shininess="0.3"/>')
        self.materials.add(name)
        return name

    # ---------- 几何 ----------
    def geom(self, geo, rel, world, mat, collision, name):
        tag = geo.tag
        attrs = f'name="{name}" pos="{fmt(rel.t)}" quat="{fmt(quat_from_R(rel.R))}"'
        if collision:
            attrs += ' group="3"'
        else:
            attrs += ' contype="0" conaffinity="0" group="2"'
        if mat and not collision:
            attrs += f' material="{mat}"'
        if tag == "box":
            s = num_list(geo, "size", [0.5, 0.5, 0.5])
            return f'      <geom type="box" size="{fmt([x / 2 for x in s])}" {attrs}/>'
        if tag == "cylinder":
            r, h = float(text(geo, "radius")), float(text(geo, "length"))
            return f'      <geom type="cylinder" size="{r:.6g} {h / 2:.6g}" {attrs}/>'
        if tag == "sphere":
            r = float(text(geo, "radius"))
            return f'      <geom type="sphere" size="{r:.6g}" {attrs}/>'
        if tag == "capsule":
            r, h = float(text(geo, "radius")), float(text(geo, "length"))
            return f'      <geom type="capsule" size="{r:.6g} {h / 2:.6g}" {attrs}/>'
        if tag == "plane":
            s = num_list(geo, "size", [50.0, 50.0])
            if self.args.checker_ground:
                attrs = attrs.replace('material="mat_0p5_0p5_0p5_1"', 'material="checker_mat"')
            return f'      <geom type="plane" size="{s[0]:.6g} {s[1]:.6g} 0.1" {attrs}/>'
        if tag == "mesh" and self.args.part_colors and not collision:
            uri = text(geo, "uri")
            scale = num_list(geo, "scale", [1.0, 1.0, 1.0])
            if len(scale) == 1:
                scale = scale * 3
            path = self.resolve(uri)
            parts = self.split_parts(path, world, scale, name)
            self.mesh_worlds.append((path, world, scale))
            out = []
            for pname, mesh_name in parts:
                out.append(f'      <geom type="mesh" mesh="{mesh_name}" pos="{fmt(rel.t)}" '
                           f'quat="{fmt(quat_from_R(rel.R))}" contype="0" conaffinity="0" '
                           f'group="2" material="part_{pname}"/>')
            return "\n".join(out)
        if tag == "mesh":
            if collision and self.args.mesh_collision == "none":
                return None          # 不做网格碰撞, 也不必生成 UV 副本
            uri = text(geo, "uri")
            scale = num_list(geo, "scale", [1.0, 1.0, 1.0])
            if len(scale) == 1:
                scale = scale * 3
            path = self.resolve(uri)
            mesh_name, bb = self.add_mesh(path, scale)
            if bb:
                pts = [world.apply([c[i] * scale[i] for i in range(3)]) for c in bbox_corners(*bb)]
                lo = [min(p[i] for p in pts) for i in range(3)]
                hi = [max(p[i] for p in pts) for i in range(3)]
                if self.mesh_bbox is None:
                    self.mesh_bbox = (lo, hi)
                else:
                    self.mesh_bbox = ([min(a, b) for a, b in zip(self.mesh_bbox[0], lo)],
                                      [max(a, b) for a, b in zip(self.mesh_bbox[1], hi)])
            if not collision:
                self.mesh_worlds.append((path, world, scale))
            if not collision and self.args.texture:
                mat = "field_mat"
            if collision and self.args.mesh_collision == "none":
                return None  # 网格不做碰撞, 由调用方补 box
            return f'      <geom type="mesh" mesh="{mesh_name}" {attrs}/>'
        raise SystemExit(f"不支持的几何类型 <{tag}>")

    # ---------- 模型 ----------
    def model(self, src, xf, depth=0, name=None):
        if depth > 8:
            raise SystemExit(f"include 嵌套过深: {src}")
        if isinstance(src, ET.Element):
            root, tag = src, "<inline>"
        else:
            root, tag = ET.parse(src).getroot(), src
            if root.tag == "sdf":
                root = root.find("model")
            if root is None or root.tag != "model":
                raise SystemExit(f"{tag} 里没有 <model>")
        mxf = xf @ Xf.pose(parse_pose(root.find("pose")))
        mname = name or root.get("name") or os.path.basename(os.path.dirname(str(tag)))
        if mname in self.excludes:
            print(f"  [跳过] {mname}")
            return
        static = text(root, "static", "false") == "true"
        if not static:
            self.warnings.append(f"模型 {mname} 不是 static=true, 这里按静态处理(不生成关节)")

        # 模型级 include / 嵌套 model
        for inc in root.findall("include"):
            self.model(self.model_file(text(inc, "uri")),
                       mxf @ Xf.pose(parse_pose(inc.find("pose"))), depth + 1,
                       name=text(inc, "name"))

        for link in root.findall("link"):
            lname = link.get("name") or "link"
            lxf = mxf @ Xf.pose(parse_pose(link.find("pose")))
            items, n = [], 0
            for vis in link.findall("visual"):
                g = vis.find("geometry")
                if g is None or len(g) == 0:
                    continue
                n += 1
                rel = Xf.pose(parse_pose(vis.find("pose")))
                s = self.geom(g[0], rel, lxf @ rel, self.material(vis.find("material")),
                              False, f"{mname}_{lname}_v{n}")
                if s:
                    items.append(s)
            for col in link.findall("collision"):
                g = col.find("geometry")
                if g is None or len(g) == 0:
                    continue
                n += 1
                rel = Xf.pose(parse_pose(col.find("pose")))
                s = self.geom(g[0], rel, lxf @ rel, None, True, f"{mname}_{lname}_c{n}")
                if s:
                    items.append(s)
            if link.findall("joint"):
                self.warnings.append(f"{mname}/{lname} 含 joint, 已忽略(固定连接)")
            if items:
                self.bodies.append(
                    f'    <body name="{mname}_{lname}" pos="{fmt(lxf.t)}" '
                    f'quat="{fmt(quat_from_R(lxf.R))}">\n' + "\n".join(items) + "\n    </body>")

    # ---------- 世界 ----------
    def world(self, path):
        root = ET.parse(path).getroot()
        if root.tag == "sdf":
            root = root.find("world")
            if root is None:
                raise SystemExit(f"{path} 里没有 <world>")
        for light in root.findall("light"):
            self.light(light)
        for child in list(root):
            if child.tag == "include":
                self.model(self.model_file(text(child, "uri")),
                           Xf.pose(parse_pose(child.find("pose"))), name=text(child, "name"))
            elif child.tag == "model":
                self.model(child, Xf())  # pose 在 model() 内部读取, 这里不能再套一次

    def light(self, light):
        ltype = light.get("type", "point")
        name = light.get("name", "light")
        pose = parse_pose(light.find("pose"))
        diff = num_list(light, "diffuse", [1, 1, 1, 1])[:3]
        if ltype == "directional":
            d = num_list(light, "direction", [0, 0, -1])
            diff = [min(1.0, c * self.args.sun_power) for c in diff]
            self.lights.append(
                f'    <light name="{name}" type="directional" dir="{fmt(d)}" '
                f'diffuse="{fmt(diff)}" specular="0.2 0.2 0.2" '
                f'castshadow="{"false" if self.args.no_shadow else "true"}"/>')
        else:
            att = light.find("attenuation")
            rng = float(text(att, "range", "10")) if att is not None else 10.0
            self.lights.append(
                f'    <light name="{name}" type="point" pos="{fmt(pose[:3])}" dir="0 0 -1" '
                f'diffuse="{fmt(diff)}" range="{rng:.6g}" specular="0.2 0.2 0.2"/>')

    # ---------- 输出 ----------
    def collision_boxes(self):
        """把网格栅格化成高度图, 再用贪心合并成实体 box(2.5D 柱体)。

        适用于赛场这种"地面 + 台阶 + 挡板"结构; 有悬空/洞穴时会退化成实心。
        """
        try:
            import numpy as np
            import trimesh
        except ImportError as e:
            raise SystemExit(f"生成碰撞体需要 numpy/trimesh: {e}")

        res = self.args.collision_res
        tris_all = []
        for path, xf, scale in self.mesh_worlds:
            mesh = trimesh.load(path, force="mesh")
            v = np.asarray(mesh.vertices) * np.asarray(scale)
            R, t = np.asarray(xf.R), np.asarray(xf.t)
            v = v @ R.T + t
            tris_all.append(v[np.asarray(mesh.faces)])
        if not tris_all:
            return []
        tris = np.concatenate(tris_all)
        lo = tris.reshape(-1, 3).min(0)
        hi = tris.reshape(-1, 3).max(0)

        nx = int(np.ceil((hi[0] - lo[0]) / res)) + 1
        ny = int(np.ceil((hi[1] - lo[1]) / res)) + 1
        if nx * ny > 4_000_000:
            raise SystemExit(f"栅格太大({nx}x{ny}), 请把 --collision-res 调大")
        H = np.zeros((ny, nx), np.float32)

        for tri in tris:
            xmin, ymin = tri[:, 0].min(), tri[:, 1].min()
            xmax, ymax = tri[:, 0].max(), tri[:, 1].max()
            i0 = max(0, int((xmin - lo[0]) / res)); i1 = min(nx - 1, int((xmax - lo[0]) / res))
            j0 = max(0, int((ymin - lo[1]) / res)); j1 = min(ny - 1, int((ymax - lo[1]) / res))
            if i1 < i0 or j1 < j0:
                continue
            gx, gy = np.meshgrid(lo[0] + np.arange(i0, i1 + 1) * res,
                                 lo[1] + np.arange(j0, j1 + 1) * res)
            # 重心坐标判断点是否在三角形内
            (x1, y1), (x2, y2), (x3, y3) = tri[0, :2], tri[1, :2], tri[2, :2]
            det = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3)
            if abs(det) < 1e-12:
                continue
            l1 = ((y2 - y3) * (gx - x3) + (x3 - x2) * (gy - y3)) / det
            l2 = ((y3 - y1) * (gx - x3) + (x1 - x3) * (gy - y3)) / det
            l3 = 1.0 - l1 - l2
            inside = (l1 >= -1e-6) & (l2 >= -1e-6) & (l3 >= -1e-6)
            if not inside.any():
                continue
            z = l1 * tri[0, 2] + l2 * tri[1, 2] + l3 * tri[2, 2]
            block = H[j0:j1 + 1, i0:i1 + 1]
            np.maximum(block, np.where(inside, z, -1e9), out=block)

        H -= lo[2]                       # 相对最低点
        H = np.round(H / 0.01) * 0.01    # 量化到 1 cm, 避免浮点噪声导致合并不上
        H = np.where(H < self.args.collision_res * 0.5, 0.0, H)

        # 贪心合并成矩形柱
        used = np.zeros_like(H, dtype=bool)
        boxes = []
        tol = 0.02
        for j in range(ny):
            for i in range(nx):
                h = H[j, i]
                if used[j, i] or h <= 0.0:
                    continue
                w = 1
                while i + w < nx and not used[j, i + w] and abs(H[j, i + w] - h) < tol:
                    w += 1
                d, ok = 1, True
                while j + d < ny and ok:
                    for k in range(i, i + w):
                        if used[j + d, k] or abs(H[j + d, k] - h) >= tol:
                            ok = False
                            break
                    if ok:
                        d += 1
                used[j:j + d, i:i + w] = True
                boxes.append((i, j, w, d, h))

        out = [f'    <!-- 自动生成的碰撞体: 栅格 {res} m, {len(boxes)} 个 box -->']
        for i, j, w, d, h in boxes:
            cx = lo[0] + (i + w / 2.0) * res
            cy = lo[1] + (j + d / 2.0) * res
            cz = lo[2] + h / 2.0
            out.append(
                f'    <geom type="box" pos="{cx:.4f} {cy:.4f} {cz:.4f}" '
                f'size="{w * res / 2:.4f} {d * res / 2:.4f} {h / 2:.4f}" group="3" '
                f'contype="1" conaffinity="1" condim="3" friction="0.9 0.008 0.0001" '
                f'solref="0.004 1" solimp="0.9 0.95 0.001" rgba="0.4 0.4 0.45 1"/>')
        print(f"  [碰撞] 栅格 {res} m -> {len(boxes)} 个 box")
        return out

    def center_light(self):
        """场地正中高处一盏向下聚光灯, 并放一个可见的小球当"灯"本身"""
        h = self.args.center_light
        if h <= 0:
            return []
        p = self.args.center_light_power
        out = [
            f'    <light name="center_lamp" type="spot" pos="0 0 {h:.2f}" dir="0 0 -1" '
            f'cutoff="85" exponent="0.8" range="{h * 3.0:.1f}" '
            f'diffuse="{p:.3f} {p * 0.97:.3f} {p * 0.92:.3f}" '
            f'specular="0.05 0.05 0.05" castshadow="{"false" if self.args.no_shadow else "true"}"/>',
            f'    <geom name="center_lamp_ball" type="sphere" pos="0 0 {h:.2f}" '
            f'size="0.18" contype="0" conaffinity="0" group="2" '
            f'material="lamp_mat"/>',
        ]
        return out

    def team_lights(self):
        """红方/蓝方各一盏低位点光, 模拟赛场队伍色氛围光"""
        p = self.args.team_lights
        if p <= 0:
            return []
        r = p
        return [
            f'    <light name="team_red" type="point" pos="0 -11 1.2" dir="0 0 -1" '
            f'diffuse="{r:.3f} {r * 0.25:.3f} {r * 0.22:.3f}" range="14" castshadow="false"/>',
            f'    <light name="team_blue" type="point" pos="0 11 1.2" dir="0 0 -1" '
            f'diffuse="{r * 0.25:.3f} {r * 0.35:.3f} {r:.3f}" range="14" castshadow="false"/>',
        ]

    def arena_lights(self):
        """在场地四周上方布 N 盏聚光灯, 冷暖交替, 制造真实赛场的强反光与硬阴影"""
        n = self.args.arena_lights
        if n <= 0:
            return []
        out = []
        warm = ("1.0 0.93 0.80", "0.95 0.97 1.00")
        for k in range(n):
            ang = 2.0 * math.pi * k / n
            px, py = 16.0 * math.cos(ang), 10.0 * math.sin(ang)
            pz = 11.0
            tgt = (0.0, 0.0, 0.8)
            d = [tgt[i] - (px, py, pz)[i] for i in range(3)]
            ln = math.sqrt(sum(x * x for x in d)) or 1.0
            d = [x / ln for x in d]
            p = self.args.arena_light_power
            rgb = (1.0, 0.96, 0.90) if k % 2 == 0 else (0.94, 0.97, 1.0)
            col = f"{rgb[0] * p:.3f} {rgb[1] * p:.3f} {rgb[2] * p:.3f}"
            out.append(
                f'    <light name="arena_spot_{k}" type="spot" '
                f'pos="{px:.2f} {py:.2f} {pz:.2f}" dir="{fmt(d)}" '
                f'cutoff="55" exponent="0.4" range="32" '
                f'diffuse="{col}" specular="0.05 0.05 0.05" '
                f'castshadow="{"true" if self.args.spot_shadows else "false"}"/>')
        return out

    def floor_geom(self):
        if not self.args.floor:
            return None
        if self.mesh_bbox:
            lo, hi = self.mesh_bbox
        else:
            lo, hi = [-20, -12, 0], [20, 12, 0]
        cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
        hx, hy = (hi[0] - lo[0]) / 2 + 1.0, (hi[1] - lo[1]) / 2 + 1.0
        z = lo[2] - 0.002
        return (f'    <geom name="floor" type="plane" pos="{cx:.6g} {cy:.6g} {z:.6g}" '
                f'size="{hx:.6g} {hy:.6g} 0.1" rgba="0.35 0.35 0.38 1" condim="3" '
                f'friction="1 0.005 0.0001"/>\n'
                f'    <!-- 四周档板/台阶等碰撞体请自己加, 例如:\n'
                f'         <geom type="box" pos="... " size="hx hy hz"/> -->')

    def write(self, name):
        parts = [f'<!-- 由 {os.path.basename(self.args.world)} 自动转换 (sdf2mjcf.py);',
                 '     网格只做显示, 碰撞体需按注释手工补 -->',
                 f'<mujoco model="{name}">',
                 '  <option timestep="0.002" gravity="0 0 -9.81" cone="elliptic" impratio="8"/>',
                 f'  <compiler angle="radian" meshdir="{self.args.meshdir}" autolimits="true"/>',
                 '  <visual>',
                 '    <global offwidth="1600" offheight="900"/>',
                 '    <headlight ambient="0.45 0.45 0.47" diffuse="0.40 0.40 0.40" specular="0.05 0.05 0.05"/>',
                 f'    <quality shadowsize="{self.args.shadowsize}" offsamples="4"/>']
        if self.args.fog:
            parts.append('    <map fogstart="30" fogend="120"/>')  # 这个版本的 MuJoCo 不支持 fogcolor
        parts += ['  </visual>', '  <asset>']
        if self.args.dark_sky:
            parts.append('    <texture name="sky" type="skybox" builtin="gradient" '
                         'rgb1="0.02 0.03 0.05" rgb2="0.07 0.09 0.14" width="512" height="256"/>')
        else:
            parts.append('    <texture name="sky" type="skybox" builtin="gradient" '
                         'rgb1="0.32 0.38 0.50" rgb2="0.86 0.88 0.92" width="512" height="256"/>')
        if self.args.checker_ground:
            parts.append('    <texture name="checker_tex" type="2d" builtin="checker" '
                         'rgb1="0.05 0.05 0.06" rgb2="0.62 0.62 0.65" width="64" height="64"/>')
            parts.append(f'    <material name="checker_mat" texture="checker_tex" '
                         f'texrepeat="{100.0 / self.args.checker_ground:.0f} '
                         f'{100.0 / self.args.checker_ground:.0f}" texuniform="false" '
                         f'specular="0.05" shininess="0.1"/>')
        if self.args.center_light > 0:
            parts.append('    <material name="lamp_mat" rgba="1 0.98 0.9 1" '
                         'emission="0.85" specular="0.2" shininess="0.8"/>')
        if self.args.texture:
            rel = os.path.relpath(os.path.abspath(self.args.texture), self.out_dir).replace(os.sep, "/")
            parts.append(f'    <texture name="field_tex" type="2d" file="{rel}"/>')
            parts.append('    <material name="field_mat" texture="field_tex" rgba="1 1 1 1" '
                         'specular="0.12" shininess="0.15"/>')
        parts += ['']
        parts += self.assets
        parts += ['  </asset>', '', '  <worldbody>']
        parts += self.lights
        parts += self.arena_lights()
        parts += self.center_light()
        parts += self.team_lights()
        fl = self.floor_geom()
        if fl:
            parts.append(fl)
        parts += self.bodies
        if self.args.collision == "boxes":
            parts += self.collision_boxes()
        parts += ['  </worldbody>']
        if self.mesh_bbox:
            lo, hi = self.mesh_bbox
            c = [(lo[i] + hi[i]) / 2 for i in range(3)]
            e = max(hi[i] - lo[i] for i in range(3))
            parts.append(f'  <statistic center="{fmt(c)}" extent="{e:.6g}"/>')
        parts.append('</mujoco>')
        with open(self.args.output, "w", encoding="utf-8") as f:
            f.write("\n".join(parts) + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description="Gazebo SDF 静态世界 -> MuJoCo MJCF")
    ap.add_argument("world")
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--search", action="append", default=[],
                    help="model:// 搜索目录, 可多次; 默认用世界文件所在目录")
    ap.add_argument("--meshdir", default="meshes")
    ap.add_argument("--floor", action="store_true", help="按网格包围盒自动加一块地面")
    ap.add_argument("--mesh-collision", choices=["none", "hull"], default="none",
                    help="none=网格只显示(推荐); hull=交给 MuJoCo 凸包碰撞(只适合小物体)")
    ap.add_argument("--exclude", action="append", default=[], metavar="NAME",
                    help="按名字丢掉某个 model/include, 可多次")
    ap.add_argument("--texture", metavar="PNG",
                    help="给网格铺一张平面投影纹理(STL 没 UV, 会自动转成带 vt 的 OBJ)")
    ap.add_argument("--arena-lights", type=int, default=0, metavar="N",
                    help="在场地周围加 N 盏聚光灯(模拟赛场灯, 冷/暖交替)")
    ap.add_argument("--fog", action="store_true", help="加体积雾(干扰视觉/SLAM 用)")
    ap.add_argument("--part-colors", action="store_true",
                    help="把场地网格按朝向/半场拆成几块, 分别给不同颜色(红蓝半场/墙体/斜坡)")
    ap.add_argument("--checker-ground", type=float, default=0.0, metavar="M",
                    help="场地外用黑白棋盘格地面, 参数是格子边长(米), 0=关闭")
    ap.add_argument("--dark-sky", action="store_true", help="深蓝渐变天空(棋盘格场景标配)")
    ap.add_argument("--team-lights", type=float, default=0.0, metavar="P",
                    help="红/蓝方各加一盏低强度氛围灯(模拟赛场队伍色光), P 为亮度")
    ap.add_argument("--center-light", type=float, default=0.0, metavar="H",
                    help="在场地中心高度 H 处放一盏向下聚光灯 + 可见灯球, 0=关闭")
    ap.add_argument("--center-light-power", type=float, default=1.0, metavar="P",
                    help="中心灯亮度, 默认 1.0")
    ap.add_argument("--arena-light-power", type=float, default=0.32, metavar="P",
                    help="每盏赛场聚光灯的亮度(默认 0.32, 1.0 会过曝)")
    ap.add_argument("--sun-power", type=float, default=1.0, metavar="S",
                    help="方向光(太阳)亮度倍率, 默认 1.0; 觉得整体偏亮就调 0.6")
    ap.add_argument("--no-shadow", action="store_true",
                    help="关掉所有 castshadow（细杆的阴影在 2k 阴影图上会糊成锯齿黑块）")
    ap.add_argument("--shadowsize", type=int, default=2048,
                    help="阴影贴图分辨率, 默认 2048; 8192 在核显上会非常卡")
    ap.add_argument("--collision", choices=["none", "boxes"], default="none",
                    help="boxes=把场地网格切成实体 box 碰撞体(墙面/台阶/坡道都能挡)")
    ap.add_argument("--collision-res", type=float, default=0.10, metavar="M",
                    help="切 box 的栅格精度, 默认 0.10 m; 0.05 更准但 geom 更多")
    ap.add_argument("--spot-shadows", action="store_true",
                    help="让聚光灯也投影(默认只有太阳投影, 省一大半渲染开销)")
    args = ap.parse_args(argv)
    args.search = [os.path.abspath(s) for s in
                   (args.search or [os.path.dirname(os.path.abspath(args.world))])]
    b = Builder(args)
    b.world(args.world)
    b.write(os.path.splitext(os.path.basename(args.world))[0])
    print(f"生成: {args.output}")
    print(f"  body: {len(b.bodies)}   light: {len(b.lights)}   mesh: {len(b.mesh_names)}")
    if b.mesh_bbox:
        lo, hi = b.mesh_bbox
        print(f"  场景包围盒: {[round(x, 2) for x in lo]} -> {[round(x, 2) for x in hi]}")
    for w in dict.fromkeys(b.warnings):
        print(f"  [注意] {w}")


if __name__ == "__main__":
    main()
