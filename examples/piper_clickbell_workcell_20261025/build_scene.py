"""Piper 双臂 click_bell 工作台：从 INTACT 现行配准生成独立场景 USD 和 scene package。

输入只用 inputs/ 下的文件（来自 INTACT-data-generation profiles/piper_real_aligned_20261022，
哈希记录在 registration.json）。几何与 INTACT real_scene.apply_scene 在统一转平（unified r5）
后的结果一致：桌面、纸贴板 A、QR 纸、头部相机；铃用 85×85×45 mm 代理体，正式资产按 asset_id 替换。

用法：python build_scene.py --out scene   （输出目录必须不存在）
"""
import argparse
import hashlib
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

HERE = Path(__file__).resolve().parent
FACES = [[0, 1, 2, 3], [4, 7, 6, 5], [0, 4, 5, 1], [1, 5, 6, 2], [2, 6, 7, 3], [3, 7, 4, 0]]
GL_TO_CV = np.diag([1., -1., -1., 1.])


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def leveled_transform(reg):
    """level_matrix @ to_nominal，再投影成严格刚体（与 INTACT unified_chain.level_profile 相同）。"""
    t = np.asarray(reg["level_matrix"], float) @ np.asarray(reg["to_nominal"], float)
    u, _, vh = np.linalg.svd(t[:3, :3])
    rigid = u @ vh
    if np.max(abs(rigid - t[:3, :3])) > 1e-6 or np.linalg.det(rigid) < 0:
        raise ValueError("配准不是刚体")
    t[:3, :3] = rigid
    return t


def apply(points, t):
    points = np.asarray(points, float)
    return points @ t[:3, :3].T + t[:3, 3]


def material(stage, name, color, texture=None):
    path = "/World/Looks/" + name
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.85)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    if texture:
        uv = UsdShade.Shader.Define(stage, path + "/UV")
        uv.CreateIdAttr("UsdPrimvarReader_float2")
        uv.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tex = UsdShade.Shader.Define(stage, path + "/Texture")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("./" + texture))
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(), "result")
        shader.GetInput("diffuseColor").ConnectToSource(tex.ConnectableAPI(), "rgb")
    return mat


def mesh(stage, path, points, faces, mat=None, collision=False, uv=False):
    obj = UsdGeom.Mesh.Define(stage, path)
    obj.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in points])
    obj.CreateFaceVertexCountsAttr([len(f) for f in faces])
    obj.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    obj.CreateSubdivisionSchemeAttr("none")
    obj.CreateDoubleSidedAttr(True)
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
    if uv:
        UsdGeom.PrimvarsAPI(obj).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                               UsdGeom.Tokens.vertex).Set([(0., 1.), (1., 1.), (1., 0.), (0., 0.)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(obj.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(obj.GetPrim()).CreateApproximationAttr("convexHull")
    return obj


def qr_points(layer_path):
    layer = Usd.Stage.Open(str(layer_path))
    prim = layer.GetPrimAtPath("/World/ReplayScene/QRSheet")
    return np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), float)


def build(out):
    inputs = HERE / "inputs"
    reg = json.loads((inputs / "registration.json").read_text(encoding="utf-8"))
    for name, digest in reg["source"]["files"].items():
        if (inputs / name).exists() and sha256(inputs / name) != digest:
            raise ValueError("输入与 INTACT 源文件不一致：" + name)
    scene = reg["scene"]
    t = leveled_transform(reg)
    z = scene["table_z_board_m"]

    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    for name in ("board_A.png", scene["qr_texture"]):
        shutil.copy2(inputs / name, out / name)
    stage = Usd.Stage.CreateNew(str(out / "workcell.usda"))
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    world.GetPrim().SetCustomDataByKey("frame", "PhysX native world, leveled (unified r5)")
    UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(1000.)

    # 桌面：配准的 1.8 m×0.805 m 桌面，30 mm 厚；板 A 系 +z 指向桌内
    top = np.asarray(scene["table_top_board_m"], float)
    box = np.vstack([top, top + [0, 0, scene["table_thickness_m"]]])
    table = apply(box, t)
    mesh(stage, "/World/Workcell/Table", table, FACES, material(stage, "Table", [.73, .72, .69]))
    mesh(stage, "/World/Colliders/Table", table, FACES, collision=True)

    # 板 A：纸贴在桌面上，物理高度 0，不单独建碰撞
    w, h = scene["board_A_size_m"]
    board = np.asarray([[0, 0, z - .0001], [w, 0, z - .0001], [w, h, z - .0001], [0, h, z - .0001]])
    mesh(stage, "/World/Workcell/BoardA", apply(board, t), [[0, 1, 2, 3]],
         material(stage, "BoardA", [1, 1, 1], "board_A.png"), uv=True)
    b = .012
    paper = np.asarray([[-b, -b, z - .00005], [w + b, -b, z - .00005], [w + b, h + b, z - .00005], [-b, h + b, z - .00005]])
    mesh(stage, "/World/Workcell/PaperA", apply(paper, t), [[0, 1, 2, 3]], material(stage, "PaperA", [.94, .94, .92]))

    # QR 纸：scene_refine v2 夜间层的配准四角，贴平到桌面
    qr = qr_points(inputs / "scene_night.usda") + [0, 0, z - .00015 - .007]
    mesh(stage, "/World/Workcell/QRSheet", apply(qr, t), [[0, 1, 2, 3]],
         material(stage, "QRSheet", [1, 1, 1], scene["qr_texture"]), uv=True)

    # 铃：真机原位，85×85×45 mm 代理体；正式资产 asset_id 见 scene package
    bx, by = reg["bell_anchor_board_m"]
    dx, dy, dz = scene["bell_dimensions_m"]
    bell = np.asarray([[bx - dx / 2, by - dy / 2, z], [bx + dx / 2, by - dy / 2, z],
                       [bx + dx / 2, by + dy / 2, z], [bx - dx / 2, by + dy / 2, z]])
    bell = np.vstack([bell, bell - [0, 0, dz]])
    proxy = mesh(stage, "/World/Objects/BellProxy", apply(bell, t), FACES,
                 material(stage, "Bell", [.75, .62, .25]), collision=True)
    proxy.GetPrim().SetCustomDataByKey("replace_with_asset_id", scene["bell_asset_id"])

    # 头部相机：authored 外参（GL 约定），随整台转平
    cam = reg["head_camera"]
    level = np.asarray(reg["level_matrix"], float)
    T = level @ np.asarray(cam["T_world_camera_gl"], float)
    c = UsdGeom.Camera.Define(stage, "/World/Cameras/head")
    UsdGeom.Xformable(c).AddTransformOp().Set(Gf.Matrix4d(T.T.tolist()))
    c.CreateFocalLengthAttr(cam["focal_length"])
    c.CreateHorizontalApertureAttr(cam["h_aperture"])
    c.CreateVerticalApertureAttr(cam["v_aperture"])
    c.CreateHorizontalApertureOffsetAttr(cam["h_offset"])
    c.CreateVerticalApertureOffsetAttr(cam["v_offset"])
    c.CreateClippingRangeAttr(Gf.Vec2f(.01, 10.))
    stage.GetRootLayer().Save()

    files = [{"path": p.name, "sha256": sha256(p), "bytes": p.stat().st_size,
              "role": "root_usd" if p.suffix == ".usda" else "texture"} for p in sorted(out.iterdir())]
    manifest = {
        "schema": "rp-real2sim2real/scene-package/1",
        "id": "piper_clickbell_workcell_20261025",
        "source": {"kind": "manual",
                   "inputs": [{"path": k, "sha256": v, "role": "intact_profile_input"}
                              for k, v in reg["source"]["files"].items() if (inputs / k).exists()],
                   "scale_evidence": "ChArUco 板 A 实测格长定尺度；桌面 1.8 m×0.805 m 与铃 85 mm 为实测"},
        "producer": {"repo": "Roboparty/RP-SceneReal2Sim", "version": "piper-workcell-20261025",
                     "command": "examples/piper_clickbell_workcell_20261025/build_scene.py"},
        "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "units": {"length": "m"}, "up_axis": "Z", "entry": "workcell.usda", "files": files,
        "objects": [
            {"name": "table", "prim_path": "/World/Workcell/Table", "category": "table", "graspable": False},
            {"name": "board_A", "prim_path": "/World/Workcell/BoardA", "category": "calibration_paper", "graspable": False},
            {"name": "qr_sheet", "prim_path": "/World/Workcell/QRSheet", "category": "paper", "graspable": False},
            {"name": "bell", "prim_path": "/World/Objects/BellProxy", "category": "bell", "graspable": False},
        ],
        "support_surfaces": [{"name": "table_top", "prim_path": "/World/Workcell/Table", "height_m": float(table[:4, 2].mean())}],
        "cameras": [{"name": "head", "prim_path": "/World/Cameras/head", "observed": True}],
        "robot_constraints": {"robots": ["piper-dual"]},
        "simulators": ["isaacsim", "physx"],
        "notes": "与 INTACT click_bell 现行场景几何一致（unified r5 转平、板 A 纸贴平、去板 B）。机器人、腕部相机和支架在 robot package 里，不在场景包。铃为代理体，正式资产 " + scene["bell_asset_id"] + "。",
    }
    (out / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=str(HERE / "scene"))
    print(build(ap.parse_args().out))
