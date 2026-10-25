"""Offline measured workcell: RGB + board A -> USD, package, reprojection receipts.

No devices, Blender, network or language models. T_parent_child uses column
vectors; board axes are the RP-RoboReal2Sim printed pattern axes, metres.
"""
import argparse
import json
import re
import shutil
from pathlib import Path

import cv2
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils

from .scene_package import file_record, write_manifest

FACES = [[0, 1, 2, 3], [4, 7, 6, 5], [0, 4, 5, 1],
         [1, 5, 6, 2], [2, 6, 7, 3], [3, 7, 4, 0]]
CV_GL = np.diag([1., -1., -1., 1.])


def load(path):
    path = Path(path)
    if path.suffix.lower() in {'.yaml', '.yml'}:
        import yaml
        return yaml.safe_load(path.read_text(encoding='utf-8-sig'))
    return json.loads(path.read_text(encoding='utf-8-sig'))


def save(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2,
                                    allow_nan=False) + '\n', encoding='utf-8')


def points(value, shape):
    a = np.asarray(value, float)
    if a.shape != shape or not np.isfinite(a).all():
        raise ValueError(f'Expected finite {shape} measurements')
    return a


def apply(p, T):
    return np.asarray(p) @ T[:3, :3].T + T[:3, 3]


def mesh(stage, path, p, faces, color, collision=False, invisible=False):
    obj = UsdGeom.Mesh.Define(stage, path)
    obj.CreatePointsAttr(np.asarray(p).tolist())
    obj.CreateFaceVertexCountsAttr([len(f) for f in faces])
    obj.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    obj.CreateSubdivisionSchemeAttr('none')
    obj.CreateDoubleSidedAttr(True)
    obj.CreateDisplayColorAttr([color])
    if collision:
        UsdPhysics.CollisionAPI.Apply(obj.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(obj.GetPrim()).CreateApproximationAttr('convexHull')
    if invisible:
        obj.CreatePurposeAttr(UsdGeom.Tokens.proxy)
    return obj


def camera(stage, path, T, K, size):
    """USD apertures in tenths of stage units; preserve fx != fy and pixel centres."""
    c = UsdGeom.Camera.Define(stage, path)
    UsdGeom.Xformable(c).AddTransformOp().Set(Gf.Matrix4d((T @ CV_GL).T.tolist()))
    w, h = size
    f = .18
    c.CreateFocalLengthAttr(f)
    c.CreateHorizontalApertureAttr(f * w / K[0, 0])
    c.CreateVerticalApertureAttr(f * h / K[1, 1])
    c.CreateHorizontalApertureOffsetAttr(f * (w / 2 - K[0, 2] - .5) / K[0, 0])
    c.CreateVerticalApertureOffsetAttr(f * (K[1, 2] + .5 - h / 2) / K[1, 1])
    c.CreateClippingRangeAttr(Gf.Vec2f(.001, 100.))
    return c


def perimeter(poly, step=1.):
    poly = np.asarray(poly, float)
    return np.concatenate([np.linspace(a, b, max(2, int(np.linalg.norm(a-b)/step)+1))
                           for a, b in zip(poly, np.roll(poly, -1, axis=0))])


def contour_error(predicted, observed):
    """Symmetric boundary distance; not a corner correspondence or PnP score."""
    from scipy.spatial import cKDTree
    a, b = perimeter(predicted), perimeter(observed)
    d = np.r_[cKDTree(a).query(b)[0], cKDTree(b).query(a)[0]]
    return dict(mean_px=float(d.mean()), rms_px=float(np.sqrt(np.mean(d*d))),
                p95_px=float(np.percentile(d, 95)), max_px=float(d.max()),
                metric='symmetric sampled boundary distance, 1px spacing')


def build(measurements_path, out):
    # Import the maintained robotics implementation rather than copying detection/PnP.
    from embodiedreal2sim.detect import detect_board
    from embodiedreal2sim.solve import solve_head_pnp, project_points
    from embodiedreal2sim.targets import make_board, resolve_spec

    measurements_path = Path(measurements_path).resolve()
    root, cfg = measurements_path.parent, load(measurements_path)
    if cfg.get('schema') != 'rp-workcell-measurements/1':
        raise ValueError('Expected rp-workcell-measurements/1')
    if cfg.get('units') != 'm':
        raise ValueError('Measurements must explicitly use metres')
    if not re.fullmatch('[a-z0-9][a-z0-9_.-]*', cfg['id']):
        raise ValueError('Invalid package id')
    targets_path = (root / cfg['targets']).resolve()
    target_config = load(targets_path)
    spec = resolve_spec(target_config)
    intrinsics_path = (root / cfg['intrinsics']).resolve()
    intr = load(intrinsics_path)
    K = points(intr['K'], (3, 3))
    D = np.asarray(intr.get('D', [0]*5), float)
    size = intr['image_size']
    if len(size) != 2 or any(not isinstance(v, int) or v <= 0 for v in size):
        raise ValueError('image_size must be positive integer [width,height]')
    top = points(cfg['table']['top_corners_board_m'], (4, 3))
    thickness = float(cfg['table']['thickness_m'])
    if not np.isfinite(thickness) or thickness <= 0:
        raise ValueError('Table thickness must be positive')
    _, _, vh = np.linalg.svd(top - top.mean(0))
    n = vh[-1]
    if n[2] < 0:
        n = -n
    if abs(n[2]) < .5 or np.linalg.matrix_rank(top-top.mean(0), tol=1e-8) < 2:
        raise ValueError('Measured tabletop must span a plane near board A')
    warnings = []
    flatness = float(np.max(abs((top - top.mean(0)) @ n)))
    if flatness > .001:
        warnings.append(f'Table nonplanarity {flatness*1000:.3f} mm; measured corners retained')
    if abs(top.mean(0) @ n) > .001:
        warnings.append('Measured table plane differs from board A z=0; no hidden height correction')
    # World origin stays at A. Flip y/z together so outward normal becomes +Z.
    W = CV_GL.copy()
    if cfg.get('level_table', False):
        # Rotate the whole workcell, including all cameras, about the board origin.
        a, b = W[:3, :3] @ (-n), np.array([0., 0., 1.])
        v, cosine = np.cross(a, b), float(a @ b)
        cross = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        W[:3, :3] = (np.eye(3) + cross + cross @ cross / (1 + cosine)) @ W[:3, :3]
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out / 'inputs').mkdir()
    # Copy small, necessary evidence, never alter inputs or overwrite prior output.
    input_records = []
    def retain(path, name):
        dst = out / 'inputs' / name
        shutil.copyfile(path, dst)
        input_records.append(file_record(dst, out, 'measurement_evidence'))
        return dst.relative_to(out).as_posix()
    retain(measurements_path, 'measurements' + measurements_path.suffix)
    retain(targets_path, 'targets' + targets_path.suffix)
    retain(intrinsics_path, 'intrinsics' + intrinsics_path.suffix)
    frames = cfg['images']
    if not frames:
        raise ValueError('At least one head RGB image is required')
    receipts, poses, images = [], [], []
    for i, frame in enumerate(frames):
        src = (root / frame['path']).resolve()
        img = cv2.imdecode(np.fromfile(src, np.uint8), cv2.IMREAD_COLOR)
        if img is None or [img.shape[1], img.shape[0]] != size:
            raise ValueError(f'Image and calibrated resolution differ: {src.name}')
        rel = retain(src, f'head_{i:03d}' + src.suffix)
        obs = detect_board(img, 'A', spec, K=K, D=D,
                           gates={'require_pose': True, 'allow_board_marker_fallback': False})
        save(out / f'detection_{i:03d}.json', obs)
        if not obs.get('object_points_m') or obs.get('T_camera_board') is None:
            raise ValueError(f'No readable board A pose in {src.name}; retained detection diagnostics')
        if not obs['accepted']:
            warnings.append(f'{src.name}: detection diagnostics {obs["reasons"]}')
        pnp = solve_head_pnp(obs['object_points_m'], obs['image_points_px'], K, D)
        if not pnp.success:
            raise ValueError(f'No valid head_pnp solution: {src.name}')
        if pnp.rms_px > cfg.get('warning_rms_px', 1.5):
            warnings.append(f'{src.name}: PnP RMS {pnp.rms_px:.3f}px')
        poses.append(W @ pnp.T_world_camera)
        images.append(img)
        receipts.append(dict(frame_id=f'head_{i:03d}', image=rel, image_size=size,
                             K=K.tolist(), D=D.tolist(), detected_corner_count=len(obs['ids']),
                             corners_board_m=obs['object_points_m'], detected_px=obs['image_points_px'],
                             T_board_camera_cv=pnp.T_world_camera.tolist(),
                             T_world_camera_cv=poses[-1].tolist(), head_pnp=pnp.to_dict()))
    stage = Usd.Stage.CreateNew(str(out / 'workcell.usdc'))
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, '/World').GetPrim())
    stage.GetDefaultPrim().SetCustomDataByKey('T_world_board', W.tolist().__str__())
    volume = np.vstack([top, top + thickness * n])
    mesh(stage, '/World/Workcell/Table', apply(volume, W), FACES, [.73, .72, .69])
    mesh(stage, '/World/Colliders/Table', apply(volume, W), FACES, [.73, .72, .69], True, True)
    bspec = spec['boards']['A']
    width, height = bspec['cols']*bspec['square_mm']/1000, bspec['rows']*bspec['square_mm']/1000
    board_corners = np.array([[0, 0, 0], [width, 0, 0], [width, height, 0], [0, height, 0]])
    board = mesh(stage, '/World/Workcell/BoardA', apply(board_corners+[0, 0, -.0001], W),
                 [[0, 1, 2, 3]], [1., 1., 1.])
    # Pattern only: quiet zone is not part of the measured coordinate system.
    pattern = make_board('A', spec).generateImage((1400, 1000), marginSize=0)
    cv2.imwrite(str(out / 'board_A.png'), pattern)
    mat = UsdShade.Material.Define(stage, '/World/Looks/Board')
    shader = UsdShade.Shader.Define(stage, '/World/Looks/Board/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    tex = UsdShade.Shader.Define(stage, '/World/Looks/Board/Texture')
    tex.CreateIdAttr('UsdUVTexture')
    tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath('./board_A.png'))
    uv = UsdShade.Shader.Define(stage, '/World/Looks/Board/UV')
    uv.CreateIdAttr('UsdPrimvarReader_float2')
    uv.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
    tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(), 'result')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(board.GetPrim()).Bind(mat)
    UsdGeom.PrimvarsAPI(board).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray,
                                          UsdGeom.Tokens.vertex).Set([(0.,1.),(1.,1.),(1.,0.),(0.,0.)])
    geometry = {'table_top': top.tolist(), 'board_A': board_corners.tolist()}
    objects = [dict(name='table', prim_path='/World/Workcell/Table', category='table', graspable=False),
               dict(name='board_A', prim_path='/World/Workcell/BoardA', category='calibration_paper', graspable=False)]
    seen = set(geometry)
    for obj in cfg.get('objects', []):
        name = obj['name']
        if not re.fullmatch('[A-Za-z_][A-Za-z0-9_]*', name) or name in seen:
            raise ValueError('Object names must be unique USD identifiers')
        seen.add(name)
        d = points(obj['dimensions_m'], (3,))
        base = points(obj['base_center_board_m'], (3,))
        if np.any(d <= 0):
            raise ValueError('Object dimensions must be positive')
        yaw = np.deg2rad(float(obj.get('yaw_deg', 0)))
        R = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1.]])
        foot = np.array([[-d[0]/2, -d[1]/2, 0], [d[0]/2, -d[1]/2, 0],
                         [d[0]/2, d[1]/2, 0], [-d[0]/2, d[1]/2, 0]])
        box = np.vstack([foot, foot-[0, 0, d[2]]]) @ R.T + base
        geometry[name] = box.tolist()
        path = '/World/Objects/' + name
        if obj.get('asset'):
            # Package USD + referenced textures directly; no downloads or LLM.
            asset = (root / obj['asset']).resolve()
            packed = out / (name + '.usdz')
            if not UsdUtils.CreateNewUsdzPackage(Sdf.AssetPath(str(asset)), str(packed)):
                raise ValueError('Unable to package asset ' + asset.name)
            asset_stage = Usd.Stage.Open(str(packed))
            prim = asset_stage.GetDefaultPrim()
            if not prim or UsdGeom.GetStageUpAxis(asset_stage) != 'Z':
                raise ValueError('Asset must have defaultPrim and Z up')
            bounds = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(prim).ComputeAlignedRange()
            lo, hi = np.array(bounds.GetMin()), np.array(bounds.GetMax())
            if np.any(hi <= lo):
                raise ValueError('Asset must have nonzero 3D bounds')
            A = np.eye(4)
            A[:3, :3] = R @ np.diag([1, -1, -1]) @ np.diag(d/(hi-lo))
            A[:3, 3] = base - A[:3, :3] @ np.r_[(lo[:2]+hi[:2])/2, lo[2]]
            visual = UsdGeom.Xform.Define(stage, path)
            # Untyped child preserves a referenced Mesh/Cube/other typed root;
            # authoring a stronger Xform at that same path would hide its type.
            stage.OverridePrim(path + '/Visual').GetReferences().AddReference('./' + packed.name)
            UsdGeom.Xformable(visual).MakeMatrixXform().Set(Gf.Matrix4d((W @ A).T.tolist()))
            mesh(stage, '/World/Colliders/'+name, apply(box, W), FACES, [.7,.6,.2], True, True)
        else:
            mesh(stage, path, apply(box, W), FACES, obj.get('color', [.75,.62,.25]), True)
        objects.append(dict(name=name, prim_path=path, category=obj.get('category', 'measured_proxy'),
                            graspable=bool(obj.get('graspable', False))))
    camera_records = []
    for i, T in enumerate(poses):
        path = '/World/Cameras/head_' + f'{i:03d}'
        camera(stage, path, T, K, size)
        camera_records.append(dict(name=f'head_{i:03d}', prim_path=path, observed=True))
    stage.GetRootLayer().Save()
    # Reopen the actual delivered USD and use its camera extrinsic for overlays.
    stage = Usd.Stage.Open(str(out / 'workcell.usdc'))
    reprojection = []
    for i, (frame, receipt, img) in enumerate(zip(frames, receipts, images)):
        Tc = np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(camera_records[i]['prim_path']))
                        .ComputeLocalToWorldTransform(0.)).T @ CV_GL
        Tcw = np.linalg.inv(Tc)
        projected = {}
        for j, (name, geom) in enumerate(geometry.items()):
            # Overlay delivered geometry, not merely the pre-export arrays.
            path = {'table_top': '/World/Workcell/Table',
                    'board_A': '/World/Workcell/BoardA'}.get(name, '/World/Objects/'+name)
            prim = stage.GetPrimAtPath(path)
            if not prim.IsA(UsdGeom.Mesh):
                prim = stage.GetPrimAtPath('/World/Colliders/'+name)
            pm = np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.)).T
            pw = apply(np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), float), pm)
            if name == 'table_top':
                pw = pw[:4]
            if np.any(apply(pw, Tcw)[:, 2] <= 0):
                warnings.append(f'head_{i:03d}/{name}: nonpositive depth; contour not drawn')
                continue
            uv = project_points(Tcw, pw, K, D)
            projected[name] = uv.tolist()
            hull = cv2.convexHull(uv.astype(np.float32)).reshape(-1, 2)
            color = [(0, 180, 255), (255, 80, 0), (0, 0, 230)][min(j, 2)]
            # clipLine avoids integer overflow for far-away or off-image points.
            for a, b in zip(hull, np.roll(hull, -1, axis=0)):
                a, b = np.clip(a, -1e6, 1e6).astype(int), np.clip(b, -1e6, 1e6).astype(int)
                ok, a, b = cv2.clipLine((0, 0, size[0], size[1]), tuple(a), tuple(b))
                if ok:
                    cv2.line(img, a, b, color, 2, cv2.LINE_AA)
        p = apply(receipt['corners_board_m'], W)
        predicted = project_points(Tcw, p, K, D)
        observed = np.asarray(receipt['detected_px'])
        errors = np.linalg.norm(predicted-observed, axis=1)
        for a, b in zip(predicted, observed):
            cv2.circle(img, tuple(np.rint(b).astype(int)), 3, (0, 220, 0), -1)
            cv2.line(img, tuple(np.rint(a).astype(int)), tuple(np.rint(b).astype(int)), (0, 0, 255), 1)
        independent = {}
        for name, contour in frame.get('object_contours_px', {}).items():
            if name not in projected:
                raise ValueError('Annotation has unknown or nonprojectable object ' + name)
            contour = np.asarray(contour, float)
            if contour.ndim != 2 or contour.shape[1] != 2 or len(contour) < 3 or not np.isfinite(contour).all():
                raise ValueError('Observed contour must be a finite Nx2 polygon')
            hull = cv2.convexHull(np.asarray(projected[name], np.float32)).reshape(-1, 2)
            independent[name] = contour_error(hull, contour)
            cv2.polylines(img, [np.rint(contour).astype(np.int32)], True, (220, 0, 220), 2)
        landmarks = []
        for mark in frame.get('landmarks', []):
            point = points(mark['point_board_m'], (3,))
            pixel = points(mark['pixel'], (2,))
            uv = project_points(Tcw, apply([point], W), K, D)[0]
            landmarks.append(dict(name=mark['name'], predicted_px=uv.tolist(), observed_px=pixel.tolist(),
                                  error_px=float(np.linalg.norm(uv-pixel))))
        result = dict(frame_id=receipt['frame_id'], board_A_fit=dict(count=len(errors),
                      rms_px=float(np.sqrt(np.mean(errors**2))), max_px=float(errors.max())),
                      projected_geometry_px=projected, independent_contours=independent,
                      independent_landmarks=landmarks,
                      scope='Board corners fit PnP; contours/landmarks are independent only when manually supplied. Unlabelled geometry has no measured pixel error.')
        text = f'Board A PnP fit: {result["board_A_fit"]["rms_px"]:.3f}px; n={len(errors)}'
        cv2.putText(img, text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, .65, (0,0,0), 3)
        cv2.putText(img, text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, .65, (0,255,0), 1)
        cv2.imwrite(str(out / f'overlay_{i:03d}.png'), img)
        reprojection.append(result)
    save(out / 'calibration.json', dict(T_world_board=W.tolist(), board_frame='x columns, y rows, +z into table',
         world_frame='board A origin; x board x, outward +Z; optional common leveling',
         board_A_physical_height_m=0, board_A_visual_offset_m=-.0001,
         level_table=bool(cfg.get('level_table', False)), table_nonplanarity_m=flatness,
         frames=receipts, warnings=warnings))
    save(out / 'geometry_board.json', geometry)
    save(out / 'reprojection.json', dict(frames=reprojection, warnings=warnings))
    write_manifest(out, cfg['id'], 'manual', 'workcell.usdc', objects=objects,
                   cameras=camera_records, inputs=input_records,
                   support_surfaces=[dict(name='table_top', prim_path='/World/Workcell/Table',
                                          height_m=float(apply(top, W)[:, 2].mean()))],
                   scale_evidence=cfg['scale_evidence'],
                   notes='Deterministic RGB board-A head_pnp + measured geometry. D in calibration.json; USD is pinhole. All views retained; no device access or LLM.')
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--measurements', required=True, help='UTF-8 JSON/YAML; paths relative to this file')
    ap.add_argument('--out', required=True, help='New output directory, must not exist')
    args = ap.parse_args()
    print(build(args.measurements, args.out))


if __name__ == '__main__':
    main()
