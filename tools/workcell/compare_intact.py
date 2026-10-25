"""Read-only comparison with INTACT's actual apply_scene + unified-r5 path."""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'workflow'))
from r2s.workcell import apply, save, CV_GL
from r2s.scene_package import file_record
from embodiedreal2sim.solve import project_points


def world_points(stage, path):
    prim = stage.GetPrimAtPath(path)
    T = np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.)).T
    return apply(np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), float), T)


def stats(a, b):
    d = np.linalg.norm(np.asarray(a)-np.asarray(b), axis=1)*1000
    return dict(count=len(d), max_mm=float(d.max()), rms_mm=float(np.sqrt(np.mean(d*d))))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--intact', required=True)
    ap.add_argument('--scene', required=True)
    ap.add_argument('--legacy-scene', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    intact = Path(args.intact).resolve()
    sys.path.insert(0, str(intact/'src'))
    from rparena.collection.backends.piper_clickbell.real_profile import ROOT, read_inputs
    from rparena.collection.backends.piper_clickbell.unified_chain import level_profile
    from rparena.collection.backends.piper_clickbell.real_scene import apply_scene
    profile = read_inputs()
    config = json.loads((ROOT/'unified-r5.json').read_text(encoding='utf-8'))
    original_to_nominal = np.asarray(profile['to_nominal'], float).copy()
    level_profile(profile, config)
    t = np.asarray(profile['to_nominal'])
    reference = Usd.Stage.CreateInMemory()
    applied = apply_scene(reference, profile, ROOT)
    old_stage = Usd.Stage.Open(str(Path(args.legacy_scene)/'workcell.usda'))
    stage = Usd.Stage.Open(str(Path(args.scene)/'workcell.usdc'))
    calibration = json.loads((Path(args.scene)/'calibration.json').read_text(encoding='utf-8'))
    W = np.asarray(calibration['T_world_board'])
    old_to_new = W @ np.linalg.inv(t)
    mapping = [('table_visual','/World/Workcell/Table','/World/ReplayScene/RegisteredTable'),
               ('table_collision','/World/Colliders/Table','/World/Colliders/RealAlignedTable'),
               ('board_A','/World/Workcell/BoardA','/World/ReplayScene/BoardA'),
               ('paper_A','/World/Workcell/PaperA','/World/ReplayScene/PaperA'),
               ('qr_sheet','/World/Workcell/QRSheet','/World/ReplayScene/QRSheet')]
    old_check, new_check = {}, {}
    for name, legacy, path in mapping:
        ref = world_points(reference, path)
        old_check[name] = stats(world_points(old_stage, legacy), ref)
        new_path = '/World/Workcell/BoardA' if name == 'board_A' else legacy
        if stage.GetPrimAtPath(new_path):
            actual = world_points(stage, new_path)
            expected = apply(ref, old_to_new)
            raw = stats(actual, expected)
            # Expected 8mm change is kept separate from remaining pointwise error.
            corrected = expected + W[:3, :3] @ np.array([0.,0.,-.008])
            new_check[name] = dict(raw=raw, after_explicit_8mm_plane_correction=stats(actual, corrected))
        else:
            new_check[name] = dict(status='not_modelled', reason='Not present in this measured fixture; not counted as matching')
    authored = json.loads((ROOT/'authored_cameras.json').read_text(encoding='utf-8'))['head']
    old_cam_board = np.linalg.inv(original_to_nominal) @ np.asarray(authored['T_world_camera_gl']) @ CV_GL
    historical_cal = json.loads((ROOT/'calibration_v2.json').read_text(encoding='utf-8'))
    head_T = np.asarray(historical_cal['head_T'])
    # The authored source uses the head_T orientation/pose but has an explicitly
    # recorded render-chain gauge. Compare rather than guessing their identity.
    camera = []
    for frame in calibration['frames']:
        new_cam_board = np.asarray(frame['T_board_camera_cv'])
        delta = np.linalg.inv(old_cam_board) @ new_cam_board
        angle = np.degrees(np.arccos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1)))
        uv = project_points(np.linalg.inv(old_cam_board), frame['corners_board_m'], frame['K'], frame['D'])
        err = np.linalg.norm(uv-np.asarray(frame['detected_px']), axis=1)
        head_err = np.linalg.norm(project_points(np.linalg.inv(head_T), frame['corners_board_m'], frame['K'], frame['D'])-np.asarray(frame['detected_px']), axis=1)
        camera.append(dict(frame_id=frame['frame_id'], new_board_fit_rms_px=frame['head_pnp']['rms_px'],
                           old_authored_pose_current_K_rms_px=float(np.sqrt(np.mean(err*err))),
                           old_calibration_head_T_current_K_rms_px=float(np.sqrt(np.mean(head_err*head_err))),
                           translation_delta_mm=float(np.linalg.norm(delta[:3,3])*1000),
                           rotation_delta_deg=float(angle)))
    source_files = [file_record(ROOT/p, ROOT, 'frozen_intact_profile') for p in
                    ['profile.json','unified-r5.json','authored_cameras.json','calibration_v2.json','scene_night.usda']]
    normal = -original_to_nominal[:3,2]
    measurements = json.loads((Path(args.scene)/'inputs/measurements.json').read_text(encoding='utf-8'))
    diagnostic = []
    for frame, receipt in zip(measurements['images'], calibration['frames']):
        pixel = frame.get('diagnostic_bell_ground_center_px')
        if pixel is None:
            continue
        T = np.asarray(receipt['T_board_camera_cv'])
        ray = T[:3,:3] @ np.linalg.inv(receipt['K']) @ np.r_[pixel,1.]
        inferred = T[:3,3] - T[2,3]/ray[2]*ray
        anchor = next(o['base_center_board_m'] for o in measurements['objects'] if o['name']=='bell')
        diagnostic.append(dict(frame_id=receipt['frame_id'], approximate_ground_center_px=pixel,
                               inferred_board_m=inferred.tolist(),
                               delta_from_frozen_anchor_mm=((inferred-anchor)*1000).tolist(),
                               scope='Manual approximate base ellipse centre, ray-plane diagnostic only; not a new tape measurement or accepted replacement anchor'))
    result = dict(schema='rp-workcell-intact-comparison/1',
                  reference='Actual INTACT real_scene.apply_scene after unified_chain.level_profile; empty USD stage, no physics/GPU or hardware job',
                  reference_profile=profile['name'], sources=source_files, apply_scene_receipt=applied,
                  legacy_regression=old_check, new_vs_current=new_check, cameras=camera,
                  historical_unlevelled_table_tilt_deg=float(np.degrees(np.arccos(normal[2]/np.linalg.norm(normal)))),
                  T_new_world_from_intact_world=old_to_new.tolist(),
                  historical_head_T_vs_authored_board_max_abs=float(np.max(abs(head_T-old_cam_board))),
                  historical_calibration_statistics_source=historical_cal.get('statistics_source'),
                  historical_calibration_joint_source=historical_cal.get('joint_source'),
                  bell_ground_diagnostic=diagnostic,
                  limitations=['QR paper and removed board B are not in the measured fixture.',
                              'Bell proxy compared separately to annotated real silhouette, not claimed identical to detailed asset.',
                              'Current-K/old-pose residual isolates extrinsics; historical render intrinsics are not silently substituted.'])
    save(args.out, result)
    print(json.dumps(dict(legacy_max_mm=max(v['max_mm'] for v in old_check.values()),
                          new_vs_current=new_check, cameras=camera), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
