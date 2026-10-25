"""Add an RP scene-package manifest without modifying delivered scene bytes."""
import argparse
import hashlib
import json
import re
from datetime import datetime, timezone
from pathlib import Path


def file_record(path, root, role='artifact'):
    path, root = Path(path), Path(root)
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return dict(path=path.relative_to(root).as_posix(), sha256=h.hexdigest(),
                bytes=path.stat().st_size, role=role)


def write_manifest(out, package_id, kind, entry, *, objects=(), cameras=(), inputs=(),
                   support_surfaces=(), scale_evidence='none', notes=''):
    out = Path(out)
    manifest = out / 'manifest.json'
    if manifest.exists():
        raise FileExistsError('Existing manifest retained: ' + str(manifest))
    if not (out / entry).is_file():
        raise FileNotFoundError(out / entry)
    excluded = {'manifest.json', 'packet.json', 'response.json', 'record.json'}
    files = [file_record(p, out, 'root_usd' if p.relative_to(out).as_posix() == entry else 'artifact')
             for p in sorted(out.rglob('*')) if p.is_file() and p.name not in excluded
             and p.suffix != '.log']
    doc = dict(schema='rp-real2sim2real/scene-package/1', id=package_id,
               source=dict(kind=kind, inputs=list(inputs), scale_evidence=scale_evidence),
               producer=dict(repo='Roboparty/RP-SceneReal2Sim', version='workcell-scene-package-1',
                             command='r2s scene package export'),
               created_at=datetime.now(timezone.utc).isoformat(timespec='seconds'),
               units=dict(length='m'), up_axis='Z', entry=entry, files=files,
               upstream=[],
               objects=list(objects), cameras=list(cameras), support_surfaces=list(support_surfaces),
               notes=notes)
    manifest.write_text(json.dumps(doc, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    return manifest


def export_workflow_package(out, case_id, camera_constraints=None):
    """Use actual exported prims, geometry audit, and accepted camera observations.

    Camera pose matching prevents diagnostic cameras being labelled observed.
    Missing externally observed views stay in the evidence receipt, not invented
    as exported cameras. No change to prior geometry/visual/physics verdicts.
    """
    import numpy as np
    from pxr import Usd, UsdGeom
    out = Path(out)
    audit = json.loads((out/'geometry_audit.json').read_text(encoding='utf-8'))
    scene = json.loads((out/'scene.json').read_text(encoding='utf-8'))
    stage = Usd.Stage.Open(str(out/'scene.usdc'))
    if stage is None:
        raise ValueError('Exported scene.usdc is unreadable')
    if abs(UsdGeom.GetStageMetersPerUnit(stage)-1.) > 1e-8 or UsdGeom.GetStageUpAxis(stage) != 'Z':
        raise ValueError('Workflow scene package requires a metre, Z-up stage')
    objects, missing = [], []
    for name in audit.get('entities', {}):
        matches = []
        for prim in stage.Traverse():
            if (prim.GetName() == name or prim.GetCustomDataByKey('entity_id') == name
                    or any(a.GetName().endswith(':entity_id') and a.Get() == name
                           for a in prim.GetAttributes())):
                matches.append(prim)
        if not matches:
            missing.append(name)
            continue
        prim = min(matches, key=lambda p: (len(str(p.GetPath())), str(p.GetPath())))
        source = next((o for o in scene.get('objects', []) if o.get('id') == name), {})
        objects.append(dict(name=name, prim_path=str(prim.GetPath()),
                            category=source.get('category', 'reconstructed_entity'),
                            graspable=bool(source.get('graspable', False))))
    observed = (camera_constraints or {}).get('cameras', [])
    cameras, matched = [], set()
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Camera):
            continue
        T = np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.)).T
        is_observed = False
        for i, c in enumerate(observed):
            # Canonical workflow cameras store world-to-CV rotation + world position.
            expected = np.eye(4)
            if 'T_world_camera' in c:
                expected = np.asarray(c['T_world_camera'], float)
            else:
                expected[:3,:3] = np.asarray(c['rotation_world_to_cv'], float).T
                expected[:3,3] = c['position']
            expected = expected @ np.diag([1., -1., -1., 1.])
            if np.allclose(T, expected, atol=1e-5, rtol=0):
                is_observed = True
                matched.add(i)
        cameras.append(dict(name=prim.GetName(), prim_path=str(prim.GetPath()), observed=is_observed))
    # Keep the externally supplied evidence in a portable receipt for traceability.
    receipt = dict(schema='rp-scene-package-export-receipt/1',
                   geometry_audit_sha256=file_record(out/'geometry_audit.json', out)['sha256'],
                   camera_observations=camera_constraints or None,
                   observed_camera_count=len(observed), observed_exported_count=len(matched),
                   unexported_observation_indices=sorted(set(range(len(observed)))-matched),
                   unmapped_audit_entities=missing,
                   acceptance='Original geometry, visual and physics conclusions are unchanged')
    (out/'package_export_receipt.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    support = [dict(name=o['name'], prim_path=o['prim_path']) for o in objects
               if o['category'] in {'table', 'floor', 'support_surface'} or o['name'] == 'floor']
    kind = 'multi_view' if len(observed) > 1 else 'single_image'
    package_id = re.sub('[^a-z0-9_.-]+', '_', str(case_id).lower()).strip('_.-') or 'scene'
    return write_manifest(out, package_id, kind, 'scene.usdc', objects=objects,
                          cameras=cameras, support_surfaces=support,
                          inputs=[file_record(out/'scene.json', out, 'canonical_scene'),
                                  file_record(out/'geometry_audit.json', out, 'geometry_audit')],
                          scale_evidence=str(scene.get('scale_evidence', 'none')),
                          notes='Manifest added after workflow USD export. Camera observed flags require pose matching to supplied camera observations. Unexported observations and unmatched entities are recorded in package_export_receipt.json. Existing acceptance verdicts unchanged.')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True, help='Existing workflow export directory; no prior manifest')
    ap.add_argument('--id', required=True)
    ap.add_argument('--camera-observations', help='Accepted workflow constraints JSON, cameras[]')
    args = ap.parse_args()
    constraints = json.loads(Path(args.camera_observations).read_text(encoding='utf-8')) if args.camera_observations else None
    print(export_workflow_package(args.out, args.id, constraints))


if __name__ == '__main__':
    main()
