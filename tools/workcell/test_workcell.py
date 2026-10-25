"""End-to-end USD regressions; real RGB fixture, independent USD projection."""
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'workflow'))
from r2s.workcell import build, load, save, apply
from r2s.scene_package import export_workflow_package, file_record


class WorkcellTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.inputs = self.root/'inputs'
        shutil.copytree(ROOT/'examples/piper_rgb_workcell_20261026/inputs', self.inputs)

    def tearDown(self):
        self.temp.cleanup()

    def generate(self, cfg=None):
        if cfg is not None:
            save(self.inputs/'measurements.json', cfg)
        return build(self.inputs/'measurements.json', self.root/'scene')

    def check_hashes(self, out):
        manifest = load(out/'manifest.json')
        for row in manifest['files']:
            self.assertEqual(row['sha256'], file_record(out/row['path'], out)['sha256'])
        return manifest

    def test_real_rgb_usd_camera_and_geometry(self):
        out = self.generate()
        manifest = self.check_hashes(out)
        stage = Usd.Stage.Open(str(out/'workcell.usdc'))
        data = load(out/'calibration.json')
        self.assertEqual(len(manifest['cameras']), 2)
        self.assertTrue(all(c['observed'] for c in manifest['cameras']))
        self.assertFalse(stage.GetPrimAtPath('/World/Workcell/BoardA').HasAPI(UsdPhysics.CollisionAPI))
        table = np.array(UsdGeom.Mesh(stage.GetPrimAtPath('/World/Workcell/Table')).GetPointsAttr().Get())
        collider = np.array(UsdGeom.Mesh(stage.GetPrimAtPath('/World/Colliders/Table')).GetPointsAttr().Get())
        np.testing.assert_array_equal(table, collider)
        np.testing.assert_allclose(table[:4, 2], 0, atol=1e-8)
        for c, receipt in zip(manifest['cameras'], data['frames']):
            frustum = UsdGeom.Camera(stage.GetPrimAtPath(c['prim_path'])).GetCamera(0).frustum
            M = frustum.ComputeViewMatrix() * frustum.ComputeProjectionMatrix()
            world = apply(receipt['corners_board_m'], np.array(data['T_world_board']))
            clip = np.array([Gf.Vec4d(*map(float,p),1.) * M for p in world])
            ndc = clip[:,:2]/clip[:,3:4]
            w,h = receipt['image_size']
            usd_px = np.c_[(ndc[:,0]+1)*w/2-.5, (1-ndc[:,1])*h/2-.5]
            observed = np.array(receipt['detected_px'])
            rms = np.sqrt(np.mean(np.sum((usd_px-observed)**2,axis=1)))
            self.assertAlmostEqual(rms, receipt['head_pnp']['rms_px'], places=4)

    def test_deterministic_and_no_overwrite(self):
        out = self.generate()
        before = (out/'manifest.json').read_bytes()
        with self.assertRaises(FileExistsError):
            build(self.inputs/'measurements.json', out)
        self.assertEqual(before, (out/'manifest.json').read_bytes())
        again = build(self.inputs/'measurements.json', self.root/'scene_again')
        for path in out.rglob('*'):
            if path.is_file() and path.name != 'manifest.json':
                self.assertEqual(path.read_bytes(), (again/path.relative_to(out)).read_bytes(),
                                 str(path.relative_to(out)))

    def test_level_and_asset_complete_package(self):
        cfg = load(self.inputs/'measurements.json')
        # A two-degree measured plane plus an actual supplied USD visual asset.
        top = np.array(cfg['table']['top_corners_board_m'])
        top[:,2] = .03*top[:,0]
        cfg['table']['top_corners_board_m'] = top.tolist()
        stage = Usd.Stage.CreateNew(str(self.inputs/'asset.usda'))
        cube = UsdGeom.Cube.Define(stage, '/Asset')
        stage.SetDefaultPrim(cube.GetPrim())
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        stage.GetRootLayer().Save()
        cfg['objects'][0]['asset'] = 'asset.usda'
        out = self.generate(cfg)
        self.check_hashes(out)
        stage = Usd.Stage.Open(str(out/'workcell.usdc'))
        p = np.array(UsdGeom.Mesh(stage.GetPrimAtPath('/World/Workcell/Table')).GetPointsAttr().Get())
        self.assertLess(np.ptp(p[:4,2]), 1e-7)
        self.assertTrue((out/'bell.usdz').is_file())
        asset = stage.GetPrimAtPath('/World/Objects/bell')
        self.assertTrue(stage.GetPrimAtPath('/World/Objects/bell/Visual').IsA(UsdGeom.Cube))
        self.assertGreater(np.linalg.det(np.array(UsdGeom.Xformable(asset).ComputeLocalToWorldTransform(0.))[:3,:3]), 0)

    def test_wrong_resolution_records_failure(self):
        intr = load(self.inputs/'intrinsics.json')
        intr['image_size'] = [640,480]
        save(self.inputs/'intrinsics.json', intr)
        with self.assertRaisesRegex(ValueError, 'resolution'):
            self.generate()
        self.assertFalse((self.root/'scene/manifest.json').exists())

    def test_workflow_export_manifest_does_not_change_scene(self):
        # Exercise workflow packaging with actual USD prims, a matched observed
        # camera, and a separate unobserved diagnostic camera.
        out = self.generate()
        shutil.copyfile(out/'workcell.usdc', out/'scene.usdc')
        cfg = load(out/'calibration.json')
        save(out/'scene.json', dict(objects=[dict(id='Table', category='table')]))
        save(out/'geometry_audit.json', dict(entities={'Table': {}}))
        (out/'manifest.json').unlink()  # our temporary output only
        before = file_record(out/'scene.usdc', out)['sha256']
        constraints = dict(cameras=[dict(frame_id='head_000', T_world_camera=cfg['frames'][0]['T_world_camera_cv'])])
        export_workflow_package(out, 'workflow_fixture', constraints)
        self.assertEqual(before, file_record(out/'scene.usdc', out)['sha256'])
        manifest = self.check_hashes(out)
        self.assertEqual([c['observed'] for c in manifest['cameras']], [True,False])
        self.assertEqual(manifest['objects'][0]['prim_path'], '/World/Workcell/Table')


if __name__ == '__main__':
    unittest.main(verbosity=2)
