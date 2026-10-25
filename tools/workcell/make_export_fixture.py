"""Blender-only synthetic fixture for the actual workflow export stage.

This is an exporter regression, not a real-room reconstruction accuracy result.
"""
import json
import sys
from pathlib import Path

import bpy

root = Path(sys.argv[sys.argv.index('--')+1]).resolve()
root.mkdir(parents=True, exist_ok=False)
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
rows = [
    ('floor', [0,0,-.05], [3,3,.1]),
    ('ceiling', [0,0,2.55], [3,3,.1]),
    ('wall_back', [0,1.55,1.25], [3,.1,2.5]),
    ('wall_front', [0,-1.55,1.25], [3,.1,2.5]),
    ('wall_left', [-1.55,0,1.25], [.1,3,2.5]),
    ('wall_right', [1.55,0,1.25], [.1,3,2.5]),
    ('lamp', [0,0,2.3], [.1,.1,.1]),
]
objects = []
for name, pos, size in rows:
    bpy.ops.mesh.primitive_cube_add(size=1, location=pos)
    obj = bpy.context.object
    obj.name = name
    obj['entity_id'] = name
    obj.scale = size
    objects.append(dict(id=name, position=pos, dimensions=size,
                        kind='lamp' if name == 'lamp' else 'room_surface',
                        category='lamp' if name == 'lamp' else ('floor' if name == 'floor' else 'shell'),
                        evidence=['synthetic exporter fixture, not a source photograph'], confidence=1.))
bpy.ops.object.light_add(type='AREA', location=(0,0,2.1))
bpy.context.object.data.energy = 100
bpy.context.object.data.shape = 'DISK'
bpy.context.object.data.size = 1
bpy.ops.object.camera_add(location=(0,-1,1.2))
camera = bpy.context.object
camera.name = 'head_observed'
scene.camera = camera
scene.render.engine = 'CYCLES'
scene.render.resolution_percentage = 100
scene.world = bpy.data.worlds.new('World')
scene.world.use_nodes = True
spec = dict(schema_version='real2sim.scene/1.0', units='m', up_axis='Z', branch='A',
            room=dict(x_min=-1.5,x_max=1.5,y_min=-1.5,y_max=1.5,height=2.5,
                      thickness=.1,preserve_full_shell=True),
            objects=objects, scale_evidence='Synthetic dimensions; exporter regression only',
            camera=dict(frame_id='head_observed',position=[0,-1,1.2],
                        rotation_world_to_cv=[[1,0,0],[0,0,-1],[0,1,0]],
                        focal_px=100.,focal_y_px=95.,principal_point=[62.,47.],
                        image_size=[128,96],pixel_coordinates='integer_centers'))
(root/'scene.json').write_text(json.dumps(spec,indent=2), encoding='utf-8')
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'workflow'))
from r2s.blender_camera import apply_camera
apply_camera(scene,camera,spec['camera'])
bpy.ops.wm.save_as_mainfile(filepath=str(root/'model.blend'))
print('EXPORT_FIXTURE_READY',root)
