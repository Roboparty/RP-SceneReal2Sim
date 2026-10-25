"""Run the real stage_worker export on the synthetic Blender fixture, CPU only."""
import json
import os
import subprocess
import sys
from pathlib import Path

root = Path(sys.argv[1]).resolve()
out = root/(sys.argv[2] if len(sys.argv)>2 else 'export')
validate_only = '--validate-only' in sys.argv[3:]
if not validate_only:
    out.mkdir(exist_ok=False)
spec = json.loads((root/'fixture/scene.json').read_text())
packet = dict(case_id='line3_export_fixture', stage='export',
              input_artifacts=dict(agent_model=[dict(path=str(root/'fixture/model.blend')),
                                                dict(path=str(root/'fixture/scene.json'))]),
              parameters=dict(blender=str(root/'blender-4.5.3-linux-x64/blender'),threads=2),
              output_directory=str(out),camera_constraints=dict(cameras=[spec['camera']],locked=True))
if not validate_only:
    (out/'packet.json').write_text(json.dumps(packet,indent=2))
env = os.environ.copy()
env.update(PYTHONPATH=str(root/'code/workflow'),R2S_CPU='1',CUDA_VISIBLE_DEVICES='')
if not validate_only:
    with (root/'stage_worker.log').open('w') as log:
        result = subprocess.run([sys.executable,'-m','r2s.stage_worker',str(out/'packet.json')],
                                env=env,stdout=log,stderr=subprocess.STDOUT)
    if result.returncode:
        print((root/'stage_worker.log').read_text()[-8000:])
        sys.exit(result.returncode)
doc = json.loads((out/'manifest.json').read_text())
receipt = json.loads((out/'package_export_receipt.json').read_text())
summary = dict(status='passed',scope='Actual workflow export stage, synthetic fixture, Blender 4.5.3, CPU 2 threads; no LLM/GPU',
               object_count=len(doc['objects']),cameras=doc['cameras'],
               observed_exported_count=receipt['observed_exported_count'],
               unmapped_audit_entities=receipt['unmapped_audit_entities'],
               files=[p.name for p in out.iterdir() if p.is_file()])
assert summary['object_count'] == 7
assert summary['observed_exported_count'] == 1
assert not summary['unmapped_audit_entities']
assert json.loads((out/'response.json').read_text())['status'] == 'complete'
sys.path.insert(0,str(root/'code/workflow'))
from r2s.core import Workflow
case = root/('acceptance_'+out.name)
case.mkdir(exist_ok=False)
(case/'case.json').write_text(json.dumps(dict(id='export_fixture',mode='single_image',inputs=[],branch='A')))
workflow = Workflow(case)
workflow.stage_map = {'export':([],False)}
workflow.state['stages']['export'] = dict(attempt=1)
assert workflow._finish('export',out,json.loads((out/'response.json').read_text()))
summary['workflow_finish_status'] = workflow.state['stages']['export']['status']
(root/'EXPORT_RESULT.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
