import json
import tempfile
import subprocess
import unittest
from pathlib import Path
from swarm_analysis.reader import read_run
from swarm_analysis.runner import run

ROOT=Path(__file__).resolve().parents[2]
class IntegrationTests(unittest.TestCase):
    def test_noisy_seeded_run_is_repeatable(self):
        binary=ROOT/'target/release/simulation.exe'
        if not binary.exists(): self.skipTest('build release simulation first')
        with tempfile.TemporaryDirectory(dir=ROOT/'work/analysis') as tmp:
            outputs=[]
            for index in (0, 1):
                trace=Path(tmp)/f'{index}.jsonl'
                result=subprocess.run([str(binary),'--agents','3','--seed','7','--steps','10','--noise','0','0','20','0.02','--trace',str(trace)],capture_output=True,check=True)
                outputs.append((json.loads(result.stdout), [json.loads(line) for line in trace.read_text().splitlines()]))
            self.assertEqual(outputs[0],outputs[1])
    def test_real_control_scenarios_and_truncation(self):
        binary=ROOT/'target/release/simulation.exe'
        if not binary.exists(): self.skipTest('build release simulation first')
        with tempfile.TemporaryDirectory(dir=ROOT/'work/analysis') as tmp:
            out=Path(tmp)/'suite'
            self.assertEqual(run(ROOT/'analysis/experiments/smoke.json',binary,out,60),0)
            folders={json.loads(p.read_text())['case']:p.parent for p in out.glob('*/metadata.json')}
            f,_=read_run(folders['free_one']); a=f[0]['agents'][0]; b=f[-1]['agents'][0]
            for k in (0,1): self.assertAlmostEqual(b['position'][k],a['position'][k]+a['velocity'][k]*f[-1]['time'],places=9)
            f,_=read_run(folders['hold_one']); self.assertEqual(f[0]['agents'][0]['position'],f[-1]['agents'][0]['position']); self.assertEqual(f[-1]['agents'][0]['state'],'arrived')
            f,_=read_run(folders['gather_one']); a=f[-1]['agents'][0]; self.assertEqual(a['state'],'arrived'); self.assertLess(a['task_observation']['target_distance'],.05)
            trace=folders['free_one']/'trace.jsonl'; trace.write_bytes(trace.read_bytes()[:-10])
            with self.assertRaises(ValueError): read_run(folders['free_one'])

if __name__=='__main__': unittest.main()
