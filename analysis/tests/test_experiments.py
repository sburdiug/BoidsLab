import json
import tempfile
import unittest
from pathlib import Path
from swarm_analysis.experiments import load_manifest

class ManifestTests(unittest.TestCase):
    def test_grid_order_and_rejects_missing_key(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[2]/'work/analysis') as tmp:
            path=Path(tmp)/'input.json'
            data={'schema_version':1,'name':'test','cases':[{'name':'one','grid':{'seed':[2,1],'weight':[0,3]},'args':['--seed','${seed}','--alignment-weight','${weight}']}]}
            path.write_text(json.dumps(data))
            cases=list(load_manifest(path)); self.assertEqual(len(cases),4); self.assertEqual(cases[0]['args'],['--seed','2','--alignment-weight','0'])
            data['cases'][0]['args']=['--seed','${missing}']; path.write_text(json.dumps(data))
            with self.assertRaises(ValueError): list(load_manifest(path))

if __name__=='__main__': unittest.main()
