import unittest
import numpy as np
from swarm_analysis.metrics import frame_metrics, route_distances

class MetricsTests(unittest.TestCase):
    def test_known_geometry_and_polarization(self):
        group={"uid":0,"config":{"body_radius":.1},"command":{"type":"flock"}}
        agents=[{"uid":i,"group_uid":0,"position":[x,0],"velocity":[v,0],"state":"moving"} for i,(x,v) in enumerate([(-1,1),(1,-1)])]
        row=frame_metrics({"groups":[group],"agents":agents,"steps":0,"time":0})[0]
        self.assertEqual(row["spread"],1); self.assertEqual(row["nearest_neighbor"],2); self.assertEqual(row["polarization"],0)
        self.assertAlmostEqual(row["min_body_gap"],1.8)
        self.assertEqual(route_distances(np.array([[1.,1.]]),[[0,0],[2,0]])[0],1)
        for a in agents: a["velocity"]=[0,0]
        self.assertIsNone(frame_metrics({"groups":[group],"agents":agents,"steps":0,"time":0})[0]["polarization"])

if __name__ == '__main__': unittest.main()
