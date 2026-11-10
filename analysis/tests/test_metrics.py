import unittest
import json
import numpy as np
from swarm_analysis.metrics import frame_metrics, route_distances

class MetricsTests(unittest.TestCase):
    def test_components_elongation_and_route_occupancy(self):
        group = {"uid": 0, "config": {"body_radius": .1},
                 "command": {"type": "route", "points": [[0, 0], [5, 0]], "radius": .5}}
        positions = [[0, 0], [.5, 0], [4, .25], [4, 1.5]]
        agents = [{"uid": i, "group_uid": 0, "position": p, "velocity": [1, 0], "state": "moving"} for i, p in enumerate(positions)]
        frame = {"groups": [group], "agents": agents, "steps": 0, "time": 0}
        row = frame_metrics(frame, link_distance=1.0)[0]
        self.assertEqual((row["component_count"], row["largest_component_fraction"]), (3, .5))
        self.assertEqual(row["route_inside_fraction"], .75)
        self.assertGreater(row["elongation"], 1)
        self.assertEqual(frame_metrics(frame, link_distance=4.0)[0]["component_count"], 1)
        skipped = frame_metrics(frame, pair_limit=1)[0]
        self.assertEqual(json.loads(skipped["undefined_reasons"])["largest_component_fraction"], "pair_limit")
        frame['agents'] = agents[:2]
        self.assertIsNone(frame_metrics(frame)[0]['elongation'])
        self.assertEqual(json.loads(frame_metrics(frame)[0]['undefined_reasons'])['elongation'], 'degenerate_transverse_spread')
        frame['agents'] = agents[:1]
        self.assertEqual(frame_metrics(frame)[0]['largest_component_fraction'], 1)
        frame['agents'][0]['state'] = 'terminated'
        self.assertIsNone(frame_metrics(frame)[0]['largest_component_fraction'])

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
