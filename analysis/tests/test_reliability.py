"""Failure paths and sampled-trace contracts, using the real exporter where relevant."""
import json
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch, MagicMock
from swarm_analysis.analyzer import analyze, write_json
from swarm_analysis.reader import iter_run, strict_json
from swarm_analysis.runner import run
from swarm_analysis.report import report
from swarm_analysis.metrics import frame_metrics, arrival_thresholds
from swarm_analysis.experiments import load_manifest

ROOT = Path(__file__).resolve().parents[2]
BINARY = ROOT / 'target/release/simulation.exe'


class ReliabilityTests(unittest.TestCase):
    def setUp(self):
        scratch = ROOT / "work/analysis"
        scratch.mkdir(parents=True, exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(dir=scratch)
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)

    def fixture(self, steps=12, interval=1, args=None):
        if not BINARY.exists():
            self.skipTest('build release simulation first')
        folder = self.path / 'run'
        folder.mkdir()
        argv = [str(BINARY), '--steps', str(steps), '--trace-every', str(interval),
                '--trace', str(folder/'trace.jsonl'), *(args or ['--agents', '2'])]
        result = subprocess.run(argv, capture_output=True, check=True)
        (folder/'stdout.json').write_bytes(result.stdout)
        write_json(folder/'metadata.json', {'argv': argv, 'exit_code': 0, 'process_status': 'process_complete',
                   'status': 'process_complete', 'case': 'test', 'params': {}, 'process_seconds': 0.125, 'run_hash': 'fixture'})
        return folder

    def manifest(self, code=None, scene=None, second=False):
        case = {'name': 'test', 'args': ['-c', code or 'pass'], 'grid': {}}
        if scene: case['scene'] = scene
        cases = [case]
        if second: cases.append({**case, 'name': 'second'})
        path = self.path/'input.json'
        write_json(path, {'schema_version': 1, 'name': 'test', 'cases': cases})
        return path

    def test_sparse_trace_final_frame_and_events(self):
        folder = self.fixture(steps=103, interval=7, args=['--group','-3','0','1','0.001','7','boids','--gather','0','0','1'])
        frames = list(iter_run(folder))
        self.assertEqual([f['steps'] for f in frames], [*range(0,104,7),103])
        summary = json.loads((folder/'stdout.json').read_text())
        self.assertEqual([e for f in frames for e in f['events']], summary['events'])
        with patch('swarm_analysis.analyzer.figures'):
            analyze(folder)
        group = json.loads((folder/'summary.json').read_text())['groups']['0']
        self.assertTrue(group['time_to_90_interval']['sparse'])
        self.assertEqual(group['time_to_90_interval']['source'], 'state_events')
        self.assertLessEqual(group['time_to_90_interval']['upper']-group['time_to_90_interval']['lower'], .7+1e-9)

    def test_arrival_event_threshold_survives_between_snapshot_reversals(self):
        initial = {'groups': [{'uid': 7, 'command': {'type': 'gather'}}],
                   'agents': [{'uid': 9, 'group_uid': 7, 'state': 'moving'}]}
        events = [{'step': 2, 'type': 'state_changed', 'agent_uids': [9], 'from': 'moving', 'to': 'arrived'},
                  {'step': 3, 'type': 'state_changed', 'agent_uids': [9], 'from': 'arrived', 'to': 'moving'}]
        self.assertEqual(arrival_thresholds(initial, events), {7: 2})

    def test_arrival_thresholds_keep_terminated_in_original_denominator(self):
        initial = {'groups': [{'uid': 7, 'command': {'type': 'gather'}}],
                   'agents': [{'uid': i, 'group_uid': 7, 'state': 'moving'} for i in range(10)]}
        events = [{'step': 2 if i < 5 else 4, 'type': 'state_changed', 'agent_uids': [i],
                   'from': 'moving', 'to': 'arrived'} for i in range(9)]
        events.append({'step': 6, 'type': 'state_changed', 'agent_uids': [9], 'from': 'moving', 'to': 'terminated'})
        self.assertEqual(arrival_thresholds(initial, events, .5), {7: 2})
        self.assertEqual(arrival_thresholds(initial, events, .9), {7: 4})
        self.assertEqual(arrival_thresholds(initial, events, 1), {})

    def test_plot_time_and_failure_do_not_change_process_time(self):
        folder = self.fixture()
        def slow_plot(*args): time.sleep(.05)
        with patch('swarm_analysis.analyzer.figures', side_effect=slow_plot):
            analyze(folder)
        meta = json.loads((folder/'metadata.json').read_text())
        self.assertEqual(meta['process_seconds'], .125)
        self.assertGreaterEqual(meta['plotting_seconds'], .04)
        with patch('swarm_analysis.analyzer.figures', side_effect=RuntimeError('plot failed')):
            with self.assertRaises(RuntimeError): analyze(folder)
        self.assertEqual(json.loads((folder/'metadata.json').read_text())['status'], 'analysis_failed')
        self.assertFalse((folder/'summary.json').exists())
        self.assertFalse((folder/'metrics.csv').exists())
        report(self.path)
        self.assertNotIn('![Trajectories]', (self.path/'report.md').read_text())

    def test_metadata_horizon_and_process_outcome_are_required(self):
        folder = self.fixture()
        meta = json.loads((folder/'metadata.json').read_text())
        meta['argv'][meta['argv'].index('--steps')+1] = '100'
        write_json(folder/'metadata.json', meta)
        with self.assertRaises(ValueError): list(iter_run(folder))
        meta['exit_code'] = 1
        write_json(folder/'metadata.json', meta)
        with self.assertRaises(ValueError): analyze(folder)
        self.assertEqual(json.loads((folder/'metadata.json').read_text())['status'], 'invalid_output')

    def test_nonfinite_numbers_parameters_and_event_references(self):
        for text in ('{"x":1e999}', '{"x":NaN}', '{"x":[Infinity]}'):
            with self.assertRaises(ValueError): strict_json(text)
        folder = self.fixture()
        originals = (folder/'trace.jsonl').read_text().splitlines()
        for mutate in (
            lambda f: f['groups'][0]['config'].update(body_radius=-1),
            lambda f: f['events'][0].update(time=100),
            lambda f: f['events'][0].update(agent_uids=[999]),
            lambda f: f['agents'][0].update(uid=True),
        ):
            frames = [json.loads(line) for line in originals]
            mutate(frames[0])
            (folder/'trace.jsonl').write_text('\n'.join(json.dumps(f) for f in frames)+'\n')
            with self.assertRaises(ValueError): list(iter_run(folder))
        (folder/'trace.jsonl').write_text('\n'.join(originals[:-1])+'\n')
        with self.assertRaises(ValueError): list(iter_run(folder))
        (folder/'trace.jsonl').write_text('{broken}\n')
        with self.assertRaises(ValueError): list(iter_run(folder))

    def test_old_and_new_models_load_but_cannot_be_mixed_in_one_trace(self):
        folder = self.fixture()
        path = folder/'trace.jsonl'
        frames = [json.loads(line) for line in path.read_text().splitlines()]
        summary_path = folder/'stdout.json'
        summary = json.loads(summary_path.read_text())
        self.assertEqual(summary['model'], 'boids-2d-v14')
        self.assertEqual(len(list(iter_run(folder))), len(frames))
        for frame in frames: frame['model'] = 'boids-2d-v13'
        summary['model'] = 'boids-2d-v13'
        path.write_text('\n'.join(json.dumps(frame) for frame in frames) + '\n')
        write_json(summary_path, summary)
        self.assertEqual(len(list(iter_run(folder))), len(frames))
        frames[1]['model'] = 'boids-2d-v14'
        path.write_text('\n'.join(json.dumps(frame) for frame in frames) + '\n')
        with self.assertRaises(ValueError): list(iter_run(folder))

    def test_pair_metric_reasons_and_all_dead(self):
        folder = self.fixture()
        frame = next(iter_run(folder))
        row = frame_metrics(frame, pair_limit=1)[0]
        self.assertEqual(json.loads(row['undefined_reasons'])['min_body_gap'], 'pair_limit')
        self.assertEqual(row['min_body_gap_scope'], 'world')
        with patch('swarm_analysis.analyzer.frame_metrics', side_effect=lambda f, **kw: frame_metrics(f, pair_limit=1, **kw)), patch('swarm_analysis.analyzer.figures'):
            analyze(folder)
        self.assertTrue(json.loads((folder/'validation.json').read_text())['warnings'])
        for agent in frame['agents']: agent['state'] = 'terminated'
        row = frame_metrics(frame)[0]
        self.assertEqual(row['terminated'], 1)
        self.assertIsNone(row['spread'])

    def test_multiple_groups_use_uid_and_world_gap(self):
        frame = next(iter_run(self.fixture()))
        frame['groups'].append({**frame['groups'][0], 'uid': 42})
        frame['agents'][1]['group_uid'] = 42
        rows = frame_metrics(frame)
        self.assertEqual([r['group_uid'] for r in rows], [0, 42])
        self.assertEqual(rows[0]['min_body_gap'], rows[1]['min_body_gap'])
        self.assertTrue(all(r['nearest_neighbor'] is None for r in rows))

    def test_failed_process_timeout_invalid_output_and_missing_scene(self):
        for index, (code, timeout, scene, expected) in enumerate([
            ('import sys; sys.exit(3)', 10, None, 'process_failed'),
            ('import time; time.sleep(5)', .1, None, 'timeout'),
            ('print("invalid")', 10, None, 'invalid_output'),
            ('pass', 10, 'missing.swarm', 'setup_failed'),
        ]):
            output = self.path/f'case{index}'
            self.assertEqual(run(self.manifest(code, scene), sys.executable, output, timeout), 1)
            meta = json.loads(next(output.glob('*/metadata.json')).read_text())
            self.assertEqual(meta['status'], expected)
            aggregate = json.loads((output/'aggregate.json').read_text())
            self.assertEqual(aggregate[0]['failed'], 1)
            self.assertIsNone(aggregate[0]['conditional_completion_rate'])

    def test_interruption_preserves_remaining_trials(self):
        process = MagicMock()
        process.wait.side_effect = [KeyboardInterrupt(), 0]
        process.__enter__.return_value = process
        output = self.path/'interrupted'
        with patch('swarm_analysis.runner.subprocess.Popen', return_value=process), patch('swarm_analysis.runner.subprocess.run', side_effect=OSError):
            self.assertEqual(run(self.manifest(second=True), sys.executable, output, 10), 130)
        process.kill.assert_called_once()
        statuses = [json.loads(p.read_text())['status'] for p in sorted(output.glob('*/metadata.json'))]
        self.assertEqual(statuses, ['interrupted', 'not_started'])

    def test_invalid_timeout_and_duplicate_trials_are_rejected(self):
        for timeout in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError): run('unused', 'unused', self.path/'out', timeout)
        path = self.manifest()
        data = json.loads(path.read_text())
        data['cases'][0].update(args=['--seed','${seed}'], grid={'seed':[1,1.0]})
        write_json(path, data)
        with self.assertRaises(ValueError): list(load_manifest(path))

    def test_component_radius_validation_and_persistence(self):
        folder = self.fixture()
        with patch('swarm_analysis.analyzer.figures'):
            analyze(folder, component_radius=2.0)
        self.assertEqual(json.loads((folder/'metadata.json').read_text())['analysis_settings']['component_radius_m'], 2.0)
        self.assertEqual(json.loads((folder/'summary.json').read_text())['analysis_settings']['component_radius_m'], 2.0)
        with self.assertRaises(ValueError): analyze(folder, component_radius=float('nan'))
        self.assertTrue((folder/'summary.json').exists())
        with patch('swarm_analysis.analyzer.figures'): analyze(folder)
        self.assertEqual(json.loads((folder/'summary.json').read_text())['analysis_settings']['component_radius_m'], 2.0)

    def test_standalone_trace_without_metadata_warns(self):
        folder = self.fixture()
        (folder/'metadata.json').unlink()
        with patch('swarm_analysis.analyzer.figures'): analyze(folder)
        warnings = json.loads((folder/'validation.json').read_text())['warnings']
        self.assertTrue(any('No process metadata' in warning for warning in warnings))
        self.assertFalse((folder/'metadata.json').exists())

    def test_report_does_not_mix_connectivity_radii(self):
        first = self.fixture()
        with patch('swarm_analysis.analyzer.figures'): analyze(first, component_radius=1.0)
        second = self.path/'another'
        second.mkdir()
        for filename in ('metadata.json', 'summary.json', 'validation.json', 'stdout.json'):
            (second/filename).write_bytes((first/filename).read_bytes())
        meta = json.loads((second/'metadata.json').read_text())
        meta['analysis_settings']['component_radius_m'] = 2.0
        write_json(second/'metadata.json', meta)
        summary = json.loads((second/'summary.json').read_text())
        summary['analysis_settings']['component_radius_m'] = 2.0
        write_json(second/'summary.json', summary)
        report(self.path)
        groups = json.loads((self.path/'aggregate.json').read_text())
        self.assertEqual({row['component_radius_m'] for row in groups}, {1.0, 2.0})
        self.assertTrue(all(row['planned'] == 1 for row in groups))

    def test_report_rejects_mixed_model_versions(self):
        first = self.fixture()
        with patch('swarm_analysis.analyzer.figures'): analyze(first)
        second = self.path/'another'
        second.mkdir()
        for filename in ('metadata.json', 'summary.json', 'validation.json', 'stdout.json'):
            (second/filename).write_bytes((first/filename).read_bytes())
        summary = json.loads((second/'summary.json').read_text())
        summary.pop('model')
        write_json(second/'summary.json', summary)
        meta = json.loads((second/'metadata.json').read_text())
        meta.pop('model')
        write_json(second/'metadata.json', meta)
        stdout = json.loads((second/'stdout.json').read_text())
        stdout['model'] = 'boids-2d-v13'
        write_json(second/'stdout.json', stdout)
        with self.assertRaisesRegex(ValueError, 'different models'): report(self.path)

    def test_bounded_plot_buffer(self):
        folder = self.fixture(steps=2100, args=['--agents','1'])
        with patch('swarm_analysis.analyzer.figures') as plots:
            analyze(folder)
        frames = plots.call_args.args[0]
        self.assertLessEqual(len(frames), 1000)
        self.assertEqual((frames[0]['steps'], frames[-1]['steps']), (0,2100))
        self.assertEqual(len((folder/'metrics.csv').read_text().splitlines()), 2102)

    def test_inapplicable_unfinished_and_failed_are_distinct(self):
        # Use real metrics summaries; only the report's trials are synthetic.
        folder = self.fixture(steps=0, args=['--agents','1'])
        with patch('swarm_analysis.analyzer.figures'): analyze(folder)
        report(self.path)
        result = json.loads((self.path/'aggregate.json').read_text())[0]
        self.assertEqual(result['completion_reason'], 'not_applicable')
        self.assertIsNone(result['conditional_completion_rate'])
        summary = json.loads((folder/'summary.json').read_text())
        summary['groups']['0']['time_to_90_reason'] = 'not_reached_by_horizon'
        write_json(folder/'summary.json', summary)
        report(self.path)
        result = json.loads((self.path/'aggregate.json').read_text())[0]
        self.assertEqual(result['conditional_completion_rate'], 0)
        self.assertEqual(result['applicable_valid'], 1)
        meta = json.loads((folder/'metadata.json').read_text())
        meta['status'] = 'process_failed'
        write_json(folder/'metadata.json', meta)
        report(self.path)
        result = json.loads((self.path/'aggregate.json').read_text())[0]
        self.assertEqual(result['failed'], 1)
        self.assertIsNone(result['conditional_completion_rate'])
