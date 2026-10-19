"""Exercise observation lifecycle using API doubles, not a live ROS integration test."""
import importlib
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

from wheeltec_autorace_evaluation.core import content_digest


class FakeNode:
    def __init__(self, name):
        self.published_topics = []

    def create_subscription(self, *args):
        return None

    def create_publisher(self, kind, topic, qos):
        self.published_topics.append(topic)
        return NS(publish=lambda msg: None)

    def create_service(self, *args):
        return None

    def create_timer(self, *args):
        return None

    def get_logger(self):
        return NS(info=lambda msg: None)


def load_adapter():
    definitions = {
        'rclpy': {}, 'rclpy.node': {'Node': FakeNode},
        'rclpy.executors': {'ExternalShutdownException': type('ExternalShutdownException', (Exception,), {})},
        'geometry_msgs': {}, 'geometry_msgs.msg': {'PoseStamped': object},
        'std_msgs': {}, 'std_msgs.msg': {'String': NS},
        'std_srvs': {}, 'std_srvs.srv': {'Trigger': object},
    }
    modules = {}
    for name, attributes in definitions.items():
        modules[name] = ModuleType(name)
        modules[name].__dict__.update(attributes)
    with patch.dict(sys.modules, modules):
        return importlib.import_module('wheeltec_autorace_evaluation.node')


class LifecycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.adapter = load_adapter()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        data = {'schema': 'wheeltec_reference_v1',
                'centerline_world_xy_m': [[0, 0], [10, 0], [10, 10], [0, 10]],
                'lane_width_m': .65, 'line_width_m': .04, 'gate_world_xy_m': [5, 0],
                'frame_id': 'world/test', 'world_spawn_pose': [1, 0, .15, 0, 0, 0],
                'assets': {}}
        data['reference_id'] = content_digest(data)
        path = self.root/'reference.json'
        path.write_text(json.dumps(data))
        self.node = self.adapter.Observer(NS(reference=str(path), max_gap=.25, max_speed=3.,
                                           stop_speed=.02, stop_duration=.5, direction=1,
                                           start_radius=.25, output=str(self.root/'results'),
                                           label='pid_test', controller_config=None, duration=.2))

    def tearDown(self):
        self.node.finish('test_cleanup')
        self.temp.cleanup()

    def pose(self, t, x=1., frame='world/test'):
        sec = int(t)
        msg = NS(header=NS(frame_id=frame, stamp=NS(sec=sec, nanosec=round((t-sec)*1e9))),
                 pose=NS(position=NS(x=x, y=0., z=.15), orientation=NS(x=0., y=0., z=0., w=1.)))
        self.node.receive(msg)

    def test_requires_pose_and_spawn(self):
        self.assertFalse(self.node.start(None, NS()).success)
        self.pose(1, frame='wrong')
        self.assertFalse(self.node.start(None, NS()).success)
        self.pose(1, x=3.)
        self.assertFalse(self.node.start(None, NS()).success)

    def test_arm_motion_save_and_no_commands(self):
        self.pose(1.)
        self.assertTrue(self.node.start(None, NS()).success)
        self.pose(1.1)
        self.pose(1.2)
        self.assertEqual(self.node.state, 'ARMED')
        self.pose(1.3, 1.1)
        self.assertEqual(self.node.state, 'RECORDING')
        self.pose(1.4, 1.2)
        self.pose(1.5, 1.3)
        self.assertEqual(self.node.state, 'DONE')
        summary = json.loads((self.node.folder/'summary.json').read_text())
        self.assertTrue(summary['has_metrics'])
        self.assertTrue(summary['continuous_trial'])
        self.assertAlmostEqual(summary['cte_rmse_time_m'], 0)
        self.assertAlmostEqual(summary['speed_mean_m_s'], 1.)
        self.assertAlmostEqual(summary['observed_span_sim_s'], .2)
        self.assertEqual(summary['finish_reason'], 'duration_reached')
        self.assertIn('quality', (self.node.folder/'trajectory.csv').read_text())
        self.assertEqual(self.node.published_topics, ['/evaluation/status'])

    def test_empty_stop_reports_null(self):
        self.pose(1.)
        self.node.start(None, NS())
        self.assertTrue(self.node.stop(None, NS()).success)
        data = json.loads((self.node.folder/'summary.json').read_text())
        self.assertIsNone(data['cte_rmse_time_m'])
        self.assertFalse(data['has_metrics'])

    def test_reset_finalizes_trial(self):
        self.pose(1.)
        self.node.start(None, NS())
        self.pose(1.1)
        self.pose(1.2, 1.1)
        self.pose(0., 1.)
        self.assertEqual(self.node.state, 'DONE')
        data = json.loads((self.node.folder/'summary.json').read_text())
        self.assertEqual(data['finish_reason'], 'simulation_time_reset')
        self.assertFalse(data['continuous_trial'])


if __name__ == '__main__':
    unittest.main()
