import math
import unittest
from wheeltec_autorace_evaluation.core import Metrics, Reference, Settings


def square():
    return Reference({'centerline_world_xy_m': [[0, 0], [10, 0], [10, 10], [0, 10]],
                      'lane_width_m': .65, 'line_width_m': .04,
                      'gate_world_xy_m': [5, 0]})


class GeometryTests(unittest.TestCase):
    def test_project_signed_and_endpoint(self):
        ref = square()
        self.assertAlmostEqual(ref.project(2, .1)['cte_m'], .1)
        self.assertAlmostEqual(ref.project(2, -.2)['cte_m'], -.2)
        p = ref.project(10.1, -.1)
        self.assertAlmostEqual(abs(p['cte_m']), math.sqrt(.02))
        self.assertAlmostEqual(ref.project(10, 3)['s_m'], 13.)

    def test_constant_error_time_weights(self):
        m = Metrics(square())
        for t in (0, .01, .07, .2, .3, .5):
            m.add(t, 1+t, .1, 0)
        s = m.summary()
        self.assertAlmostEqual(s['cte_rmse_time_m'], .1)
        self.assertAlmostEqual(s['cte_mae_time_m'], .1)
        self.assertAlmostEqual(s['cte_rmse_distance_m'], .1)
        self.assertAlmostEqual(s['speed_mean_m_s'], 1.)
        self.assertAlmostEqual(s['coverage_fraction'], 1.)

    def test_simulation_stamps_not_wall_clock(self):
        a, b = Metrics(square()), Metrics(square())
        for t in (0, .05, .15, .25):
            a.add(t, 1+t, .04, 0)
            b.add(t+100, 1+t, .04, 0)
        for key in ('cte_rmse_time_m', 'speed_mean_m_s', 'evaluated_time_sim_s'):
            self.assertAlmostEqual(a.summary()[key], b.summary()[key])

    def test_weighting_not_sample_average(self):
        m = Metrics(square())
        for t, e in ((0, 0), (.01, .01), (.2, .02)):
            m.add(t, 1+t, e, 0)
        expected = math.sqrt((.01*(0+.0001)/2 + .19*(.0001+.0004)/2)/.2)
        self.assertAlmostEqual(m.summary()['cte_rmse_time_m'], expected)

    def test_gap_excluded(self):
        m = Metrics(square())
        for t in (0, .1, 1., 1.1):
            m.add(t, 1+t, 0, 0)
        s = m.summary()
        self.assertAlmostEqual(s['evaluated_time_sim_s'], .2)
        self.assertAlmostEqual(s['coverage_fraction'], .2/1.1)
        self.assertEqual(s['quality']['gap_intervals'], 1)
        self.assertFalse(s['continuous_trial'])

    def test_teleport_duplicate_and_reset(self):
        m = Metrics(square())
        m.add(10., 1., 0., 0.)
        self.assertIsNone(m.add(10., 1., 0., 0.))
        self.assertEqual(m.add(10.1, 8., 0., 0.)['quality'], 'POSE_JUMP')
        self.assertIsNone(m.add(0., 1., 0., 0.))
        self.assertTrue(m.summary()['quality']['time_went_backwards'])
        self.assertFalse(m.summary()['has_metrics'])

    def test_no_data_is_null(self):
        s = Metrics(square()).summary()
        self.assertIsNone(s['cte_rmse_time_m'])
        self.assertIsNone(s['coverage_fraction'])
        self.assertFalse(s['has_metrics'])

    def test_nonfinite_breaks_continuity(self):
        m = Metrics(square())
        m.add(0, 1, 0, 0)
        m.add(.1, 1.1, 0, 0)
        self.assertIsNone(m.add(.2, float('nan'), 0, 0))
        m.add(.3, 1.3, 0, 0)
        m.add(.4, 1.4, 0, 0)
        self.assertAlmostEqual(m.summary()['evaluated_time_sim_s'], .2)
        self.assertFalse(m.summary()['continuous_trial'])

    def test_stop_requires_duration(self):
        m = Metrics(square())
        for i in range(11):
            m.add(i*.1, 1, 0, 0)
        self.assertEqual(m.summary()['stop_episodes'], 1)
        self.assertAlmostEqual(m.summary()['low_speed_time_s'], 1.)

    def test_boundary_is_reference_point(self):
        m = Metrics(square())
        m.add(0, 1, .32, 0)
        m.add(.1, 1.1, .32, 0)
        s = m.summary()
        self.assertEqual(s['reference_point_outside_lane_time_s'], 0)
        self.assertAlmostEqual(s['reference_point_beyond_inner_paint_edge_time_s'], .1)

    def test_heading_wrap(self):
        m = Metrics(square())
        m.add(0, 5, 10, -math.pi+.01)
        m.add(.1, 4.9, 10, math.pi-.01)
        self.assertAlmostEqual(m.summary()['heading_rmse_time_rad'], .01)

    def test_laps_partial_not_counted_and_reverse(self):
        for direction in (1, -1):
            m = Metrics(square(), Settings(direction=direction))
            # Start just after gate; 2.2 circuits contain only one gate-to-gate lap.
            for i in range(1761):
                s = (5 + direction*(.1+i*.05)) % 40
                if s < 10: x, y, yaw = s, 0, 0
                elif s < 20: x, y, yaw = 10, s-10, math.pi/2
                elif s < 30: x, y, yaw = 30-s, 10, math.pi
                else: x, y, yaw = 0, 40-s, -math.pi/2
                m.add(i*.05, x, y, yaw+(math.pi if direction < 0 else 0))
            self.assertEqual(len(m.laps), 1)
            self.assertAlmostEqual(m.laps[0], 40.)

    def test_gate_jitter_does_not_count_lap(self):
        m = Metrics(square())
        for i in range(100):
            m.add(i*.1, 5 + (.01 if i % 2 else -.01), 0, 0)
        self.assertEqual(m.laps, [])

    def test_invalid_reference(self):
        with self.assertRaises(ValueError):
            Reference({'centerline_world_xy_m': [[0, 0], [0, 0], [1, 0]],
                       'lane_width_m': .65, 'line_width_m': .04,
                       'gate_world_xy_m': [0, 0]})


if __name__ == '__main__':
    unittest.main()
