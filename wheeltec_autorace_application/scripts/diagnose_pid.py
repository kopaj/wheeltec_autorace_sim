#!/usr/bin/env python3
"""Read-only ROS 2 command-envelope observer. Never publishes a Twist.

Geometry comes from the supplied model SDF. This checks requested commands,
not actual vehicle motion or the plugin's instantaneous internal state.
Run --self-test without ROS; otherwise source ROS 2 Humble before running.
"""
import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET


def curvature_limit(wheelbase, steering_limit):
    if not (math.isfinite(wheelbase) and wheelbase > 0):
        raise ValueError("wheel_base must be positive and finite")
    if not (math.isfinite(steering_limit) and 0 < steering_limit < math.pi / 2):
        raise ValueError("steering_limit must be finite and between 0 and pi/2")
    # Fortress ign-gazebo6: minimum radius = wheel_base / sin(steering_limit).
    return math.sin(steering_limit) / wheelbase


def analyze_command(v, omega, wheelbase, steering_limit):
    limit = curvature_limit(wheelbase, steering_limit)
    if not (math.isfinite(v) and math.isfinite(omega)):
        return {"finite": False, "outside_geometric_envelope": None}
    max_omega = abs(v) * limit
    outside = abs(omega) > max_omega + 1e-9
    curvature = omega / v if v != 0 else None
    return {
        "finite": True,
        "v_command_m_s": v,
        "omega_command_rad_s": omega,
        "curvature_command_1_m": (
            curvature if curvature is not None and math.isfinite(curvature) else None),
        "equivalent_bicycle_angle_rad": (
            math.atan(wheelbase * curvature) if curvature is not None else None),
        "omega_envelope_rad_s": max_omega,
        "outside_geometric_envelope": outside,
        "zero_speed_turn_request": v == 0 and omega != 0,
    }


def read_geometry(path):
    root = ET.parse(path).getroot()
    plugins = [p for p in root.iter("plugin")
               if "AckermannSteering" in p.get("name", "")]
    if len(plugins) != 1:
        raise ValueError("Expected exactly one AckermannSteering plugin in robot SDF")
    plugin = plugins[0]
    wheelbase = float(plugin.findtext("wheel_base"))
    steering_limit = float(plugin.findtext("steering_limit"))
    curvature_limit(wheelbase, steering_limit)
    return wheelbase, steering_limit


class CommandBreakdown:
    """Receipt-based summaries; these labels never assert a controller mode."""

    GAP_S = 0.20

    def __init__(self):
        self.bands = {name: {"nonzero_samples": 0, "outside_samples": 0,
                             "largest_excess_sample": None}
                      for name in ("abs_v=0", "0<abs_v<0.05",
                                   "0.05<=abs_v<0.18", "abs_v>=0.18")}
        self.last_receipt = self.last_zero = None
        self.previous_was_zero = False
        self.zero_samples = self.zero_runs = self.after_zero = 0
        self.gaps = 0
        self.max_gap = 0.0
        self.episode = None
        self.episode_count = 0
        self.top_episodes = []

    def finish_episode(self):
        if self.episode is None:
            return
        episode = self.episode
        episode["receipt_span_wall_s"] = episode["last_wall_s"] - episode["first_wall_s"]
        self.episode_count += 1
        self.top_episodes.append(dict(episode))
        self.top_episodes.sort(key=lambda e: e["peak_excess_rad_s"], reverse=True)
        del self.top_episodes[5:]
        self.episode = None

    def observe(self, result, elapsed):
        gap = elapsed - self.last_receipt if self.last_receipt is not None else None
        if gap is not None:
            self.max_gap = max(self.max_gap, gap)
        if gap is not None and gap > self.GAP_S:
            self.gaps += 1
            self.finish_episode()
            self.previous_was_zero = False
        self.last_receipt = elapsed
        if not result["finite"]:
            self.finish_episode()
            self.previous_was_zero = False
            return
        v, omega = result["v_command_m_s"], result["omega_command_rad_s"]
        if v == 0 and omega == 0:
            self.zero_samples += 1
            self.zero_runs += int(not self.previous_was_zero)
            self.previous_was_zero = True
            self.last_zero = elapsed
            self.finish_episode()
            return
        self.after_zero += int(self.previous_was_zero)
        self.previous_was_zero = False
        sample = dict(result)
        sample["receipt_wall_s_from_start"] = elapsed
        sample["since_last_observed_zero_command_wall_s"] = (
            elapsed - self.last_zero if self.last_zero is not None else None)
        excess = abs(omega) - result["omega_envelope_rad_s"]
        sample["yaw_envelope_excess_rad_s"] = excess
        speed = abs(v)
        band_name = ("abs_v=0" if speed == 0 else "0<abs_v<0.05" if speed < .05
                     else "0.05<=abs_v<0.18" if speed < .18 else "abs_v>=0.18")
        band = self.bands[band_name]
        band["nonzero_samples"] += 1
        band["outside_samples"] += int(result["outside_geometric_envelope"])
        if result["outside_geometric_envelope"]:
            peak = band["largest_excess_sample"]
            if peak is None or excess > peak["yaw_envelope_excess_rad_s"]:
                band["largest_excess_sample"] = sample
            if self.episode is None:
                self.episode = {"first_wall_s": elapsed, "last_wall_s": elapsed,
                                "samples": 0, "min_abs_v_m_s": speed,
                                "max_abs_v_m_s": speed, "peak_excess_rad_s": excess,
                                "peak_sample": sample}
            episode = self.episode
            episode["last_wall_s"] = elapsed
            episode["samples"] += 1
            episode["min_abs_v_m_s"] = min(episode["min_abs_v_m_s"], speed)
            episode["max_abs_v_m_s"] = max(episode["max_abs_v_m_s"], speed)
            if excess > episode["peak_excess_rad_s"]:
                episode["peak_excess_rad_s"] = excess
                episode["peak_sample"] = sample
        else:
            self.finish_episode()

    def report(self):
        # Called once at the end of observation; close any boundary-truncated run.
        self.finish_episode()
        return {
            "speed_bands_m_s": {
                name: {**band, "outside_fraction": (
                    band["outside_samples"] / band["nonzero_samples"]
                    if band["nonzero_samples"] else None)}
                for name, band in self.bands.items()},
            "zero_command_samples": self.zero_samples,
            "observed_zero_command_runs": self.zero_runs,
            "nonzero_transitions_after_observed_zero": self.after_zero,
            "receipt_gaps_over_0_20_s": self.gaps,
            "max_receipt_gap_wall_s": self.max_gap,
            "outside_episode_count": self.episode_count,
            "top_5_episodes_by_yaw_excess": self.top_episodes,
        }


def self_test():
    import unittest

    class EnvelopeTests(unittest.TestCase):
        def test_audited_limit(self):
            self.assertAlmostEqual(curvature_limit(.3187, .4), 1.2218962733)

        def test_straight_and_stop(self):
            for v in (0, .5, -.5):
                self.assertFalse(analyze_command(v, 0, .3187, .4)[
                    "outside_geometric_envelope"])

        def test_boundary_and_signs(self):
            bound = .15 * curvature_limit(.3187, .4)
            for v in (.15, -.15):
                for sign in (-1, 1):
                    self.assertFalse(analyze_command(v, sign * bound, .3187, .4)[
                        "outside_geometric_envelope"])
                    self.assertTrue(analyze_command(v, sign * (bound + .001), .3187, .4)[
                        "outside_geometric_envelope"])

        def test_audit_counterexamples(self):
            for v, w in ((.5, .7), (.18, .7), (.15, .4), (.15, .198)):
                self.assertTrue(analyze_command(v, w, .3187, .4)[
                    "outside_geometric_envelope"])
            self.assertFalse(analyze_command(.29, .3, .3187, .4)[
                "outside_geometric_envelope"])

        def test_nonfinite_and_zero_velocity(self):
            for v, w in ((float("nan"), 0), (0, float("inf"))):
                self.assertFalse(analyze_command(v, w, .3187, .4)["finite"])
            result = analyze_command(0, .3, .3187, .4)
            self.assertTrue(result["outside_geometric_envelope"])
            self.assertIsNone(result["curvature_command_1_m"])
            json.dumps(analyze_command(1e-320, 1, .3187, .4), allow_nan=False)

        def test_invalid_geometry(self):
            for l, s in ((0, .4), (.3, 0), (.3, math.nan), (.3, 2)):
                with self.assertRaises(ValueError):
                    curvature_limit(l, s)

        def test_breakdown_and_stop_context(self):
            stats = CommandBreakdown()
            for t, v, w in ((0, 0, 0), (.025, .02, .18), (.050, .03, .18),
                            (.075, .15, .2), (.100, .5, 0), (.125, 0, 0)):
                stats.observe(analyze_command(v, w, .3187, .4), t)
            report = stats.report()
            self.assertEqual(report["zero_command_samples"], 2)
            self.assertEqual(report["nonzero_transitions_after_observed_zero"], 1)
            self.assertEqual(report["outside_episode_count"], 1)
            ep = report["top_5_episodes_by_yaw_excess"][0]
            self.assertEqual(ep["samples"], 3)
            self.assertAlmostEqual(ep["receipt_span_wall_s"], .05)
            self.assertEqual(report["speed_bands_m_s"]["0<abs_v<0.05"]["outside_samples"], 2)
            self.assertEqual(report["speed_bands_m_s"]["abs_v>=0.18"]["outside_fraction"], 0)
            self.assertAlmostEqual(ep["peak_sample"]["since_last_observed_zero_command_wall_s"], .025)

        def test_breakdown_gaps_nonfinite_and_boundaries(self):
            stats = CommandBreakdown()
            for t, v, w in ((0, 0, .1), (.3, .05, .1), (.325, .18, .3),
                            (.350, math.nan, 0), (.375, -.18, -.3)):
                stats.observe(analyze_command(v, w, .3187, .4), t)
            report = stats.report()
            self.assertEqual(report["outside_episode_count"], 3)
            self.assertEqual(report["receipt_gaps_over_0_20_s"], 1)
            self.assertEqual(report["speed_bands_m_s"]["abs_v=0"]["outside_samples"], 1)
            self.assertEqual(report["speed_bands_m_s"]["0.05<=abs_v<0.18"]["outside_samples"], 1)
            self.assertEqual(report["speed_bands_m_s"]["abs_v>=0.18"]["outside_samples"], 2)
            self.assertIsNone(report["top_5_episodes_by_yaw_excess"][0]["peak_sample"][
                "since_last_observed_zero_command_wall_s"])
            json.dumps(report, allow_nan=False)

    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(EnvelopeTests))
    return 0 if result.wasSuccessful() else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--model", type=Path, default=(
        Path(__file__).resolve().parents[2]
        / "wheeltec_autorace_description/models/roboworks/model.sdf"))
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if not (math.isfinite(args.seconds) and args.seconds > 0):
        parser.error("--seconds must be positive and finite")
    try:
        wheelbase, steering_limit = read_geometry(args.model)
    except (OSError, ET.ParseError, ValueError, TypeError) as exc:
        parser.error(f"Cannot read geometry: {exc}; supply --model /path/to/model.sdf")

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from geometry_msgs.msg import Twist
        from rosgraph_msgs.msg import Clock
    except ImportError as exc:
        parser.error(f"ROS 2 Python packages unavailable: {exc}. Source ROS 2 Humble.")

    rclpy.init(args=[])

    class Observer(Node):
        def __init__(self):
            super().__init__("pid_command_envelope_observer")
            self.start = time.monotonic()
            self.count = self.finite = self.moving = self.outside = self.bad = 0
            self.first = self.last = self.largest = None
            self.max_excess = -1.0
            self.first_clock = self.last_clock = None
            self.clock_reset = False
            self.observed_command_publishers = set()
            self.command_breakdown = CommandBreakdown()
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(Twist, args.cmd_topic, self.on_command, qos)
            self.create_subscription(Clock, "/clock", self.on_clock, qos)
            self.create_timer(1.0, self.observe_publishers)

        def observe_publishers(self):
            for info in self.get_publishers_info_by_topic(args.cmd_topic):
                self.observed_command_publishers.add(info.node_namespace.rstrip("/") + "/" + info.node_name)

        def on_clock(self, msg):
            now = (msg.clock.sec + msg.clock.nanosec * 1e-9, time.monotonic())
            if self.last_clock and now[0] < self.last_clock[0]:
                self.clock_reset = True
            if self.first_clock is None:
                self.first_clock = now
            self.last_clock = now

        def on_command(self, msg):
            now = time.monotonic()
            if self.first is None:
                self.first = now
            self.last = now
            self.count += 1
            result = analyze_command(msg.linear.x, msg.angular.z, wheelbase, steering_limit)
            self.command_breakdown.observe(result, now - self.start)
            if not result["finite"]:
                self.bad += 1
                return
            self.finite += 1
            moving = msg.linear.x != 0 or msg.angular.z != 0
            self.moving += int(moving)
            self.outside += int(result["outside_geometric_envelope"])
            excess = abs(msg.angular.z) - result["omega_envelope_rad_s"]
            if moving and excess > self.max_excess:
                self.max_excess = excess
                self.largest = result

        def report(self):
            self.observe_publishers()
            elapsed = time.monotonic() - self.start
            hz = ((self.count - 1) / (self.last - self.first)
                  if self.count > 1 and self.last > self.first else None)
            rtf = None
            if self.first_clock and self.last_clock and not self.clock_reset:
                ds = self.last_clock[0] - self.first_clock[0]
                dw = self.last_clock[1] - self.first_clock[1]
                if dw > 0:
                    rtf = ds / dw
            print(json.dumps({
                "diagnostic": "PID command envelope v2; observer only",
                "model_path": str(args.model.resolve()),
                "wheel_base_m": wheelbase,
                "steering_limit_parameter_rad": steering_limit,
                "fortress_curvature_limit_1_m": curvature_limit(wheelbase, steering_limit),
                "wall_observation_s": elapsed,
                "cmd_topic": args.cmd_topic,
                "publishers_seen": sorted(self.observed_command_publishers),
                "pid_attribution": (
                    "Only /pid_lane_controller observed; discovery is not per-message attribution"
                    if self.observed_command_publishers == {"/pid_lane_controller"}
                    else "Not established; verify controller and all command publishers"),
                "messages_received": self.count,
                "receipt_hz": hz,
                "nonfinite_messages": self.bad,
                "nonzero_finite_commands": self.moving,
                "outside_geometric_envelope_count": self.outside,
                "outside_fraction_of_nonzero_finite_commands": (
                    self.outside / self.moving if self.moving else None),
                "largest_yaw_envelope_excess_sample": self.largest,
                "clock_went_backwards": self.clock_reset,
                "simulation_to_wall_time_ratio": rtf,
                "command_breakdown": self.command_breakdown.report(),
                "limitations": [
                    "Counts describe received command messages, not time spent or distance travelled.",
                    "No measured wheel angle, velocity, plugin state or path error is observed.",
                    "Outside envelope is a command test, not proof of actual instantaneous saturation.",
                    "Fortress acceleration limits, steering dynamics and near-zero yaw handling are not simulated.",
                    "Discovery snapshots cannot rule out a short-lived additional publisher.",
                    "Speed bands are command ranges, not NORMAL/DEGRADED or measured motion states.",
                    "An episode groups consecutive received outside samples; a gap >0.20 s breaks it.",
                    "Episode span is first-to-last sample receipt time, not exact saturation duration.",
                    "Zero commands and their timing do not identify why the controller stopped.",
                ],
            }, ensure_ascii=False, indent=2, allow_nan=False))

    node = Observer()
    print(f"Observing {args.cmd_topic} for {args.seconds:g} wall seconds; no commands are sent.",
          flush=True)
    try:
        deadline = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.1, max(0, deadline - time.monotonic())))
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
