"""ROS observation adapter; never publishes motion commands or changes parameters."""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .core import Metrics, Reference, Settings, digest_file, wrap


class Observer(Node):
    def __init__(self, args):
        super().__init__('tracking_evaluator')
        self.args = args
        self.reference = Reference.load(args.reference)
        self.settings = Settings(args.max_gap, args.max_speed, args.stop_speed,
                                 args.stop_duration, args.direction)
        self.metrics = Metrics(self.reference, self.settings)
        self.state = 'IDLE'
        self.last_pose = None
        self.last_receipt = None
        self.problem = None
        self.folder = None
        self.stream = None
        self.writer = None
        self.start_wall = None
        self.arm_previous = None
        self.manifest = None
        self.create_subscription(PoseStamped, '/evaluation/ground_truth_pose', self.receive, 100)
        self.status_pub = self.create_publisher(String, '/evaluation/status', 10)
        self.create_service(Trigger, '/evaluation/start', self.start)
        self.create_service(Trigger, '/evaluation/stop', self.stop)
        self.create_timer(1.0, self.status)
        self.get_logger().info('Observer ready; wait for pose, then call /evaluation/start. '
                               'Controller must be enabled separately.')

    def start(self, request, response):
        if self.state in ('ARMED', 'RECORDING'):
            response.success = False
            response.message = 'Already armed/recording'
            return response
        if self.last_receipt is None or time.monotonic() - self.last_receipt > 1.0:
            response.success = False
            response.message = 'No recent valid ground-truth pose; check adapter/world name'
            return response
        try:
            self.reference = Reference.load(self.args.reference)  # Check asset hashes on every trial.
            p = self.last_pose
            spawn = self.reference.data['world_spawn_pose']
            if math.hypot(p[1]-spawn[0], p[2]-spawn[1]) > self.args.start_radius:
                raise ValueError('Robot is not at saved spawn. Restart the SAME simulation for a comparable trial.')
            if abs(wrap(p[3]-spawn[5])) > 0.35:
                raise ValueError('Initial heading differs from saved spawn by >0.35 rad')
            if abs(self.reference.project(p[1], p[2])['cte_m']) > self.reference.inner_half_width:
                raise ValueError('Initial reference point is outside the lane')
            output = Path(self.args.output).expanduser().resolve()
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
            label = ''.join(c if c.isalnum() or c in '-_' else '_' for c in self.args.label)
            self.folder = output / (stamp + '_' + label)
            self.folder.mkdir(parents=True, exist_ok=False)
            shutil.copyfile(self.args.reference, self.folder / 'reference.json')
            config = None
            if self.args.controller_config:
                path = Path(self.args.controller_config).expanduser().resolve(strict=True)
                shutil.copyfile(path, self.folder / 'controller_config.yaml')
                config = {'path': str(path), 'sha256': digest_file(path),
                          'note': 'File snapshot only; runtime overrides must be documented separately'}
            self.manifest = {'controller_label': self.args.label, 'controller_config': config,
                             'armed_at_utc': stamp, 'requested_duration_sim_s': self.args.duration,
                             'armed_pose_t_x_y_yaw': list(p),
                             'start_rule': 'first observed planar speed exceeding stop_speed after arming',
                             'reference_file_sha256': digest_file(self.args.reference),
                             'assets_verified_at_start': True}
            self.metrics = Metrics(self.reference, self.settings)
            self.stream = (self.folder / 'trajectory.csv').open('x', newline='')
            self.writer = None
            self.arm_previous = None
            self.start_wall = None
            self.state = 'ARMED'
            self.problem = None
            self.write_summary('ARMED')
            response.success = True
            response.message = f'Armed; starts on motion, saves to {self.folder}'
            self.get_logger().info(response.message)
        except (ValueError, OSError, KeyError) as exc:
            if self.stream:
                self.stream.close()
                self.stream = None
            self.state = 'ERROR'
            response.success = False
            response.message = str(exc)
        return response

    def stop(self, request, response):
        response.success = self.state in ('ARMED', 'RECORDING')
        if response.success:
            self.finish('manual_stop')
            response.message = f'Saved: {self.folder}. Robot control is unaffected.'
        else:
            response.message = 'No active measurement'
        return response

    def receive(self, msg):
        if msg.header.frame_id != self.reference.data['frame_id']:
            self.problem = 'FRAME_MISMATCH'
            return
        p, q = msg.pose.position, msg.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if not all(math.isfinite(v) for v in values) or not 0.9 < n < 1.1:
            self.problem = 'INVALID_POSE'
            if self.state == 'RECORDING':
                self.metrics.add(float('nan'), 0., 0., 0.)
            return
        qx, qy, qz, qw = (v/math.sqrt(n) for v in (q.x, q.y, q.z, q.w))
        yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        current = (t, p.x, p.y, yaw)
        self.last_pose, self.last_receipt = current, time.monotonic()
        self.problem = None
        if self.state == 'ARMED':
            previous, self.arm_previous = self.arm_previous, current
            if previous is None:
                return
            dt = t - previous[0]
            if not 0 < dt <= self.settings.max_gap_s:
                return
            speed = math.hypot(p.x-previous[1], p.y-previous[2])/dt
            if not self.settings.stop_speed_m_s < speed <= self.settings.max_plausible_speed_m_s:
                return
            self.state = 'RECORDING'
            self.start_wall = time.monotonic()
            self.record(self.metrics.add(*previous))
            self.get_logger().info('Motion detected; measurement started in simulation time')
        if self.state == 'RECORDING':
            row = self.metrics.add(*current)
            self.record(row)
            if self.metrics.time_reset:
                self.finish('simulation_time_reset')
            elif self.args.duration > 0 and t - self.metrics.first_t + 1e-9 >= self.args.duration:
                self.finish('duration_reached')

    def record(self, row):
        if row is None:
            return
        if self.writer is None:
            self.writer = csv.DictWriter(self.stream, fieldnames=list(row))
            self.writer.writeheader()
        self.writer.writerow(row)

    def write_summary(self, reason):
        summary = self.metrics.summary()
        summary.update(self.manifest or {})
        summary['finish_reason'] = reason
        summary['wall_recording_s'] = time.monotonic()-self.start_wall if self.start_wall else None
        summary['duration_overshoot_sim_s'] = (
            max(0., summary['observed_span_sim_s']-self.args.duration)
            if self.args.duration > 0 else None)
        path = self.folder / 'summary.json'
        temp = path.with_suffix('.tmp')
        temp.write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
        temp.replace(path)

    def finish(self, reason):
        if self.state not in ('ARMED', 'RECORDING'):
            return
        if self.stream:
            self.stream.flush()
            self.stream.close()
            self.stream = None
        self.write_summary(reason)
        self.state = 'DONE'
        self.get_logger().info(f'Measurement saved: {self.folder}; reason={reason}; '
                               'the robot is still controlled by its controller')

    def status(self):
        age = None if self.last_receipt is None else time.monotonic()-self.last_receipt
        quality = self.problem or ('WAITING_FOR_POSE' if age is None else
                                   'NO_RECENT_POSE_OR_SIM_PAUSED' if age > 1.0 else 'POSE_OK')
        payload = {'state': self.state, 'input': quality, 'last_pose_wall_age_s': age,
                   'output': str(self.folder) if self.folder else None,
                   'evaluated_time_sim_s': self.metrics.duration,
                   'eligible_laps': len(self.metrics.laps)}
        self.status_pub.publish(String(data=json.dumps(payload)))
        if self.state == 'RECORDING':
            self.stream.flush()
            self.write_summary('IN_PROGRESS')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', required=True)
    parser.add_argument('--output', default=str(Path.home()/'wheeltec_results'))
    parser.add_argument('--label', required=True, help='Metadata only, e.g. pid_trial01')
    parser.add_argument('--controller-config')
    parser.add_argument('--duration', type=float, default=60., help='Simulation seconds; 0 = manual stop')
    parser.add_argument('--max-gap', type=float, default=0.25)
    parser.add_argument('--max-speed', type=float, default=3.)
    parser.add_argument('--stop-speed', type=float, default=0.02)
    parser.add_argument('--stop-duration', type=float, default=0.5)
    parser.add_argument('--start-radius', type=float, default=0.25)
    parser.add_argument('--direction', type=int, choices=(-1, 1), default=1)
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.duration) or args.duration < 0 or not math.isfinite(args.start_radius) or args.start_radius <= 0:
        parser.error('duration must be finite and >=0; start-radius must be finite and >0')
    args.reference = str(Path(args.reference).expanduser().resolve())
    rclpy.init(args=ros_args)
    node = None
    try:
        node = Observer(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.finish('process_shutdown')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
