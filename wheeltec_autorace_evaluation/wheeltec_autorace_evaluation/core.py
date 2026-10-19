"""Pure numerical core. Units: metres, radians, source simulation seconds."""
import hashlib
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np


def digest_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def content_digest(document):
    return hashlib.sha256(json.dumps(
        document, sort_keys=True, separators=(',', ':'), allow_nan=False
    ).encode()).hexdigest()


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Reference:
    def __init__(self, data):
        self.data = data
        self.points = np.asarray(data['centerline_world_xy_m'], dtype=float)
        if (self.points.ndim != 2 or self.points.shape[1] != 2 or
                len(self.points) < 3 or not np.isfinite(self.points).all()):
            raise ValueError('Invalid closed reference polyline')
        self.delta = np.roll(self.points, -1, axis=0) - self.points
        self.lengths = np.linalg.norm(self.delta, axis=1)
        if np.any(self.lengths < 1e-9):
            raise ValueError('Reference contains duplicate adjacent points')
        self.cumulative = np.r_[0., np.cumsum(self.lengths)]
        self.length = float(self.cumulative[-1])
        self.half_width = float(data['lane_width_m']) / 2
        self.inner_half_width = (float(data['lane_width_m']) -
                                 float(data['line_width_m'])) / 2
        if not 0 < self.inner_half_width < self.half_width:
            raise ValueError('Invalid lane/line width')
        self.gate_s = self.project(*data['gate_world_xy_m'])['s_m']

    @classmethod
    def load(cls, path, verify_assets=True):
        data = json.loads(Path(path).read_text())
        reference_id = data.pop('reference_id')
        if data.get('schema') != 'wheeltec_reference_v1' or content_digest(data) != reference_id:
            raise ValueError('Reference schema/hash mismatch; export it again')
        data['reference_id'] = reference_id
        if verify_assets:
            for asset in data['assets'].values():
                if digest_file(asset['path']) != asset['sha256']:
                    raise ValueError('Asset changed after reference export: ' + asset['path'])
        return cls(data)

    def project(self, x, y):
        p = np.array([x, y], dtype=float)
        fractions = np.clip(np.einsum('ij,ij->i', p - self.points, self.delta)
                            / self.lengths**2, 0., 1.)
        nearest = self.points + fractions[:, None] * self.delta
        d2 = np.sum((p - nearest)**2, axis=1)
        i = int(np.argmin(d2))
        residual = p - nearest[i]
        cross = self.delta[i, 0] * residual[1] - self.delta[i, 1] * residual[0]
        error = math.copysign(math.sqrt(float(d2[i])), float(cross))
        return {'s_m': float(self.cumulative[i] + fractions[i] * self.lengths[i]),
                'cte_m': error, 'heading_rad': math.atan2(*self.delta[i, ::-1]),
                'reference_x_m': float(nearest[i, 0]),
                'reference_y_m': float(nearest[i, 1])}


@dataclass(frozen=True)
class Settings:
    max_gap_s: float = 0.25
    max_plausible_speed_m_s: float = 3.0
    stop_speed_m_s: float = 0.02
    stop_min_duration_s: float = 0.5
    direction: int = 1

    def __post_init__(self):
        if (self.direction not in (-1, 1) or
                not all(math.isfinite(v) and v > 0 for v in (
                    self.max_gap_s, self.max_plausible_speed_m_s,
                    self.stop_speed_m_s, self.stop_min_duration_s)) or
                self.stop_speed_m_s >= self.max_plausible_speed_m_s):
            raise ValueError('Invalid evaluation settings')


class Metrics:
    """Trapezoidal time/distance weighting; discontinuities are not interpolated.

    Lap times are gate-to-gate. The first partial lap is never counted. A lap
    is eligible only if all its intervals are continuous and the reference
    point stays inside the nominal lane boundaries. This is not collision or
    full-vehicle departure detection.
    """
    def __init__(self, reference, settings=Settings()):
        self.ref, self.settings = reference, settings
        self.prev = None
        self.first_t = self.last_t = None
        self.samples = self.duplicates = self.invalid_samples = 0
        self.gaps = self.teleports = self.progress_jumps = 0
        self.time_reset = False
        self.duration = self.excluded = self.distance = self.progress = 0.
        self.e2 = self.eabs = self.esigned = self.h2 = self.distance_e2 = 0.
        self.max_error = self.max_speed = self.outside_time = self.paint_time = 0.
        self.stop_duration = self.stopped_time = 0.
        self.stop_count = 0
        self.stop_counted = False
        self.gate_time = None
        self.lap_progress = 0.
        self.laps = []
        self.max_gap_seen = 0.

    def break_continuity(self):
        self.gate_time = None
        self.lap_progress = 0.
        self.stop_duration = 0.
        self.stop_counted = False

    def add(self, t, x, y, yaw):
        if self.time_reset:
            raise ValueError('Time reset ended this trial; start a new trial')
        if not all(math.isfinite(v) for v in (t, x, y, yaw)):
            self.invalid_samples += 1
            self.prev = None
            self.break_continuity()
            return None
        if self.last_t is not None and t < self.last_t:
            self.time_reset = True
            self.break_continuity()
            return None
        if self.last_t is not None and t == self.last_t:
            self.duplicates += 1
            return None
        if self.first_t is None:
            self.first_t = t
        self.last_t = t
        self.samples += 1
        projection = self.ref.project(x, y)
        row = dict(t_sim_s=t, x_m=x, y_m=y, yaw_rad=yaw, **projection)
        row['heading_error_rad'] = wrap(yaw - projection['heading_rad'] -
                                        (math.pi if self.settings.direction < 0 else 0))
        row.update(speed_m_s=None, interval_s=None, progress_m=self.progress,
                   quality='FIRST', point_outside_lane=abs(row['cte_m']) > self.ref.half_width)
        self.max_error = max(self.max_error, abs(row['cte_m']))
        old = self.prev
        if old is not None:
            dt = t - old['t_sim_s']
            ds = math.hypot(x - old['x_m'], y - old['y_m'])
            speed = ds / dt
            self.max_gap_seen = max(self.max_gap_seen, dt)
            row['interval_s'] = dt
            if dt > self.settings.max_gap_s:
                row['quality'] = 'GAP'
                self.gaps += 1
                self.excluded += dt
                self.break_continuity()
            elif speed > self.settings.max_plausible_speed_m_s:
                row['quality'] = 'POSE_JUMP'
                self.teleports += 1
                self.excluded += dt
                self.break_continuity()
            else:
                row['quality'] = 'OK'
                row['speed_m_s'] = speed
                self.duration += dt
                self.distance += ds
                self.max_speed = max(self.max_speed, speed)
                e0, e1 = old['cte_m'], row['cte_m']
                self.e2 += dt * (e0*e0 + e1*e1) / 2
                self.distance_e2 += ds * (e0*e0 + e1*e1) / 2
                self.eabs += dt * (abs(e0) + abs(e1)) / 2
                self.esigned += dt * (e0 + e1) / 2
                self.h2 += dt * (old['heading_error_rad']**2 + row['heading_error_rad']**2) / 2
                self.outside_time += dt * (int(abs(e0) > self.ref.half_width) +
                                          int(abs(e1) > self.ref.half_width)) / 2
                self.paint_time += dt * (int(abs(e0) > self.ref.inner_half_width) +
                                        int(abs(e1) > self.ref.inner_half_width)) / 2
                if speed < self.settings.stop_speed_m_s:
                    self.stop_duration += dt
                    self.stopped_time += dt
                    if self.stop_duration >= self.settings.stop_min_duration_s and not self.stop_counted:
                        self.stop_count += 1
                        self.stop_counted = True
                else:
                    self.stop_duration = 0.
                    self.stop_counted = False
                self.update_progress(old, row, dt, ds)
        elif self.samples > 1:
            row['quality'] = 'AFTER_INVALID'
        row['progress_m'] = self.progress
        self.prev = row
        return row

    def update_progress(self, old, row, dt, displacement):
        length = self.ref.length
        d = (row['s_m'] - old['s_m'] + length/2) % length - length/2
        # Nearby Euclidean poses must not jump to a distant parallel branch.
        if abs(d) > 3 * displacement + 0.10:
            self.progress_jumps += 1
            self.break_continuity()
            row['quality'] = 'PROJECTION_JUMP'
            return
        forward = d * self.settings.direction
        self.progress += forward
        self.lap_progress += forward
        if row['point_outside_lane'] or old['point_outside_lane']:
            self.gate_time = None
            self.lap_progress = 0.
            return
        a = (self.settings.direction * (old['s_m'] - self.ref.gate_s)) % length
        # Unwrapped forward crossing avoids false laps from jitter around s=0.
        if forward > 0 and a + forward >= length:
            crossing_t = old['t_sim_s'] + dt * (length-a) / forward
            if self.gate_time is not None and self.lap_progress >= 0.95 * length:
                self.laps.append(crossing_t - self.gate_time)
            self.gate_time = crossing_t
            self.lap_progress = a + forward - length

    def summary(self):
        span = 0. if self.first_t is None else self.last_t - self.first_t
        valid = self.duration > 0
        return {
            'schema': 'wheeltec_tracking_metrics_v1',
            'reference_id': self.ref.data.get('reference_id'),
            'settings': asdict(self.settings),
            'measurement_point': 'top-level roboworks model origin; rear axle midpoint in supplied model',
            'start_sim_s': self.first_t, 'end_sim_s': self.last_t,
            'observed_span_sim_s': span, 'evaluated_time_sim_s': self.duration,
            'coverage_fraction': self.duration / span if span > 0 else None,
            'has_metrics': valid,
            'continuous_trial': valid and not (self.gaps or self.teleports or self.time_reset or
                                               self.invalid_samples or self.progress_jumps),
            'cte_rmse_time_m': math.sqrt(self.e2 / self.duration) if valid else None,
            'cte_mae_time_m': self.eabs / self.duration if valid else None,
            'cte_bias_time_m': self.esigned / self.duration if valid else None,
            'cte_max_sample_m': self.max_error if self.samples else None,
            'cte_rmse_distance_m': math.sqrt(self.distance_e2 / self.distance) if self.distance > 0 else None,
            'heading_rmse_time_rad': math.sqrt(self.h2 / self.duration) if valid else None,
            'distance_observed_m': self.distance,
            'net_reference_progress_m': self.progress,
            'speed_mean_m_s': self.distance / self.duration if valid else None,
            'speed_max_interval_m_s': self.max_speed if valid else None,
            'reference_point_outside_lane_time_s': self.outside_time,
            'reference_point_outside_lane_fraction': self.outside_time / self.duration if valid else None,
            'reference_point_beyond_inner_paint_edge_time_s': self.paint_time,
            'low_speed_time_s': self.stopped_time, 'stop_episodes': self.stop_count,
            'eligible_complete_laps': len(self.laps), 'lap_times_sim_s': self.laps,
            'quality': {'samples': self.samples, 'duplicate_stamps': self.duplicates,
                        'invalid_samples': self.invalid_samples, 'gap_intervals': self.gaps,
                        'pose_jump_intervals': self.teleports, 'projection_jumps': self.progress_jumps,
                        'time_went_backwards': self.time_reset,
                        'max_source_interval_s': self.max_gap_seen},
            'limitations': [
                'Polyline reference; sampling/rasterization have finite resolution.',
                'Errors use a reference point, not the complete vehicle footprint.',
                'Position differences measure planar speed, not wheel speed.',
                'Time/distance integrals use trapezoidal endpoint values.',
                'Missing/jump intervals are excluded; check coverage before comparison.',
                'Lap times require two forward gate crossings with a continuous in-lane lap.',
                'Manual stop uses the last received source stamp; no wall-time extrapolation.',
                'This does not diagnose controller stop causes or detect collisions.',
            ],
        }
