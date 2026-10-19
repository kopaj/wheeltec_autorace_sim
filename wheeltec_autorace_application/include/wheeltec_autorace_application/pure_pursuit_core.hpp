#pragma once

// Project-owned, ROS-independent geometry. No external tracking library.
#include <algorithm>
#include <cmath>
#include <vector>

namespace wheeltec_autorace_application::pp
{
struct Point { double x{0.0}; double y{0.0}; };
struct Solution { double alpha; double requested_curvature; double curvature; double steering; };

// A Hough score is evidence strength, not distance to the corner.
inline double previewRisk(const double score, const double activation)
{
  return std::clamp((score - activation) / (1.0 - activation), 0.0, 1.0);
}

// Offset an observed ground-plane boundary along its unit normal. The tangent
// points forward. A left boundary moves right (-normal), and vice versa.
inline bool boundaryCenter(const Point boundary, const Point tangent,
  const bool left_boundary, const double lane_width, Point & center)
{
  const double length = std::hypot(tangent.x, tangent.y);
  if (!std::isfinite(length) || length < 1e-9 || tangent.x <= 0.0 ||
    !std::isfinite(lane_width) || lane_width <= 0.0) { return false; }
  const double offset = (left_boundary ? -0.5 : 0.5) * lane_width;
  center = {boundary.x - offset * tangent.y / length,
    boundary.y + offset * tangent.x / length};
  return std::isfinite(center.x) && std::isfinite(center.y) && center.x > 0.0;
}

inline Solution solve(const Point target, const double wheelbase, const double steering_limit)
{
  const double ld = std::hypot(target.x, target.y);
  const double alpha = std::atan2(target.y, target.x);
  const double requested = 2.0 * std::sin(alpha) / ld;
  // Fortress defines R_min = wheelbase / sin(SDF steering_limit).
  // This is stricter than the bicycle-model tan(delta)/wheelbase bound.
  const double max_curvature = std::sin(steering_limit) / wheelbase;
  const double curvature = std::clamp(requested, -max_curvature, max_curvature);
  const double steering = std::atan(wheelbase * curvature);
  return {alpha, requested, curvature, steering};
}

inline double speedLimit(const double curvature, const double preview, const bool degraded,
  const double vmax, const double preview_speed, const double degraded_speed, const double a_lat)
{
  const double v_curve = std::abs(curvature) > 1e-9 ?
    std::sqrt(a_lat / std::abs(curvature)) : vmax;
  const double v_preview = vmax - preview * (vmax - preview_speed);
  return std::min({vmax, v_curve, v_preview, degraded ? degraded_speed : vmax});
}

inline double rateLimit(const double previous, const double target, const double dt,
  const double acceleration, const double deceleration)
{
  return std::clamp(target, std::max(0.0, previous - deceleration * dt),
    previous + acceleration * dt);
}

// Pinhole camera: image u right, v down. Robot x forward, y left.
// Pitch is positive down. Origin is rear axle projected onto the road plane.
// CameraInfo K supplies intrinsics; h is height above the *visual* road plane.
inline bool imageToGround(const double u, const double v,
  const double fx, const double fy, const double cx, const double cy,
  const double camera_x, const double h, const double pitch, Point & ground)
{
  const double a = (u - cx) / fx;
  const double b = (v - cy) / fy;
  const double c = std::cos(pitch), s = std::sin(pitch);
  const double down = s + b * c;
  if (!std::isfinite(down) || down <= 1e-6) { return false; }
  const double distance = h / down;
  ground = {camera_x + distance * (c - b * s), -distance * a};
  return std::isfinite(ground.x) && std::isfinite(ground.y) && ground.x > 0.0;
}

// First outward intersection of a forward centerline segment with the fixed
// lookahead circle. Reject missing intersections: never manufacture a target.
inline bool circleIntersection(const Point a, const Point b, const double radius, Point & p)
{
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double aa = dx * dx + dy * dy;
  const double bb = 2.0 * (a.x * dx + a.y * dy);
  const double cc = a.x * a.x + a.y * a.y - radius * radius;
  const double disc = bb * bb - 4.0 * aa * cc;
  if (aa < 1e-14 || disc < 0.0 || cc > 1e-9 ||
    b.x * b.x + b.y * b.y < radius * radius) { return false; }
  const double t = (-bb + std::sqrt(disc)) / (2.0 * aa);
  if (t < 0.0 || t > 1.0) { return false; }
  p = {a.x + t * dx, a.y + t * dy};
  return p.x > 0.0;
}

enum class TargetMode { invalid, both, left_only, right_only };
struct TargetSample { Point point; TargetMode mode{TargetMode::invalid}; };

// Confidence and target selection method are different properties. A fully
// observed two-sided point beyond the nominal circle retains normal confidence.
// Keep the conservative cap when the available horizon is actually shortened.
inline bool targetIsDegraded(const TargetMode mode, const Point target,
  const double nominal_lookahead, const bool visible_fallback)
{
  const double distance = std::hypot(target.x, target.y);
  if (mode != TargetMode::both || !std::isfinite(distance) || target.x <= 0.0 ||
    !std::isfinite(nominal_lookahead) || nominal_lookahead <= 0.0) { return true; }
  return visible_fallback && distance + 1e-9 < nominal_lookahead;
}

// Ordered near-to-far samples. Invalid samples explicitly break the path.
// Adjacent supported rows can change confidence (e.g. a stripe reappears),
// provided the centerline is continuous within 5 cm. Such an intersection
// keeps single-boundary confidence. Never bridge gaps or geometric jumps.
inline bool supportedTarget(const std::vector<TargetSample> & samples,
  const double lookahead, Point & target, TargetMode & mode)
{
  mode = TargetMode::invalid;
  for (std::size_t i = 1; i < samples.size(); ++i) {
    const auto & a = samples[i - 1];
    const auto & b = samples[i];
    if (a.mode == TargetMode::invalid || b.mode == TargetMode::invalid ||
      b.point.x <= a.point.x ||
      std::hypot(b.point.x - a.point.x, b.point.y - a.point.y) > 0.05) {
      continue;
    }
    if (circleIntersection(a.point, b.point, lookahead, target)) {
      mode = a.mode == TargetMode::both ? b.mode : a.mode;
      return true;
    }
  }
  return false;
}

// Keep the nominal circle as the first choice. If it is not observable, choose
// an ACTUAL sample from a continuous visible fragment, never extrapolate one.
// Only a +/-0.20 m window is allowed (and never less than 0.40 m). Require
// at least 0.10 m of continuous support with three samples. A missing interval
// spanning the nominal circle is not a reason to jump across that interval.
inline bool visibleTarget(const std::vector<TargetSample> & samples,
  const double lookahead, Point & target, TargetMode & mode, bool & fallback)
{
  fallback = false;
  mode = TargetMode::invalid;
  if (!std::isfinite(lookahead) || lookahead <= 0.0) { return false; }
  if (supportedTarget(samples, lookahead, target, mode)) { return true; }
  const double minimum = std::max(0.40, lookahead - 0.20);
  const double maximum = lookahead + 0.20;
  auto usable = [](const TargetSample & s) {
      return s.mode != TargetMode::invalid && std::isfinite(s.point.x) &&
        std::isfinite(s.point.y) && s.point.x > 0.0;
    };
  double all_min = 1e100, all_max = 0.0;
  for (const auto & s : samples) {
    if (usable(s)) {
      const double radius = std::hypot(s.point.x, s.point.y);
      all_min = std::min(all_min, radius);
      all_max = std::max(all_max, radius);
    }
  }
  if (all_min <= lookahead && all_max >= lookahead) { return false; }
  double best_error = 1e100;
  std::size_t begin = 0;
  while (begin < samples.size()) {
    if (!usable(samples[begin])) { ++begin; continue; }
    std::size_t end = begin + 1;
    double length = 0.0;
    double min_radius = std::hypot(samples[begin].point.x, samples[begin].point.y);
    double max_radius = min_radius;
    while (end < samples.size() && usable(samples[end])) {
      const auto a = samples[end - 1].point, b = samples[end].point;
      const double step = std::hypot(b.x - a.x, b.y - a.y);
      if (b.x <= a.x || step > 0.05) { break; }
      length += step;
      const double radius = std::hypot(b.x, b.y);
      min_radius = std::min(min_radius, radius);
      max_radius = std::max(max_radius, radius);
      ++end;
    }
    // Fallback only for a fragment wholly short of or beyond the nominal
    // circle, not for a failed/interrupted intersection inside a fragment.
    if (end - begin >= 3 && length >= 0.10 &&
      (max_radius < lookahead || min_radius > lookahead)) {
      for (std::size_t i = begin; i < end; ++i) {
        const double radius = std::hypot(samples[i].point.x, samples[i].point.y);
        const double error = std::abs(radius - lookahead);
        if (radius >= minimum && radius <= maximum && error < best_error) {
          target = samples[i].point;
          mode = samples[i].mode;
          best_error = error;
        }
      }
    }
    begin = end;
  }
  fallback = best_error < 1e100;
  return fallback;
}
}  // namespace wheeltec_autorace_application::pp
