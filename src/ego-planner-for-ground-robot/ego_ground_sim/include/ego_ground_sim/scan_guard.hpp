#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>

namespace ego_ground_sim
{
struct Point2 {double x; double y;};

// Exact constant-twist arc, with a conservative constant-speed braking horizon.
// A circle bounds the V550 body and wheels for every heading, including rotation.
inline bool sweepBlocked(const std::vector<Point2> & points, double speed, double turn,
  double radius, double reaction_time, double deceleration)
{
  const double horizon = reaction_time + std::abs(speed)/deceleration;
  const int steps = std::max(1, static_cast<int>(std::ceil(horizon/0.02)));
  // Cover the distance between arc samples as well as the sampled circles.
  const double bound = radius + std::abs(speed)*0.02;
  for (int i = 0; i <= steps; ++i) {
    const double t = horizon*i/steps;
    const double x = std::abs(turn) < 1e-6 ? speed*t : speed*std::sin(turn*t)/turn;
    const double y = std::abs(turn) < 1e-6 ? 0.0 : speed*(1-std::cos(turn*t))/turn;
    for (const auto & p : points) {
      if (std::hypot(p.x-x, p.y-y) <= bound) {return true;}
    }
  }
  return false;
}

// Permit escape from an existing soft overlap while retaining the hard body
// envelope. Translation must improve clearance; rotating a bounded circle
// changes neither its clearance nor its occupied area.
inline bool guardMotionBlocked(const std::vector<Point2> & points, double speed, double turn,
  double hard_radius, double soft_radius, double reaction_time, double deceleration)
{
  if (sweepBlocked(points, speed, turn, hard_radius, reaction_time, deceleration)) {return true;}
  double initial = std::numeric_limits<double>::infinity();
  for (const auto & p : points) {initial = std::min(initial, std::hypot(p.x, p.y));}
  if (initial > soft_radius+0.004) {
    return sweepBlocked(points, speed, turn, soft_radius, reaction_time, deceleration);
  }
  if (std::abs(speed) < 1e-6) {return false;}
  const double horizon = reaction_time+std::abs(speed)/deceleration;
  double final_distance = initial;
  for (double t = 0.02; t <= horizon+0.02; t += 0.02) {
    const double x = std::abs(turn) < 1e-6 ? speed*t : speed*std::sin(turn*t)/turn;
    const double y = std::abs(turn) < 1e-6 ? 0.0 : speed*(1-std::cos(turn*t))/turn;
    final_distance = std::numeric_limits<double>::infinity();
    for (const auto & p : points) {
      const double before = std::hypot(p.x, p.y), after = std::hypot(p.x-x, p.y-y);
      if (before < soft_radius+0.02 && after < before-0.003) {return true;}
      final_distance = std::min(final_distance, after);
    }
  }
  return final_distance < std::min(initial+0.008, soft_radius+0.008);
}
}  // namespace ego_ground_sim
