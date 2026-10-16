#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

namespace ego_planner
{
using EscapePoints = std::vector<Eigen::Vector2d>;

inline double obstacleDistance(const Eigen::Vector2d & p, const EscapePoints & obstacles)
{
  double result = std::numeric_limits<double>::infinity();
  for (const auto & q : obstacles) {result = std::min(result, (p-q).norm());}
  return result;
}

inline bool escapeSegmentClear(const Eigen::Vector2d & from, const Eigen::Vector2d & to,
  const EscapePoints & obstacles, double hard_radius)
{
  const int steps = std::max(1, static_cast<int>(std::ceil((to-from).norm()/0.025)));
  for (int i = 0; i <= steps; ++i) {
    if (obstacleDistance(from+(to-from)*(double(i)/steps), obstacles) < hard_radius+0.013) {
      return false;
    }
  }
  return true;
}

// Repulsion supplies a preferred direction, not a physical force/teleport.
// Search nearby free cells, allowing the initial soft inflation overlap while
// retaining measured-obstacle clearance along the entire escape segment.
inline bool findInflationExit(const Eigen::Vector2d & position, double yaw,
  const Eigen::Vector2d & goal, const EscapePoints & obstacles,
  const std::function<bool(const Eigen::Vector2d &)> & free,
  double hard_radius, Eigen::Vector2d & target)
{
  const double initial_distance = obstacleDistance(position, obstacles);
  if (obstacles.empty() || initial_distance < hard_radius+0.013) {return false;}
  Eigen::Vector2d force = Eigen::Vector2d::Zero();
  for (const auto & q : obstacles) {
    const Eigen::Vector2d delta = position-q;
    const double d = delta.norm();
    if (d > 0.01 && d < 0.8) {force += delta*(1.0/d-1.0/0.8)/(d*d);}
  }
  Eigen::Vector2d goal_direction = goal-position;
  if (goal_direction.norm() > 1e-6) {goal_direction.normalize();}
  if (force.norm() > 1e-6) {force.normalize();} else {force = goal_direction;}
  double best = std::numeric_limits<double>::infinity();
  for (double distance = 0.25; distance <= 1.01; distance += 0.1) {
    for (int k = 0; k < 48; ++k) {
      const double angle = k*2*M_PI/48;
      const Eigen::Vector2d direction(std::cos(angle), std::sin(angle));
      const Eigen::Vector2d candidate = position+distance*direction;
      if (!free(candidate) || !free(candidate+0.06*direction)) {continue;}
      // Only move outward when close enough for an actual repulsion field.
      if (initial_distance < 0.7 &&
        obstacleDistance(candidate, obstacles) < initial_distance+0.04) {continue;}
      if (!escapeSegmentClear(position, candidate, obstacles, hard_radius)) {continue;}
      const double score = distance + 0.25*(1-direction.dot(force)) +
        0.08*(1-direction.dot(goal_direction)) + 0.03*(1-std::cos(angle-yaw));
      if (score < best) {best = score; target = candidate;}
    }
  }
  return std::isfinite(best);
}

inline Eigen::Vector2d inflationExitCommand(const Eigen::Vector2d & position, double yaw,
  const Eigen::Vector2d & target, double speed, double turn_limit)
{
  const Eigen::Vector2d delta = target-position;
  const double error = std::remainder(std::atan2(delta.y(), delta.x())-yaw, 2*M_PI);
  const double turn = std::abs(error) > 0.15 ? std::copysign(turn_limit, error) :
    std::clamp(1.5*error, -turn_limit, turn_limit);
  // Rotate first: no lateral displacement and no reverse command.
  const double forward = std::abs(error) > 0.15 ? 0.0 :
    speed*std::max(0.0, std::cos(error));
  return {forward, turn};
}
}  // namespace ego_planner
