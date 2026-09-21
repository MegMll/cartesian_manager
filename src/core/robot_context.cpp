#include "cartesian_manager/core/types.hpp"
#include <iostream>

namespace manager_core
{
  namespace
  {
    constexpr double kVectorEpsilon = 1e-12;
    Eigen::Vector3d projectOntoNormalPlane(const Eigen::Vector3d &vector,
                                           const Eigen::Vector3d &plane_normal)
    {
      return vector - vector.dot(plane_normal) * plane_normal;
    }

    Eigen::Vector3d normalizedProjection(const Eigen::Vector3d &preferred_axis,
                                         const Eigen::Vector3d &plane_normal)
    {
      Eigen::Vector3d projected = projectOntoNormalPlane(preferred_axis, plane_normal);
      if (projected.norm() < kVectorEpsilon)
      {
        projected = projectOntoNormalPlane(Eigen::Vector3d::UnitY(), plane_normal);
      }
      if (projected.norm() < kVectorEpsilon)
      {
        projected = plane_normal.unitOrthogonal();
      }
      return projected.normalized();
    }
  } // namespace

  void RobotContext::updateHybridPose(const Eigen::Vector3d &angular_input)
  {
    Eigen::Quaterniond q_ee = ee_pose.orientation;
    const Eigen::Vector3d z_ee = q_ee.toRotationMatrix().col(2);
    const Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d x_0 = Eigen::Vector3d::UnitX();

    const double vertical_alignment = std::clamp(z_0.dot(z_ee), -1.0, 1.0);
    hybrid_state.inside_cone = std::abs(vertical_alignment) > std::cos(hybrid_state.min_cone_ang);

    const bool active = angular_input.norm() > hybrid_state.released_input;
    const bool reversed = active && hybrid_state.has_previous_active_input &&
                          angular_input.dot(hybrid_state.previous_angular_input_) < 0.0;
    const bool reanchor = !active || reversed;
    Eigen::Vector3d x;
    Eigen::Vector3d reference_x = normalizedProjection(x_0, z_ee);

    if (hybrid_state.inside_cone)
    {
      x = normalizedProjection(reanchor ? x_0 : hybrid_state.previous_hybrid_x_, z_ee);
    }
    else
    {
      x = z_0.cross(z_ee).normalized();
      if (!reanchor && x.dot(hybrid_state.previous_hybrid_x_) < 0.0)
      {
        x = -x;
      }
    }

    hybrid_state.previous_hybrid_x_ = x;
    if (active)
    {
      hybrid_state.previous_angular_input_ = angular_input;
      hybrid_state.has_previous_active_input = true;
    }
    else
    {
      hybrid_state.previous_angular_input_.setZero();
      hybrid_state.has_previous_active_input = false;
    }

    Eigen::Vector3d y = z_ee.cross(x).normalized();

    Eigen::Matrix3d frame;
    frame.col(0) = x;
    frame.col(1) = y;
    frame.col(2) = z_ee;

    Eigen::Quaterniond q(frame);
    q.normalize();

    hybrid_frame_pose.orientation = q;
  }
} // namespace manager_core
