#include "cartesian_manager/core/types.hpp"
#include <iostream>

namespace manager_core
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

  /*bool isSupportedOrientationFrame(const std::string &orientation_frame_id)
    {
      using Command = geometry_msgs::msg::TwistStamped;
      return orientation_frame_id == Command::BASE_FRAME ||
             orientation_frame_id == Command::EFFECTOR_FRAME ||
             orientation_frame_id == Command::HYBRID_FRAME;
    }*/

    
  void RobotContext::updateHybridPose(/*const Eigen::Vector3d &angular_input,*/ double cone_angle_rad)
  {
    
    
    const Eigen::Matrix3d normalized_orientation = ee_pose.orientation.toRotationMatrix();
    const Eigen::Vector3d z_ee = normalized_orientation.col(2);
    const Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d x_0 = Eigen::Vector3d::UnitX();
    Eigen::Vector3d x;
    Eigen::Vector3d x_temp;

    double alpha = std::acos(z_0.dot(z_ee));
    std::cout << "ALPHA :" << alpha << std::endl;
    std::cout << "CONE ANGLE RAD :" << cone_angle_rad << std::endl;

    if (alpha < cone_angle_rad){
      //x = x_0;
      x= normalizedProjection(x_0, z_ee);

    }
    else{
      x_temp = z_0.cross(z_ee);
      x = x_temp.normalized();
      
    }

    Eigen::Vector3d y = z_ee.cross(x);
    y = y.normalized();

    Eigen::Matrix3d frame;
    frame.col(0) = x;
    frame.col(1) = y;
    frame.col(2) = z_ee;
    std::cout << "Frame :" << std::endl;
    std::cout << "next_x : " << frame.col(0).transpose() << std::endl;
    std::cout << "next_y : " << frame.col(1).transpose() << std::endl;
    std::cout << "tool_z : " << frame.col(2).transpose() << std::endl;

    Eigen::Quaterniond q(frame);
    hybrid_frame_pose.orientation = q ;





    /*if (normalized_orientation.norm() < kVectorEpsilon)
    {
      normalized_orientation = Eigen::Quaterniond::Identity();
    }
    else
    {
      normalized_orientation.normalize();
    }
    const Eigen::Matrix3d current_hybrid = hybrid_frame_pose.orientation.toRotationMatrix();
    const Eigen::Vector3d base_x = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d base_z = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d tool_z =
        (current_hybrid * Eigen::Vector3d::UnitZ()).normalized();

    const double vertical_alignment = std::clamp(base_z.dot(tool_z), -1.0, 1.0);
    const bool inside_cone = std::abs(vertical_alignment) > std::cos(cone_angle_rad);*/
    //const bool active = angular_input.norm() > release_threshold;
    //const bool reversed =
        //active && has_last_active_input_ && angular_input.dot(last_active_input_) < 0.0;
    //const bool reanchor = !initialized_ || !active || reversed;
    /*
    Eigen::Vector3d next_x;

    :*/
    /*
    if (inside_cone)
    {*/
      //next_x = normalizedProjection(/*reanchor ?*/current_hybrid.col(0), tool_z);
    //}
    /*else
    {
      next_x = base_z.cross(tool_z).normalized();*/
      //if (/*!reanchor &&*/ next_x.dot(base_x) < 0.0)
      /*{
        next_x = -next_x;
      }
    }

    Eigen::Vector3d next_y = tool_z.cross(next_x);
    if (next_y.norm() < kVectorEpsilon)
    {
      next_x = normalizedProjection(base_x, tool_z);
      next_y = tool_z.cross(next_x);
    }
    next_y.normalize();
    next_x = next_y.cross(tool_z).normalized();
    */
    //x_axis_ = next_x;
    //initialized_ = true;
    /*if (active)
    {
      last_active_input_ = angular_input;
      has_last_active_input_ = true;
    }
    else
    {
      last_active_input_.setZero();
      has_last_active_input_ = false;
    }*/

    /*Eigen::Matrix3d frame;
    frame.col(0) = next_x;
    frame.col(1) = next_y;
    frame.col(2) = tool_z;
    std::cout << "Frame :" << std::endl;
    std::cout << "next_x : " << frame.col(0).transpose() << std::endl;
    std::cout << "next_y : " << frame.col(1).transpose() << std::endl;
    std::cout << "tool_z : " << frame.col(2).transpose() << std::endl;

    Eigen::Quaterniond q(frame);
    hybrid_frame_pose.orientation = q ;*/

  }

}
