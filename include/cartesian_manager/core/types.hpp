#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace manager_core
{

  enum class InputSource
  {
    JOYSTICK,
    VISUAL_SERVOING
  };

  enum class Geometrics
  {
    BOTH,
    JACO,
    SNAKE,
  };

  enum class Behaviours
  {
    PASSTHROUGH,
    JOINT_TARGET
  };

  struct FramesConfig
  {
    std::string ee_frame{"ft_frame"};
    std::string base_frame{"base_link"};
    std::string hybrid_frame{"hybrid_frame"};
  };

  struct CartesianPose
  {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    std::string frame_id;
  };

  struct CartesianVelocity
  {
    Eigen::Vector3d linear = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular = Eigen::Vector3d::Zero();
    std::string frame_id;
  };

  using CartesianCommand = CartesianVelocity;

  struct TimedCartesianCommand
  {
    CartesianCommand command;
    double stamp_sec = 0.0;
    bool received = false;
  };

  struct InputChannel
  {
    TimedCartesianCommand latest;
    double timeout{0.2};
    bool enabled{true};
    double weight{1.};
  };

  struct HybridState
  {
    Eigen::Vector3d previous_hybrid_x_ = Eigen::Vector3d::UnitX();
    double enter_cone_ang = 0.09;
    double exit_cone_ang = 0.11;
    bool inside_cone = false;
  };

  struct RobotContext
  {
    CartesianPose ee_pose;
    CartesianVelocity ee_vel;
    
    Eigen::MatrixXd ee_jac;
    
    std::vector<std::string> joint_names;
    Eigen::VectorXd joint_positions;
    
    HybridState hybrid_state;
    CartesianPose hybrid_frame_pose;

    void updateHybridPose();
  };
} // namespace manager_core
