#include "cartesian_manager/core/types.hpp"
#include "cartesian_manager/core/hybrid_orientation_frame.hpp"

namespace manager_core
{
    void RobotContext::updateHybridPose(const Eigen::Vector3d &angular_input, double cone_angle_rad)
    {
        HybridOrientationFrame hybrid_frame;

        const Eigen::Matrix3d R_base_hybrid = hybrid_frame.update(ee_pose.orientation, angular_input, cone_angle_rad, 0.0, hybrid);

        const Eigen::Matrix3d R_hybrid_ee = R_base_hybrid.transpose() * ee_pose.orientation.toRotationMatrix();

        hybrid_frame_pose.position = Eigen::Vector3d::Zero();
        hybrid_frame_pose.orientation = Eigen::Quaterniond(R_hybrid_ee);

        hybrid_frame_pose.orientation.normalize();
        hybrid_frame_pose.frame_id = "hybrid_frame";
        /*hybrid_frame_pose = hybrid_frame.getPose(ee_pose, angular_input, cone_angle_rad);*/
    }
}
