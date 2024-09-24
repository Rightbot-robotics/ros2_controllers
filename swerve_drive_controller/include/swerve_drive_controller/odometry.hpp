#ifndef SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_
#define SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_

#include <swerve_drive_controller/types.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace swerve_drive_controller {

class OdometryProcessor {
public:
    nav_msgs::msg::Odometry odom_msg;
    geometry_msgs::msg::TransformStamped transform_msg;
    
    OdometryProcessor();
    void update_odometry(Velocity& base_pos_diff, Velocity& base_vel);

private:
    double x_;
    double y_;
    double theta_;
    double theta_temp_;
    double dx_, dx_local_;
    double dy_, dy_local_;
    double dtheta_, dtheta_local_;
    double linear_x_;
    double linear_y_;
    double angular_z_;

    rclcpp::Clock clock_;
};

}

#endif  // SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_