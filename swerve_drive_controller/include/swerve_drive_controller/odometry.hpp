#ifndef SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_
#define SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_

#include <swerve_drive_controller/types.h>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

namespace swerve_drive_controller {

class OdometryProcessor {
public:
    nav_msgs::msg::Odometry odom_msg;
    
    OdometryProcessor();
    update_odometry(Velocity& base_pos_diff, Velocity& base_vel);

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
}

}

#endif  // SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_