#ifndef SWERVE_DRIVE_CONTROLLER__TYPES_HPP_
#define SWERVE_DRIVE_CONTROLLER__TYPES_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

namespace swerve_drive_controller{

class Velocity {
public:
    double linear_x;
    double linear_y;
    double angular_z;
    double epsilon = 1e-6;

    Velocity();
    void set_speed(double linear_x, double linear_y, double angular_z);
    void set_speed(geometry_msgs::msg::Twist& twist);
    void set_speed(Velocity& velocity);
    bool is_zero();
};

}

#endif  // SWERVE_DRIVE_CONTROLLER__TYPES_HPP_