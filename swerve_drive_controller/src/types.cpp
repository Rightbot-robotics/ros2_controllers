#include <swerve_drive_controller/types.hpp>


namespace swerve_drive_controller {

Velocity::Velocity() {
    set_speed(0.0, 0.0, 0.0);
}

void Velocity::set_speed(double linear_x, double linear_y, double angular_z) {
    this->linear_x = linear_x;
    this->linear_y = linear_y;
    this->angular_z = angular_z;
}

void Velocity::set_speed(geometry_msgs::msg::Twist& twist) {
    linear_x = twist.linear.x;
    linear_y = twist.linear.y;
    angular_z = twist.angular.z;
}

void Velocity::set_speed(Velocity& velocity) {
    linear_x = velocity.linear_x;
    linear_y = velocity.linear_y;
    angular_z = velocity.angular_z;
}

bool Velocity::is_zero() {
    return (
        std::abs(linear_x) < epsilon &&
        std::abs(linear_y) < epsilon &&
        std::abs(angular_z) < epsilon
    );
}

}