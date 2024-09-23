#include <swerve_drive_controller/odometry.hpp>


namespace swerve_drive_controller {

OdometryProcessor::OdometryProcessor() {
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    odom_msg = nav_msgs::msg::Odometry();
}

OdometryProcessor::update_odometry(Velocity& base_pos_diff, Velocity& base_vel) {
    dx_local_ = base_pos_diff.linear_x;
    dy_local_ = base_pos_diff.linear_y;
    dtheta_local_ = base_pos_diff.angular_z;

    theta_temp_ = theta_ + dtheta_local_ / 2;
    dx_ = dx_local_ * std::cos(theta_temp_) - dy_local_ * std::sin(theta_temp_);
    dy_ = dx_local_ * std::sin(theta_temp_) + dy_local_ * std::cos(theta_temp_);
    dtheta_ = dtheta_local_;

    x_ += dx_;
    y_ += dy_;
    theta_ += dtheta_;

    // odom_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    odom_msg.twist.twist.linear.x = base_vel.linear_x;
    odom_msg.twist.twist.linear.y = base_vel.linear_y;
    odom_msg.twist.twist.angular.z = base_vel.angular_z;
}

}