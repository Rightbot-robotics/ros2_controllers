#include <swerve_drive_controller/kinematics.hpp>


namespace swerve_drive_controller {

SwerveDriveKinematics::SwerveDriveKinematics(std::vector<std::pair<double, double>> module_positions) {
    num_modules_ = module_positions.size();

    matrix_inverse_kinematics_.resize(num_modules_*2, 3);
    for(int i = 0; i < num_modules_; i++) {
        matrix_inverse_kinematics_(i*2, 0) = 1;
        matrix_inverse_kinematics_(i*2, 1) = 0;
        matrix_inverse_kinematics_(i*2, 2) = module_positions[i].second;  // y_pos

        matrix_inverse_kinematics_(i*2+1, 0) = 0;
        matrix_inverse_kinematics_(i*2+1, 1) = 1;
        matrix_inverse_kinematics_(i*2+1, 2) = module_positions[i].first;  // x_pos
    }

    matrix_forward_kinematics_ = matrix_inverse_kinematics_.householderQr();

    modules_vel_matrix_fk_.resize(num_modules_*2, 1);
    modules_vel_matrix_ik_.resize(num_modules_*2, 1);
}

void SwerveDriveKinematics::forward_kinematics(std::vector<std::pair<double, double>>& modules_vel, Velocity& base_vel) {
    for(int i = 0; i < num_modules_; i++) {
        modules_vel_matrix_fk_(i*2, 0) = modules_vel[i].first;
        modules_vel_matrix_fk_(i*2+1, 0) = modules_vel[i].second;
    }
    
    drive_vel_matrix_fk_ = matrix_forward_kinematics_.solve(modules_vel_matrix_fk_);
    
    base_vel.linear_x = drive_vel_matrix_fk_(0);
    base_vel.linear_y = drive_vel_matrix_fk_(1);
    base_vel.angular_z = drive_vel_matrix_fk_(2);
}

void SwerveDriveKinematics::inverse_kinematics(Velocity& base_vel, std::vector<std::pair<double, double>>& modules_vel) {
    drive_vel_matrix_ik_(0) = base_vel.linear_x;
    drive_vel_matrix_ik_(1) = base_vel.linear_y;
    drive_vel_matrix_ik_(2) = base_vel.angular_z;

    modules_vel_matrix_ik_ = matrix_inverse_kinematics_ * drive_vel_matrix_ik_;
    
    if((int)modules_vel.size() != num_modules_) {
        while((int)modules_vel.size() < num_modules_) {
            modules_vel.emplace_back(0, 0);
        }
    }

    for(int i = 0; i < num_modules_; i++) {
        modules_vel[i].first = modules_vel_matrix_ik_(i*2, 0);
        modules_vel[i].second = modules_vel_matrix_ik_(i*2+1, 0);
    }
}

}