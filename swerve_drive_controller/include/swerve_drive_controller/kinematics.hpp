#ifndef SWERVE_DRIVE_CONTROLLER__KINEMATICS_HPP_
#define SWERVE_DRIVE_CONTROLLER__KINEMATICS_HPP_

#include <eigen3/Eigen/Dense>
#include <vector>
#include <utility>
#include <swerve_drive_controller/types.hpp>

namespace swerve_drive_controller
{

class SwerveDriveKinematics {
public:
    SwerveDriveKinematics(std::vector<std::pair<double, double>> module_positions);
    // while forward_kinematics() and inverse_kinematics() functions can be called parallely
    // multiple instances to forward_kinematics() or inverse_kinematics() should not be called in parallel
    void forward_kinematics(std::vector<std::pair<double, double>>& modules_vel, Velocity& base_vel);
    void inverse_kinematics(Velocity& base_vel, std::vector<std::pair<double, double>>& modules_vel);

private:
    int num_modules_;
    Eigen::HouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, 3>> matrix_forward_kinematics_;
    Eigen::Matrix<double, Eigen::Dynamic, 3> matrix_inverse_kinematics_;
    Eigen::Matrix<double, Eigen::Dynamic, 1> modules_vel_matrix_fk_, modules_vel_matrix_ik_;
    Eigen::Vector3d drive_vel_matrix_fk_, drive_vel_matrix_ik_;
};

}

#endif  // SWERVE_DRIVE_CONTROLLER__KINEMATICS_HPP_