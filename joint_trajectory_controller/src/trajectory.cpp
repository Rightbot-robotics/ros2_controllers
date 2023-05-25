// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "joint_trajectory_controller/trajectory.hpp"

#include <memory>
#include <iostream>
#include "hardware_interface/macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rcppmath/clamp.hpp"
#include "std_msgs/msg/header.hpp"
namespace joint_trajectory_controller
{
Trajectory::Trajectory() : trajectory_start_time_(0), time_before_traj_msg_(0) {}

Trajectory::Trajectory(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
: trajectory_msg_(joint_trajectory),
  trajectory_start_time_(static_cast<rclcpp::Time>(joint_trajectory->header.stamp))
{
}

Trajectory::Trajectory(
  const rclcpp::Time & current_time,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_point,
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
: trajectory_msg_(joint_trajectory),
  trajectory_start_time_(static_cast<rclcpp::Time>(joint_trajectory->header.stamp))
{
  set_point_before_trajectory_msg(current_time, current_point);
  update(joint_trajectory);
}

void Trajectory::set_point_before_trajectory_msg(
  const rclcpp::Time & current_time,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_point)
{
  time_before_traj_msg_ = current_time;
  state_before_traj_msg_ = current_point;
}

void Trajectory::update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
{
  trajectory_msg_ = joint_trajectory;
  trajectory_start_time_ = static_cast<rclcpp::Time>(joint_trajectory->header.stamp);
  sampled_already_ = false;
}

bool Trajectory::sample(
  const rclcpp::Time & sample_time,
  const interpolation_methods::InterpolationMethod interpolation_method,
  trajectory_msgs::msg::JointTrajectoryPoint & output_state,
  TrajectoryPointConstIter & start_segment_itr, TrajectoryPointConstIter & end_segment_itr)
{
  THROW_ON_NULLPTR(trajectory_msg_)

  if (trajectory_msg_->points.empty())
  {
    start_segment_itr = end();
    end_segment_itr = end();
    return false;
  }

  // first sampling of this trajectory
  if (!sampled_already_)
  {
    if (trajectory_start_time_.seconds() == 0.0)
    {
      trajectory_start_time_ = sample_time;
    }

    sampled_already_ = true;
  }

  // sampling before the current point
  if (sample_time < time_before_traj_msg_)
  {
    return false;
  }

  output_state = trajectory_msgs::msg::JointTrajectoryPoint();
  auto & first_point_in_msg = trajectory_msg_->points[0];
  const rclcpp::Time first_point_timestamp =
    trajectory_start_time_ + first_point_in_msg.time_from_start;

  // current time hasn't reached traj time of the first point in the msg yet
  if (sample_time < first_point_timestamp)
  {
    // If interpolation is disabled, just forward the next waypoint
    if (interpolation_method == interpolation_methods::InterpolationMethod::NONE)
    {
      output_state = state_before_traj_msg_;
    }
    else
    {
      // it changes points only if position and velocity do not exist, but their derivatives
      deduce_from_derivatives(
        state_before_traj_msg_, first_point_in_msg, state_before_traj_msg_.positions.size(),
        (first_point_timestamp - time_before_traj_msg_).seconds());

      interpolate_between_points(
        time_before_traj_msg_, state_before_traj_msg_, first_point_timestamp, first_point_in_msg,
        sample_time, output_state);
    }
    start_segment_itr = begin();  // no segments before the first
    end_segment_itr = begin();
    return true;
  }

  // time_from_start + trajectory time is the expected arrival time of trajectory
  const auto last_idx = trajectory_msg_->points.size() - 1;
  for (size_t i = 0; i < last_idx; ++i)
  {
    auto & point = trajectory_msg_->points[i];
    auto & next_point = trajectory_msg_->points[i + 1];

    const rclcpp::Time t0 = trajectory_start_time_ + point.time_from_start;
    const rclcpp::Time t1 = trajectory_start_time_ + next_point.time_from_start;

    if (sample_time >= t0 && sample_time < t1)
    {
      // If interpolation is disabled, just forward the next waypoint
      if (interpolation_method == interpolation_methods::InterpolationMethod::NONE)
      {
        output_state = next_point;
      }
      // Do interpolation
      else
      {
      
        // it changes points only if position and velocity do not exist, but their derivatives
        deduce_from_derivatives(
          point, next_point, state_before_traj_msg_.positions.size(), (t1 - t0).seconds());

        interpolate_between_points(t0, point, t1, next_point, sample_time, output_state);
      }
      start_segment_itr = begin() + i;
      end_segment_itr = begin() + (i + 1);

      return true;
    }
  }

  // whole animation has played out
  start_segment_itr = --end();
  end_segment_itr = end();
  output_state = (*start_segment_itr);
  // the trajectories in msg may have empty velocities/accel, so resize them
  if (output_state.velocities.empty())
  {
    output_state.velocities.resize(output_state.positions.size(), 0.0);
  }
  if (output_state.accelerations.empty())
  {
    output_state.accelerations.resize(output_state.positions.size(), 0.0);
  }
  return true;
}

bool Trajectory::sample_test(
  const rclcpp::Time & sample_time,
  const interpolation_methods::InterpolationMethod interpolation_method,
  trajectory_msgs::msg::JointTrajectoryPoint & output_state, trajectory_msgs::msg::JointTrajectoryPoint & output_state_parsed, bool & active,
  TrajectoryPointConstIter & start_segment_itr, TrajectoryPointConstIter & end_segment_itr, bool & empty_traj)
{
  THROW_ON_NULLPTR(trajectory_msg_)

  if (trajectory_msg_->points.empty())
  {
    start_segment_itr = end();
    end_segment_itr = end();
    empty_traj = true;
    return false;
  }
  empty_traj = false;

  // first sampling of this trajectory
  if (!sampled_already_)
  {
    if (trajectory_start_time_.seconds() == 0.0)
    {
      trajectory_start_time_ = sample_time;
    }

    sampled_already_ = true;
    std::cout <<"TEST sampled_already_" << std::endl;
    max_pos.clear();
    max_vel.clear();
    max_accel.clear();
    traj_start_time.clear();

    bool traj_started;
    bool traj_finished;

    final_positions.clear();
    final_velocities.clear();
    final_accelerations.clear();

    final_positions.resize(trajectory_msg_->joint_names.size());
    final_velocities.resize(trajectory_msg_->joint_names.size());
    final_accelerations.resize(trajectory_msg_->joint_names.size());

    for (size_t i = 0; i < trajectory_msg_->joint_names.size(); ++i)
    {

      const std::string & incoming_joint_name = trajectory_msg_->joint_names[i];
      // std::vector<double> positions;
      double maximum_position;
      std::vector<double> velocities;
      std::vector<double> accelerations;

      std::vector<double> current_joint_pos;
      std::vector<double> current_joint_vel;
      std::vector<double> current_joint_accel;
      std::vector<rclcpp::Duration> current_joint_traj_start_time;

      traj_started = false;
      traj_finished = false;
      bool acceleration_value = false;

      for (size_t j = 0; j < trajectory_msg_->points.size()-2; ++j){

        auto p = trajectory_msg_->points;

        double start_floating_window_point_1;
        double start_floating_window_point_2;
        double start_floating_window_point_3;

        double finish_floating_window_point_1;
        double finish_floating_window_point_2;
        double finish_floating_window_point_3;

        // std::cout << "velocities " << incoming_joint_name << " is " <<  p[j].velocities[i]  << std::endl;
        rclcpp::Duration time_temp = p[j].time_from_start; 
        // std::cout << "time temp " << time_temp.seconds() << std::endl;

        if(!traj_started && !traj_finished){

          start_floating_window_point_1 = abs(p[j].velocities[i]);
          start_floating_window_point_2 = abs(p[j+1].velocities[i]);
          start_floating_window_point_3 = abs(p[j+2].velocities[i]);

          if((start_floating_window_point_1 > 0.0 + 10e-5) 
            && (start_floating_window_point_2 > start_floating_window_point_1)
            && (start_floating_window_point_3 > start_floating_window_point_2)){
            //
            traj_started = true;
            rclcpp::Duration current_trapezoid_start_time = p[j].time_from_start;
            current_joint_traj_start_time.push_back(current_trapezoid_start_time); // traj start time
            
            std::cout << "traj start time for joint " << i << " is " << current_trapezoid_start_time.seconds() << std::endl;

            velocities.push_back(abs(p[j].velocities[i]));
            accelerations.push_back(abs(p[j].accelerations[i]));

          }
        }

        if(traj_started && !traj_finished){
          finish_floating_window_point_1 = abs(p[j].velocities[i]);
          finish_floating_window_point_2 = abs(p[j+1].velocities[i]);
          finish_floating_window_point_3 = abs(p[j+2].velocities[i]);

          // std::cout << "start_floating_window_point_1 " << start_floating_window_point_1 << std::endl;
          // std::cout << "start_floating_window_point_2 " << start_floating_window_point_2 << std::endl;
          // std::cout << "start_floating_window_point_3 " << start_floating_window_point_3 << std::endl;
          

          if((finish_floating_window_point_3 < 0.0 + 10e-3) 
            && (finish_floating_window_point_3 < finish_floating_window_point_2)
            && (finish_floating_window_point_2 < finish_floating_window_point_1)){
            //
            traj_finished = true;
            traj_started = false;

            maximum_position = p[j+2].positions[i];

            velocities.push_back(abs(p[j].velocities[i]));
            velocities.push_back(abs(p[j+1].velocities[i]));
            velocities.push_back(abs(p[j+2].velocities[i]));

            accelerations.push_back(abs(p[j].accelerations[i]));
            accelerations.push_back(abs(p[j+1].accelerations[i]));
            accelerations.push_back(abs(p[j+2].accelerations[i]));

            std::cout << "traj finished " << std::endl;

          }else{
           
            velocities.push_back(abs(p[j].velocities[i]));
            accelerations.push_back(abs(p[j].accelerations[i]));
            

          }
        }

        if(traj_finished && !traj_started){

          auto maxElementPos =  maximum_position;
          auto maxElementVel = std::max_element(velocities.begin(), velocities.end());
          auto maxElementAccel = std::max_element(accelerations.begin(), accelerations.end());
          
          current_joint_pos.push_back(maxElementPos);
          current_joint_vel.push_back(*maxElementVel);
          current_joint_accel.push_back(*maxElementAccel);

          std::cout << "traj adding " << incoming_joint_name << std::endl;
          std::cout << "traj adding pos " << maxElementPos << std::endl;
          std::cout << "traj adding vel " << *maxElementVel << std::endl;
          std::cout << "traj adding accel " << *maxElementAccel << std::endl;

          traj_finished = false;
          traj_started = false;

          // positions.clear();
          velocities.clear();
          accelerations.clear();

        }

        if(j == trajectory_msg_->points.size()-3){
          if(current_joint_vel.empty()){

            std::cout << "traj empty adding " << incoming_joint_name << std::endl;
            std::cout << "traj adding pos " << 0.0 << std::endl;
            std::cout << "traj adding vel " << 0.0 << std::endl;
            std::cout << "traj adding accel " << 0.0 << std::endl;

            current_joint_pos.push_back(0.0);
            current_joint_vel.push_back(0.0);
            current_joint_accel.push_back(0.0);
            current_joint_traj_start_time.push_back(p[0].time_from_start);

          }
        }
        
      }

      max_pos.push_back(current_joint_pos);
      max_vel.push_back(current_joint_vel);
      max_accel.push_back(current_joint_accel);
      traj_start_time.push_back(current_joint_traj_start_time);

    }

  }

  // sampling before the current point
  if (sample_time < time_before_traj_msg_)
  {
    return false;
  }

  output_state = trajectory_msgs::msg::JointTrajectoryPoint();
  output_state_parsed = trajectory_msgs::msg::JointTrajectoryPoint();
  auto & first_point_in_msg = trajectory_msg_->points[0];
  const rclcpp::Time first_point_timestamp =
    trajectory_start_time_ + first_point_in_msg.time_from_start;

  // current time hasn't reached traj time of the first point in the msg yet
  if (sample_time < first_point_timestamp)
  {
    // If interpolation is disabled, just forward the next waypoint
    if (interpolation_method == interpolation_methods::InterpolationMethod::NONE)
    {
      output_state = state_before_traj_msg_;
    }
    else
    {
      // it changes points only if position and velocity do not exist, but their derivatives
      deduce_from_derivatives(
        state_before_traj_msg_, first_point_in_msg, state_before_traj_msg_.positions.size(),
        (first_point_timestamp - time_before_traj_msg_).seconds());

      interpolate_between_points(
        time_before_traj_msg_, state_before_traj_msg_, first_point_timestamp, first_point_in_msg,
        sample_time, output_state);
    }
    start_segment_itr = begin();  // no segments before the first
    end_segment_itr = begin();
    return true;
  }

  // time_from_start + trajectory time is the expected arrival time of trajectory
  const auto last_idx = trajectory_msg_->points.size() - 1;
  for (size_t i = 0; i < last_idx; ++i)
  {
    auto & point = trajectory_msg_->points[i];
    auto & next_point = trajectory_msg_->points[i + 1];

    const rclcpp::Time t0 = trajectory_start_time_ + point.time_from_start;
    const rclcpp::Time t1 = trajectory_start_time_ + next_point.time_from_start;

    if (sample_time >= t0 && sample_time < t1)
    {
      // If interpolation is disabled, just forward the next waypoint
      if (interpolation_method == interpolation_methods::InterpolationMethod::NONE)
      {
        output_state = next_point;
      }
      // Do interpolation
      else
      {
      
        // it changes points only if position and velocity do not exist, but their derivatives
        deduce_from_derivatives(
          point, next_point, state_before_traj_msg_.positions.size(), (t1 - t0).seconds());

        interpolate_between_points(t0, point, t1, next_point, sample_time, output_state);

        output_state_parsed = output_state;

        for(size_t i = 0; i < max_vel.size(); ++i){// joints

          // std::cout << "max vel size " << max_vel.size() << std::endl;
          //
          for(size_t j = 0; j < max_vel[i].size(); ++j){ // mulitple trapezoids for the joint
          //
            const rclcpp::Time traj_time = trajectory_start_time_ + traj_start_time[i][j];

            rclcpp::Duration traj_time_temp = traj_start_time[i][j];
            // std::cout << "traj start time for joint " << i << " is " << traj_time_temp.seconds() << std::endl;
          
            if(sample_time >= traj_time){

              // std::cout << "traj sending joint " << i << std::endl;

              // std::cout << "traj update position " << max_pos[i][j] << std::endl;
              // std::cout << "traj update velocitites " << max_vel[i][j] << std::endl;
              // std::cout << "traj update accelerations " << max_accel[i][j] << std::endl;

              

              final_positions[i] = max_pos[i][j];
              final_velocities[i] = max_vel[i][j];
              final_accelerations[i] = max_accel[i][j];

              // std::cout << "output pos size " << output_state_parsed.positions.size() << std::endl;
              // output_state_parsed.positions[i]= max_pos[i][j];
              // std::cout << "output vel size " << output_state_parsed.velocities.size() << std::endl;
              // output_state_parsed.velocities[i]= max_vel[i][j];
              // std::cout << "output accel size " << output_state_parsed.accelerations.size() << std::endl;
              // output_state_parsed.accelerations[i]= max_accel[i][j];

              
              
            }
            

          }

        }

        active = true;

        std::cout << "active in sample " << active << std::endl;

        output_state_parsed.positions = final_positions;
        output_state_parsed.velocities = final_velocities;
        output_state_parsed.accelerations = final_accelerations;

        // std::cout << "traj sending joint " << i << std::endl;
        
      }
      start_segment_itr = begin() + i;
      end_segment_itr = begin() + (i + 1);

      return true;
    }
  }

  active = false;

  std::cout << "active " << active << std::endl;
  // whole animation has played out
  start_segment_itr = --end();
  end_segment_itr = end();
  output_state = (*start_segment_itr);
  // the trajectories in msg may have empty velocities/accel, so resize them
  if (output_state.velocities.empty())
  {
    output_state.velocities.resize(output_state.positions.size(), 0.0);
  }
  if (output_state.accelerations.empty())
  {
    output_state.accelerations.resize(output_state.positions.size(), 0.0);
  }
  return true;
}

void Trajectory::interpolate_between_points(
  const rclcpp::Time & time_a, const trajectory_msgs::msg::JointTrajectoryPoint & state_a,
  const rclcpp::Time & time_b, const trajectory_msgs::msg::JointTrajectoryPoint & state_b,
  const rclcpp::Time & sample_time, trajectory_msgs::msg::JointTrajectoryPoint & output)
{
  rclcpp::Duration duration_so_far = sample_time - time_a;
  rclcpp::Duration duration_btwn_points = time_b - time_a;

  const size_t dim = state_a.positions.size();
  output.positions.resize(dim, 0.0);
  output.velocities.resize(dim, 0.0);
  output.accelerations.resize(dim, 0.0);

  auto generate_powers = [](int n, double x, double * powers)
  {
    powers[0] = 1.0;
    for (int i = 1; i <= n; ++i)
    {
      powers[i] = powers[i - 1] * x;
    }
  };

  bool has_velocity = !state_a.velocities.empty() && !state_b.velocities.empty();
  bool has_accel = !state_a.accelerations.empty() && !state_b.accelerations.empty();
  if (duration_so_far.seconds() < 0.0)
  {
    duration_so_far = rclcpp::Duration::from_seconds(0.0);
    has_velocity = has_accel = false;
  }
  if (duration_so_far.seconds() > duration_btwn_points.seconds())
  {
    duration_so_far = duration_btwn_points;
    has_velocity = has_accel = false;
  }

  double t[6];
  generate_powers(5, duration_so_far.seconds(), t);

  if (!has_velocity && !has_accel)
  {
    // do linear interpolation
    for (size_t i = 0; i < dim; ++i)
    {
      double start_pos = state_a.positions[i];
      double end_pos = state_b.positions[i];

      double coefficients[2] = {0.0, 0.0};
      coefficients[0] = start_pos;
      if (duration_btwn_points.seconds() != 0.0)
      {
        coefficients[1] = (end_pos - start_pos) / duration_btwn_points.seconds();
      }

      output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1];
      output.velocities[i] = t[0] * coefficients[1];
    }
  }
  else if (has_velocity && !has_accel)
  {
    // do cubic interpolation
    double T[4];
    generate_powers(3, duration_btwn_points.seconds(), T);

    for (size_t i = 0; i < dim; ++i)
    {
      double start_pos = state_a.positions[i];
      double start_vel = state_a.velocities[i];
      double end_pos = state_b.positions[i];
      double end_vel = state_b.velocities[i];

      double coefficients[4] = {0.0, 0.0, 0.0, 0.0};
      coefficients[0] = start_pos;
      coefficients[1] = start_vel;
      if (duration_btwn_points.seconds() != 0.0)
      {
        coefficients[2] =
          (-3.0 * start_pos + 3.0 * end_pos - 2.0 * start_vel * T[1] - end_vel * T[1]) / T[2];
        coefficients[3] =
          (2.0 * start_pos - 2.0 * end_pos + start_vel * T[1] + end_vel * T[1]) / T[3];
      }

      output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1] +
                            t[2] * coefficients[2] + t[3] * coefficients[3];
      output.velocities[i] =
        t[0] * coefficients[1] + t[1] * 2.0 * coefficients[2] + t[2] * 3.0 * coefficients[3];
      output.accelerations[i] = t[0] * 2.0 * coefficients[2] + t[1] * 6.0 * coefficients[3];
    }
  }
  else if (has_velocity && has_accel)
  {
    // do quintic interpolation
    double T[6];
    generate_powers(5, duration_btwn_points.seconds(), T);

    for (size_t i = 0; i < dim; ++i)
    {
      double start_pos = state_a.positions[i];
      double start_vel = state_a.velocities[i];
      double start_acc = state_a.accelerations[i];
      double end_pos = state_b.positions[i];
      double end_vel = state_b.velocities[i];
      double end_acc = state_b.accelerations[i];

      double coefficients[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      coefficients[0] = start_pos;
      coefficients[1] = start_vel;
      coefficients[2] = 0.5 * start_acc;
      if (duration_btwn_points.seconds() != 0.0)
      {
        coefficients[3] = (-20.0 * start_pos + 20.0 * end_pos - 3.0 * start_acc * T[2] +
                           end_acc * T[2] - 12.0 * start_vel * T[1] - 8.0 * end_vel * T[1]) /
                          (2.0 * T[3]);
        coefficients[4] = (30.0 * start_pos - 30.0 * end_pos + 3.0 * start_acc * T[2] -
                           2.0 * end_acc * T[2] + 16.0 * start_vel * T[1] + 14.0 * end_vel * T[1]) /
                          (2.0 * T[4]);
        coefficients[5] = (-12.0 * start_pos + 12.0 * end_pos - start_acc * T[2] + end_acc * T[2] -
                           6.0 * start_vel * T[1] - 6.0 * end_vel * T[1]) /
                          (2.0 * T[5]);
      }

      output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1] +
                            t[2] * coefficients[2] + t[3] * coefficients[3] +
                            t[4] * coefficients[4] + t[5] * coefficients[5];
      output.velocities[i] = t[0] * coefficients[1] + t[1] * 2.0 * coefficients[2] +
                             t[2] * 3.0 * coefficients[3] + t[3] * 4.0 * coefficients[4] +
                             t[4] * 5.0 * coefficients[5];
      output.accelerations[i] = t[0] * 2.0 * coefficients[2] + t[1] * 6.0 * coefficients[3] +
                                t[2] * 12.0 * coefficients[4] + t[3] * 20.0 * coefficients[5];
    }
  }
}

void Trajectory::interpolate_between_points_test(
  const rclcpp::Time & time_a, const trajectory_msgs::msg::JointTrajectoryPoint & state_a,
  const rclcpp::Time & time_b, const trajectory_msgs::msg::JointTrajectoryPoint & state_b,
  const rclcpp::Time & sample_time, trajectory_msgs::msg::JointTrajectoryPoint & output)
{
  rclcpp::Duration duration_so_far = sample_time - time_a;
  rclcpp::Duration duration_btwn_points = time_b - time_a;

  const size_t dim = state_a.positions.size();
  output.positions.resize(dim, 0.0);
  output.velocities.resize(dim, 0.0);
  output.accelerations.resize(dim, 0.0);

  auto generate_powers = [](int n, double x, double * powers)
  {
    powers[0] = 1.0;
    for (int i = 1; i <= n; ++i)
    {
      powers[i] = powers[i - 1] * x;
    }
  };

  bool has_velocity = !state_a.velocities.empty() && !state_b.velocities.empty();
  bool has_accel = !state_a.accelerations.empty() && !state_b.accelerations.empty();
  if (duration_so_far.seconds() < 0.0)
  {
    duration_so_far = rclcpp::Duration::from_seconds(0.0);
    has_velocity = has_accel = false;
  }
  if (duration_so_far.seconds() > duration_btwn_points.seconds())
  {
    duration_so_far = duration_btwn_points;
    has_velocity = has_accel = false;
  }

  double t[6];
  generate_powers(5, duration_so_far.seconds(), t);

  if (!has_velocity && !has_accel)
  {
    // do linear interpolation
    for (size_t i = 0; i < dim; ++i)
    {
      double start_pos = state_a.positions[i];
      double end_pos = state_b.positions[i];

      double coefficients[2] = {0.0, 0.0};
      coefficients[0] = start_pos;
      if (duration_btwn_points.seconds() != 0.0)
      {
        coefficients[1] = (end_pos - start_pos) / duration_btwn_points.seconds();
      }

      output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1];
      output.velocities[i] = t[0] * coefficients[1];
    }
  }
  else if (has_velocity && !has_accel)
  {
    // do cubic interpolation
    double T[4];
    generate_powers(3, duration_btwn_points.seconds(), T);

    for (size_t i = 0; i < dim; ++i)
    {
      double start_pos = state_a.positions[i];
      double start_vel = state_a.velocities[i];
      double end_pos = state_b.positions[i];
      double end_vel = state_b.velocities[i];

      double coefficients[4] = {0.0, 0.0, 0.0, 0.0};
      coefficients[0] = start_pos;
      coefficients[1] = start_vel;
      if (duration_btwn_points.seconds() != 0.0)
      {
        coefficients[2] =
          (-3.0 * start_pos + 3.0 * end_pos - 2.0 * start_vel * T[1] - end_vel * T[1]) / T[2];
        coefficients[3] =
          (2.0 * start_pos - 2.0 * end_pos + start_vel * T[1] + end_vel * T[1]) / T[3];
      }

      output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1] +
                            t[2] * coefficients[2] + t[3] * coefficients[3];
      output.velocities[i] =
        t[0] * coefficients[1] + t[1] * 2.0 * coefficients[2] + t[2] * 3.0 * coefficients[3];
      output.accelerations[i] = t[0] * 2.0 * coefficients[2] + t[1] * 6.0 * coefficients[3];
    }
  }
  else if (has_velocity && has_accel)
  {
    // do quintic interpolation
    double T[6];
    generate_powers(5, duration_btwn_points.seconds(), T);

    for (size_t i = 0; i < dim; ++i)
    {
      double start_pos = state_a.positions[i];
      double start_vel = state_a.velocities[i];
      double start_acc = state_a.accelerations[i];
      double end_pos = state_b.positions[i];
      double end_vel = state_b.velocities[i];
      double end_acc = state_b.accelerations[i];

      double coefficients[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      coefficients[0] = start_pos;
      coefficients[1] = start_vel;
      coefficients[2] = 0.5 * start_acc;
      if (duration_btwn_points.seconds() != 0.0)
      {
        coefficients[3] = (-20.0 * start_pos + 20.0 * end_pos - 3.0 * start_acc * T[2] +
                           end_acc * T[2] - 12.0 * start_vel * T[1] - 8.0 * end_vel * T[1]) /
                          (2.0 * T[3]);
        coefficients[4] = (30.0 * start_pos - 30.0 * end_pos + 3.0 * start_acc * T[2] -
                           2.0 * end_acc * T[2] + 16.0 * start_vel * T[1] + 14.0 * end_vel * T[1]) /
                          (2.0 * T[4]);
        coefficients[5] = (-12.0 * start_pos + 12.0 * end_pos - start_acc * T[2] + end_acc * T[2] -
                           6.0 * start_vel * T[1] - 6.0 * end_vel * T[1]) /
                          (2.0 * T[5]);
      }

      output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1] +
                            t[2] * coefficients[2] + t[3] * coefficients[3] +
                            t[4] * coefficients[4] + t[5] * coefficients[5];
      output.velocities[i] = t[0] * coefficients[1] + t[1] * 2.0 * coefficients[2] +
                             t[2] * 3.0 * coefficients[3] + t[3] * 4.0 * coefficients[4] +
                             t[4] * 5.0 * coefficients[5];
      output.accelerations[i] = t[0] * 2.0 * coefficients[2] + t[1] * 6.0 * coefficients[3] +
                                t[2] * 12.0 * coefficients[4] + t[3] * 20.0 * coefficients[5];
    }
  }
}

void Trajectory::deduce_from_derivatives(
  trajectory_msgs::msg::JointTrajectoryPoint & first_state,
  trajectory_msgs::msg::JointTrajectoryPoint & second_state, const size_t dim, const double delta_t)
{
  if (second_state.positions.empty())
  {
    second_state.positions.resize(dim);
    if (first_state.velocities.empty())
    {
      first_state.velocities.resize(dim, 0.0);
    }
    if (second_state.velocities.empty())
    {
      second_state.velocities.resize(dim);
      if (first_state.accelerations.empty())
      {
        first_state.accelerations.resize(dim, 0.0);
      }
      for (size_t i = 0; i < dim; ++i)
      {
        second_state.velocities[i] =
          first_state.velocities[i] +
          (first_state.accelerations[i] + second_state.accelerations[i]) * 0.5 * delta_t;
      }
    }
    for (size_t i = 0; i < dim; ++i)
    {
      // second state velocity should be reached on the end of the segment, so use middle
      second_state.positions[i] =
        first_state.positions[i] +
        (first_state.velocities[i] + second_state.velocities[i]) * 0.5 * delta_t;
    }
  }
}

TrajectoryPointConstIter Trajectory::begin() const
{
  THROW_ON_NULLPTR(trajectory_msg_)

  return trajectory_msg_->points.begin();
}

TrajectoryPointConstIter Trajectory::end() const
{
  THROW_ON_NULLPTR(trajectory_msg_)

  return trajectory_msg_->points.end();
}

rclcpp::Time Trajectory::time_from_start() const { return trajectory_start_time_; }

bool Trajectory::has_trajectory_msg() const { return trajectory_msg_.get() != nullptr; }

}  // namespace joint_trajectory_controller
