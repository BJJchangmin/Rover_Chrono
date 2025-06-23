#ifndef ROVER_MODEL
#define ROVER_MODEL

// C++ standard library
#include <iostream>
#include <memory>
#include <vector>

// Eigen3
#include <eigen3/Eigen/StdVector>

// customized headers
#include "cppTypes.hpp"

using std::vector;

template <typename T>
class Mclrover
{
  public:
    struct model
    {
      //Rover Body State
      Vec3<T> body_pos_, body_vel_, body_acc_;
      Vec4<T> body_ang_pos_;  // quaternion
      Vec3<T> body_ang_vel_, body_ang_acc_;

      Vec3<T> joint_pos_act_[4], joint_vel_act_[4]; // [i][j] = i: FL, FR, RL, RR, j: lift, steer, drive

      Vec3<T> contact_force_[4];
      Vec3<T> contact_torque_[4];

      T slip_ratio_[4];
      T tire_alpha_[4];
      T sinkage_[4];

      Vec3<T> wheel_lin_vel_[4];
    };

    struct Traj
    {
      Vec3<T> traj_motor_pos_[4];// [i][j] = i: FL(0), FR(1), RL(2), RR(3), j: lift(0), steer(1), drive(2)
      Vec3<T> traj_motor_vel_[4];// [i][j] = i: FL(0), FR(1), RL(2), RR(3), j: lift(0), steer(1), drive(2)
      Vec3<T> traj_body_pos_;// x(0), y(1), z(2) (전진, 화면, 상하)
      Vec3<T> traj_body_vel_;// x(0), y(1), z(2) (전진, 화면, 상하)
      Vec3<T> traj_body_omega_;
    };

    struct ctrl
    {
      T sus_torque_[4];  //i: FL(0), FR(1), RL(2), RR(3)
      T steer_torque_[4];  //i: FL(0), FR(1), RL(2), RR(3)
      T drive_torque_[4];  //i: FL(0), FR(1), RL(2), RR(3)
      T slip_ref_;

      T drive_pid_ctrl_[4];  //i: FL(0), FR(1), RL(2), RR(3)
      T soil_compensation_[4];  //i: FL(0), FR(1), RL(2), RR(3)

      T steer_pid_ctrl_[4];  //i: FL(0), FR(1), RL(2), RR(3)
      T steer_ff_ctrl_[4];  //i: FL(0), FR(1), RL(2), RR(3)
      T steer_dob_ctrl_[4];  //i: FL(0), FR(1), RL(2), RR(3)

    };



    std::shared_ptr<model> model_ptr_;
    std::shared_ptr<model> set_model_ptr();

    std::shared_ptr<Traj> traj_ptr_;
    std::shared_ptr<Traj> set_traj_ptr();

    std::shared_ptr<ctrl> ctrl_ptr_;
    std::shared_ptr<ctrl> set_ctrl_ptr();

  public:

    Mclrover();

    void BodyTrajectory(T time);
    void MotorTrajectory(T time);


};


#endif