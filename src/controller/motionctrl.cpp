#include "motionctrl.hpp"
#include "Useful.hpp"


template <typename T>
motionctrl<T>::motionctrl()
{

  model_ptr_ = nullptr;
  traj_ptr_ = nullptr;
  ctrl_ptr_ = nullptr;

  for (size_t i = 0; i < 4; i++)
  {
    J_sus_[i] = 0.04; J_steer_[i] = 0.3; J_drive_[i] = 0.09;
    B_sus_[i] = 0.0101; B_steer_[i] = 0.0084; B_drive_[i] = 0.0034;

    error_sus_[i][0] = 0; error_sus_[i][1] = 0;
    error_dot_sus_[i][0] = 0; error_dot_sus_[i][1] = 0;
    error_int_sus_[i][0] = 0; error_int_sus_[i][1] = 0;
    pid_out_sus_[i][0] = 0; pid_out_sus_[i][1] = 0;

    error_steer_[i][0] = 0; error_steer_[i][1] = 0;
    error_dot_steer_[i][0] = 0; error_dot_steer_[i][1] = 0;
    error_int_steer_[i][0] = 0; error_int_steer_[i][1] = 0;
    pid_out_steer_[i][0] = 0; pid_out_steer_[i][1] = 0;

    error_drive_[i][0] = 0; error_drive_[i][1] = 0;
    error_dot_drive_[i][0] = 0; error_dot_drive_[i][1] = 0;
    error_int_drive_[i][0] = 0; error_int_drive_[i][1] = 0;
    pid_out_drive_[i][0] = 0; pid_out_drive_[i][1] = 0;

  }

}

template <typename T>
void motionctrl<T>::drive_motor_control()
{

  M_PID_drive();
  slip_ratio_estimation();

  for(size_t i = 0; i < 4; i++)
  {
    ctrl_ptr_->drive_torque_[i] = 40*saturation_block(12.8, -12.8, pid_out_drive_[i][0]);
    pid_out_drive_[i][1] = pid_out_drive_[i][0];
  }

}

template <typename T>
void motionctrl<T>::M_PID_drive()
{

  //Drive Motor Control
  Vec4 <T> ctrl_input;
  Vec4 <T> track_error; // Variable for Anti-Windup


  Vector3d K[4];

  for (size_t i = 0; i < 4; i++)
  {
    // Gain Setting
    // K[i] = PI_Gain_velctrl(J_drive_[i], B_drive_[i], 5, 10);
    K[i][0] = 10;
    K[i][1] = 2;

    // Error Calculation
    error_drive_[i][0] = traj_ptr_->traj_motor_vel_[i][2] - model_ptr_->joint_vel_act_[i][2];

    //********** */ Calculate for Anti-windup ******************************************** */
    //todo: Anti-windup on/off 기능 추가
    //!Anti-windup error 계산에 문제있음
    track_error[i] = pid_out_drive_[i][0] - ctrl_ptr_->drive_torque_[i];
    // std::cout << "track_error : " << track_error[i] << std::endl;

    //************************************ */

    error_drive_anti_[i][0] = Anti_Windup(0, track_error[i], error_drive_[i][0]);
    error_int_drive_[i][0] = integrate(error_drive_anti_[i][0], error_drive_anti_[i][1], error_int_drive_[i][1]);

    pid_out_drive_[i][0] = K[i][0]* error_drive_[i][0] + K[i][1] * error_int_drive_[i][0];


    // Error Update
    error_drive_[i][1] = error_drive_[i][0];
    error_drive_anti_[i][1] = error_drive_anti_[i][0];
    error_dot_drive_[i][1] = error_dot_drive_[i][0];
    error_int_drive_[i][1] = error_int_drive_[i][0];
  }


}

template <typename T>
void motionctrl<T>::slip_ratio_estimation()
{
  T radius = 0.25;
  T Rover_vel_x;
  double wheel_vel[4];

  Rover_vel_x = model_ptr_->body_vel_[0];

  for(size_t i = 0; i < 4; i++)
  {
    wheel_vel[i] = Max(model_ptr_->joint_vel_act_[i][2] * radius , 0.00001) ;

    model_ptr_->slip_ratio_[i] = (wheel_vel[i] - Rover_vel_x) / wheel_vel[i];

    model_ptr_->slip_ratio_[i] = saturation_block(1, -1, model_ptr_->slip_ratio_[i]);

  }

}

template <typename T>
void motionctrl<T>::get_Mclrovoer_ptr(
  std::shared_ptr<typename Mclrover<T>::model> model_ptr,
  std::shared_ptr<typename Mclrover<T>::Traj> traj_ptr,
  std::shared_ptr<typename Mclrover<T>::ctrl> ctrl_ptr
)
{
  model_ptr_ = model_ptr;
  traj_ptr_ = traj_ptr;
  ctrl_ptr_ = ctrl_ptr;
}

template class motionctrl<double>;
template class motionctrl<float>;



