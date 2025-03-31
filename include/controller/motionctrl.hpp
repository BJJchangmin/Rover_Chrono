#ifndef ROVER_MOTION_CTRL
#define ROVER_MOTION_CTRL


#include "rover.hpp"


template <typename T>
class motionctrl
{

  private:

    T J_sus_[4], J_steer_[4], J_drive_[4]; // Motor Inertia from Identification
    T B_sus_[4], B_steer_[4], B_drive_[4]; // Damping from Identification

    //** PID Control Variable */
    Vec2<T> error_sus_[4], error_dot_sus_[4], error_int_sus_[4], pid_out_sus_[4]; // [i][0] : Current, [i][1] : Old
    Vec2<T> error_steer_[4], error_dot_steer_[4], error_int_steer_[4], pid_out_steer_[4]; // [i][0] : Current, [i][1] : Old
    Vec2<T> error_drive_[4], error_drive_anti_[4], error_dot_drive_[4], error_int_drive_[4], pid_out_drive_[4]; // [i][0] : Current, [i][1] : Old

    std::shared_ptr<typename Mclrover<T>::model> model_ptr_;
    std::shared_ptr<typename Mclrover<T>::Traj> traj_ptr_;
    std::shared_ptr<typename Mclrover<T>::ctrl> ctrl_ptr_;


  public:

    explicit motionctrl();

    void M_PID_drive(); // Drive Motor PID Control
    void M_PID_steer(); // Steer Motor PID Control
    void M_PID_sus(); // Suspension Motor PID Control

    void drive_motor_control(); // Drive Motor Control(PID+FF+DOB)
    void suspension_motor_control(); // Suspension Motor Control(PID+FF+DOB)
    void steer_motor_control(); // Steer Motor Control(PID+FF+DOB)

    void slip_ratio_estimation(); // Slip Ratio Estimation

    void get_Mclrovoer_ptr(
      std::shared_ptr<typename Mclrover<T>::model> model_ptr,
      std::shared_ptr<typename Mclrover<T>::Traj> traj_ptr,
      std::shared_ptr<typename Mclrover<T>::ctrl> ctrl_ptr
    );


};





#endif
