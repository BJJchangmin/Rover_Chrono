#ifndef ROVER_MOTION_CTRL
#define ROVER_MOTION_CTRL


#include "rover.hpp"
#include "ControlUtils.hpp"


template <typename T>
class motionctrl
{

  private:


    ControlUtils::LPF<T> soil_ff_ctrl_lpf_[4];
    ControlUtils::PIcontroller<T> drive_pi_ctrl_[4];
    ControlUtils::Feedforward_1st<T> drive_ff_ctrl_[4];

    ControlUtils::PDcontroller<T> steer_pd_ctrl_[4];
    ControlUtils::Feedforward_2nd<T> steer_ff_ctrl_[4];
    ControlUtils::DOB<T> steer_dob_[4];

    ControlUtils::PIcontroller<T> lateral_PI_ctrl_;
    ControlUtils::LPF<T> lateral_vel_feedback_lpf_;
    ControlUtils::LPF<T> steer_M_ref_;

    T J_sus_[4], J_steer_[4], J_drive_[4]; // Motor Inertia from Identification
    T B_sus_[4], B_steer_[4], B_drive_[4]; // Damping from Identification

    //** PID Control Variable */
    Vec2<T> error_sus_[4], error_dot_sus_[4], error_int_sus_[4], pid_out_sus_[4]; // [i][0] : Current, [i][1] : Old
    Vec2<T> error_steer_[4], error_dot_steer_[4], error_int_steer_[4], pid_out_steer_[4]; // [i][0] : Current, [i][1] : Old
    Vec2<T> steer_ctrl_input_[4]; // [i][0] : Current, [i][1] : Old

    //** Compensation */
    T soil_compensation[4];

    //** Anti_Wind_up */
    T dri_track_error_[4]; // Variable for Anti-Windup

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
    void alpha_estimation(); // Tire Slip Angle Estimation

    void soil_resistance_compensation(); // Soil Resistance Compensation

    void longitudinal_vel_ctrl();
    void Lateral_vel_ctrl(T time);

    void Kin_High_level_ctrl();
    void Kinematic_Mapping();
    void Kin_Low_level_ctrl();

    void get_Mclrovoer_ptr(
      std::shared_ptr<typename Mclrover<T>::model> model_ptr,
      std::shared_ptr<typename Mclrover<T>::Traj> traj_ptr,
      std::shared_ptr<typename Mclrover<T>::ctrl> ctrl_ptr
    );


};





#endif
