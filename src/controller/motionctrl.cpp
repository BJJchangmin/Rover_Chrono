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
    J_sus_[i] = 0.1; J_steer_[i] = 0.1; J_drive_[i] = 0.1;
    B_sus_[i] = 0.0; B_steer_[i] = 0; B_drive_[i] = 0.0;

    error_sus_[i][0] = 0; error_sus_[i][1] = 0;
    error_dot_sus_[i][0] = 0; error_dot_sus_[i][1] = 0;
    error_int_sus_[i][0] = 0; error_int_sus_[i][1] = 0;
    pid_out_sus_[i][0] = 0; pid_out_sus_[i][1] = 0;

    error_steer_[i][0] = 0; error_steer_[i][1] = 0;
    error_dot_steer_[i][0] = 0; error_dot_steer_[i][1] = 0;
    error_int_steer_[i][0] = 0; error_int_steer_[i][1] = 0;
    pid_out_steer_[i][0] = 0; pid_out_steer_[i][1] = 0;

    dri_track_error_[i] = 0;

    steer_ctrl_input_[i][0] = 0;
    steer_ctrl_input_[i][1] = 0;

  }

}

template <typename T>
void motionctrl<T>::drive_motor_control()
{
  Vector3d K[4]; // Gain

  slip_ratio_estimation();
  soil_resistance_compensation();
  M_PID_drive();





  for(size_t i = 0; i < 4; i++)
  {

    T pid_unsaturated = ctrl_ptr_->drive_pid_ctrl_[i] + ctrl_ptr_->soil_compensation_[1] ;

    T pid_saturated = saturation_block(12.8, -12.8, pid_unsaturated);

    ctrl_ptr_->drive_torque_[i] = 40*pid_saturated;

    dri_track_error_[i] = 1*(pid_unsaturated - pid_saturated); // for Anti-Windup

  }

}

template <typename T>
void motionctrl<T>::steer_motor_control()
{
  M_PID_steer();

  for(size_t i = 0; i < 4; i++)
  {
    ctrl_ptr_->steer_ff_ctrl_[i] = steer_ff_ctrl_[i].control(J_steer_[i], B_steer_[i], 0, traj_ptr_->traj_motor_pos_[i][1], 10, 1);

    ctrl_ptr_->steer_dob_ctrl_[i] = steer_dob_[i].control(J_steer_[i], B_steer_[i], steer_ctrl_input_[i][1], model_ptr_->joint_vel_act_[i][1], 5);

    T ctrl_unsaturated = ctrl_ptr_->steer_pid_ctrl_[i] + ctrl_ptr_->steer_ff_ctrl_[i] + ctrl_ptr_->steer_dob_ctrl_[i];

    T ctrl_saturated = saturation_block(9.87, -9.87, ctrl_unsaturated);

    steer_ctrl_input_[i][0] = ctrl_saturated;

    ctrl_ptr_->steer_torque_[i] = 1*ctrl_saturated;

    steer_ctrl_input_[i][1] = steer_ctrl_input_[i][0] ;

  }

}

template <typename T>
void motionctrl<T>::M_PID_drive()
{

  Vector3d K[4]; // Gain

  for (size_t i = 0; i < 4; i++)
  {
    // Gain Setting
    K[i] = PI_Gain_velctrl(0.1, 0, 1, 1.5*3);
    // contorl(T kp, T ki, T vel_ref, T act_vel, T Anti_windup_error)
    ctrl_ptr_->drive_pid_ctrl_[i] = drive_pi_ctrl_[i].contorl(K[i][0], K[i][1],traj_ptr_->traj_motor_vel_[i][2],model_ptr_->joint_vel_act_[i][2],dri_track_error_[i] );
  }


}

template <typename T>
void motionctrl<T>::M_PID_steer()
{
  Vector3d K[4]; // Gain

  for (size_t i = 0; i < 4; i++)
  {
    // Gain Setting
    K[i] = PD_Gain_posctrl(0.1, 0, 1.5, 1.5*3);

    //control(T kp, T kd, T pos_ref, T pos_act, T err_cut_off)
    ctrl_ptr_->steer_pid_ctrl_[i] = steer_pd_ctrl_[i].control(K[i][0], K[i][2], traj_ptr_->traj_motor_pos_[i][1], model_ptr_->joint_pos_act_[i][1], 100);
  }

}


template <typename T>
void motionctrl<T>::slip_ratio_estimation()
{
  T radius = 0.25;
  T Rover_vel_x;
  double wheel_vel[4];



  for(size_t i = 0; i < 4; i++)
  {
    Rover_vel_x = model_ptr_->wheel_lin_vel_[i][0];

    wheel_vel[i] = model_ptr_->joint_vel_act_[i][2] * radius;

    model_ptr_->slip_ratio_[i] = (wheel_vel[i] - Rover_vel_x) / Max( wheel_vel[i], 0.01) ;

    model_ptr_->slip_ratio_[i] = saturation_block(1, -1, model_ptr_->slip_ratio_[i]);

  }

}

template <typename T>
void motionctrl<T>::alpha_estimation()
{
  T v_x;
  for (size_t i = 0; i < 4; i++)
  {
    v_x = Max(model_ptr_->wheel_lin_vel_[i][0], 0.0001);
    model_ptr_->tire_alpha_[i] = atan2(model_ptr_->wheel_lin_vel_[i][1], v_x);
  }
}

template <typename T>
void motionctrl<T>::longitudinal_vel_ctrl()
{
  T err;
  T K_rov_gain = 4;

  err = traj_ptr_->traj_body_vel_[0] - model_ptr_->body_vel_[0];
  ctrl_ptr_->slip_ref_ = saturation_block(0.3,0,K_rov_gain*err);

  for (size_t i = 0; i < 4; i++)
  {
    traj_ptr_->traj_motor_vel_[i][2] = model_ptr_->body_vel_[0]/Max(0.01,((1-ctrl_ptr_->slip_ref_)*0.25));

    // traj_ptr_->traj_motor_vel_[i][2] = traj_ptr_->traj_body_vel_[0]/0.25;
  }

}

template <typename T>
void motionctrl<T>::Kin_High_level_ctrl()
{

  T new_yaw_rate;
  T new_vx_vel;

  T kp_vx = fst_order_model_gain(1, 0.6);
  T kp_gamma = fst_order_model_gain(1, 0.12);

  // T kp_vx = 5;
  // T kp_gamma = 1;

  traj_ptr_->traj_body_omega_[2] = traj_ptr_->traj_body_omega_[2] + kp_gamma * (traj_ptr_->traj_body_omega_[2] - model_ptr_->body_ang_vel_[2]);

  traj_ptr_->traj_body_vel_[0]  = traj_ptr_->traj_body_vel_[0] + kp_vx * (traj_ptr_->traj_body_vel_[0] - model_ptr_->body_vel_[0]);


  // traj_ptr_->traj_body_omega_[2] = traj_ptr_->traj_body_omega_[2];

  // traj_ptr_->traj_body_vel_[0]  = traj_ptr_->traj_body_vel_[0];


}

template <typename T>
void motionctrl<T>::Kinematic_Mapping()
{

  double L = 1.2836;
  double W = 1.2196;

  T rov_ICR = Max(traj_ptr_->traj_body_vel_[0],0.000001)/std::copysign(Max(abs(traj_ptr_->traj_body_omega_[2]) , 0.00000001), traj_ptr_->traj_body_omega_[2]);

  T wheel_ICR[4];

  wheel_ICR[0] = sqrt(pow(L/2,2)+pow(rov_ICR-W/2,2));
  wheel_ICR[1] = sqrt(pow(L/2,2)+pow(rov_ICR+W/2,2));
  wheel_ICR[2] = sqrt(pow(L/2,2)+pow(rov_ICR-W/2,2));
  wheel_ICR[3] = sqrt(pow(L/2,2)+pow(rov_ICR+W/2,2));

  for (size_t i = 0; i < 4; i++)
  {
    traj_ptr_->traj_motor_vel_[i][2] = (traj_ptr_->traj_body_vel_[0] * abs(wheel_ICR[i] / abs(rov_ICR)))/0.25;
  }

  traj_ptr_->traj_motor_pos_[0][1] = saturation_block(60*M_PI_2/180, -60*M_PI_2/180, atan((L/2)/(rov_ICR-W/2))); // FL
  traj_ptr_->traj_motor_pos_[1][1] = saturation_block(60*M_PI_2/180, -60*M_PI_2/180, atan((L/2)/(rov_ICR+W/2))); // FR
  traj_ptr_->traj_motor_pos_[2][1] = saturation_block(60*M_PI_2/180, -60*M_PI_2/180, atan((-L/2)/(rov_ICR-W/2))); // RL
  traj_ptr_->traj_motor_pos_[3][1] = saturation_block(60*M_PI_2/180, -60*M_PI_2/180, atan((-L/2)/(rov_ICR+W/2))); // RR



}

template <typename T>
void motionctrl<T>::Kin_Low_level_ctrl()
{
  Kin_High_level_ctrl();
  Kinematic_Mapping();
  drive_motor_control();
  steer_motor_control();
  alpha_estimation();
}




template <typename T>
void motionctrl<T>::Lateral_vel_ctrl(T time)
{
  alpha_estimation();
  Vector3d K; // Gain
  double J = 1/(2*M_PI*20);
  K = PI_Gain_velctrl(J, 1, 1, 1);

  // std::cout << "K : " << K[0] << std::endl;

  double lpf_body_vel_act = lateral_vel_feedback_lpf_.process(model_ptr_->body_vel_[1], 1);

  double ctrl_vy_ref = lateral_PI_ctrl_.contorl(3,0, traj_ptr_->traj_body_vel_[1], lpf_body_vel_act, 0);

  double L = 1.2836;
  double W = 1.2196;
  double V_y = std::copysign( ControlUtils::Max<double>(abs(traj_ptr_->traj_body_vel_[1]+ctrl_vy_ref), 0.00001) ,traj_ptr_->traj_body_vel_[1]+ctrl_vy_ref );
  double V_x = traj_ptr_->traj_body_vel_[0];
  double R = (L*V_x/2)/V_y;




  double aaa = std::copysign( ControlUtils::Max<double>(abs(R-W/2), 0.0000001) ,R-W/2 );
  double bbb = std::copysign( ControlUtils::Max<double>(abs(R+W/2), 0.0000001) ,R+W/2 );


  double time_limit = 3;

  double Drive_Gear_ratio = 1;
  double f = 0.25 ;
  double pi = 3.141592;
  double T_half = 0.5 / f;


  if(0<=time && time<1)
  {
    traj_ptr_->traj_motor_pos_[0][1] = 0; // FL
    traj_ptr_->traj_motor_pos_[1][1] = 0; // FR
    traj_ptr_->traj_motor_pos_[2][1] = 0; // RL
    traj_ptr_->traj_motor_pos_[3][1] = 0; // RR



  }
  else if(1 <= time && time < time_limit )
  {
    traj_ptr_->traj_motor_pos_[0][1] = 0; // FL
    traj_ptr_->traj_motor_pos_[1][1] = 0; // FR
    traj_ptr_->traj_motor_pos_[2][1] = 0; // RL
    traj_ptr_->traj_motor_pos_[3][1] = 0; // RR

  }
  else
  {

    double steer_ref_= steer_M_ref_.process(-60*M_PI/180,0.13);
    // double steer_ref_= -50*M_PI/180*sin(2 * M_PI * f * (time - time_limit));

    // traj_ptr_->traj_motor_pos_[0][1] = atan((L/2)/(R-W/2)); // FL
    // traj_ptr_->traj_motor_pos_[1][1] = atan((L/2)/(R+W/2)); // FR
    // traj_ptr_->traj_motor_pos_[2][1] = atan((-L/2)/(R-W/2)); // RL
    // traj_ptr_->traj_motor_pos_[3][1] = atan((-L/2)/(R+W/2)); // RR

    // traj_ptr_->traj_motor_pos_[0][1] = atan((L/2)/(R-W/2)) + model_ptr_->tire_alpha_[0]; // FL
    // traj_ptr_->traj_motor_pos_[1][1] = atan((L/2)/(R+W/2)) + model_ptr_->tire_alpha_[1]; // FR
    // traj_ptr_->traj_motor_pos_[2][1] = atan((-L/2)/(R-W/2)) + model_ptr_->tire_alpha_[2]; // RL
    // traj_ptr_->traj_motor_pos_[3][1] = atan((-L/2)/(R+W/2)) + model_ptr_->tire_alpha_[3]; // RR


    traj_ptr_->traj_motor_pos_[0][1] = steer_ref_; // FL
    traj_ptr_->traj_motor_pos_[1][1] = steer_ref_; // FRㅉ
    traj_ptr_->traj_motor_pos_[2][1] = -steer_ref_; // RL
    traj_ptr_->traj_motor_pos_[3][1] = -steer_ref_; // RR


  }

}

template <typename T>
void motionctrl<T>::soil_resistance_compensation()
{
  T k_c = 0.14*pow(10,4);
  T k_phi = 0.82*pow(10,6);

  T ks = k_c/0.25 +k_phi;

  T c= 0.017*pow(10,4);
  T fric_ang = 35*M_PI/180;
  T janosi_k = 1.78*pow(10,-2);

  T N =1;

  T r = 0.25;
  T b = 0.3;

  T th_1;
  T th_m;

  T Horizontal_Normal_Stress_[4];
  T Vertical_Shear_Stress_[4];

  for(int i = 0; i < 4; i++)
  {
    // th_1 = acos(1-(model_ptr_->sinkage_[i]/r));
    // th_1 = 17*M_PI/180;

    th_1 = 6*M_PI/180;
    th_m = th_1 * 0.5;

    T A = sin(th_m)/th_m + (sin(th_m)-sin(th_1))/(th_1-th_m);
    T sigma_m = ks*pow((r*(cos(th_m)-cos(th_1))),N);
    T tau_m_1 = (c + sigma_m*tan(fric_ang));
    T exp_tau = -1*(r/janosi_k)*((th_1-th_m) - (1)*(sin(th_1)-sin(th_m)));
    T tau_m_2 = 1-exp(exp_tau);

    T B =  (cos(th_m) - 1)/th_m + (cos(th_m)-cos(th_1))/(th_1-th_m);

    Horizontal_Normal_Stress_[i] = (A*r*b*tau_m_1*tau_m_2);
    Vertical_Shear_Stress_[i] = (B*r*b*sigma_m);

    // ctrl_ptr_->soil_compensation_[i]  = 0; // Translate force to torque
    ctrl_ptr_->soil_compensation_[i]  = 0.4*soil_ff_ctrl_lpf_[i].process((Horizontal_Normal_Stress_[i] + Vertical_Shear_Stress_[i])*r,0.5) ;

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



