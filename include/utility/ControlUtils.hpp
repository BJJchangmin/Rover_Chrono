#ifndef CONTROL_UTILS_HPP
#define CONTROL_UTILS_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <limits>   // Required for numeric_limits (used in FilterLPF1)
#include <stdexcept> // For potential exception handling (e.g., invalid Ts)
#include <iostream>  // For potential error logging

using namespace Eigen;

// M_PI 정의
#ifndef M_PI
#define M_PI 3.14159265358979323846

#endif

#ifndef Ts
#define Ts 0.001 // Default sampling time
#endif

namespace ControlUtils {



template <typename T>
inline T saturation_block(T up_limit, T low_limit, T input)
{
  if(input > up_limit) { return up_limit; }
  if(input < low_limit) { return low_limit; }
  return input;

}

template <typename T>
inline T Max(T input, T compare)
{
    if(input > compare) { return input; }
    return compare;
}

// --- Gain Calculation Functions (Declarations) ---
Vector3d PD_Gain_posctrl(double J, double B, double zeta, double Wn_Hz);
Vector3d PI_Gain_velctrl(double J, double B, double zeta, double Wn_Hz);


template <typename T = double>
class Integrator {

private :
  T input_old_ = T(0);
  T output_old_ = T(0);
  T Ts_ = Ts;

public :
  explicit Integrator() {
    if (Ts_ <= T(0)) {
      std::cerr << "Error: Sampling time must be positive." << std::endl;
      Ts_ = T(Ts); // Default value if invalid
    }
  }

  T process(T input)
  {
    T output = (Ts_ / T(2.0)) * (input + input_old_) + output_old_;

    input_old_ = input;     // x[n] becomes x[n-1]
    output_old_ = output;   // y[n] becomes y[n-1]
    return output;          // Return the current integral value y[n]

  }

  T get_integral_value() {
    return output_old_;
  }


};


template <typename T>
class LPF {

private:
  T input_old_ = T(0);
  T output_old_ = T(0);
  T time_constant_ = T(0);
  T cutoff_freq_ = T(0);
  T Ts_ = Ts;


public:
  LPF(){}

  T process(T input,T cutoff_freq) {

    time_constant_ = T(1.0) / (T(2.0) * M_PI * cutoff_freq);

    T output = (Ts * (input + input_old_) - (Ts - 2 * time_constant_) * output_old_) / (Ts + 2 * time_constant_);

    // Update states for the next iteration
    input_old_ = input;
    output_old_ = output;
    return output;
    }

};

template <typename T>
class LPF_2nd{

private:
  T input_old_ = T(0);
  T input_old2_ = T(0);
  T output_old_ = T(0);
  T output_old2_ = T(0);
  T Ts_;

public:
  LPF_2nd(){
    input_old_ = T(0);
    input_old2_ = T(0);
    output_old_ = T(0);
    output_old2_ = T(0);
    Ts_ = Ts;
  }

  T process(T input, T cutoff_freq, T zeta)
  {
    T wn = 2 * M_PI * cutoff_freq;
    T a = pow(wn,2)*pow(Ts,2)*(input + 2*input_old_ + input_old2_);
    T y_param = 4 + 4*zeta*wn*Ts + pow(wn*Ts,2);
    T y_old_param = 2*pow(wn*Ts,2) - 8;
    T y_old2_param = 4 - 4*zeta*wn*Ts + pow(wn*Ts,2);

    T output = (a - y_old_param*output_old_ - y_old2_param*output_old2_) / y_param;



    // 다음 계산을 위해 상태 변수 업데이트
    input_old2_ = input_old_;
    input_old_  = input;
    output_old2_= output_old_;
    output_old_ = output;

    return output;


  }


};

template <typename T>
class dot{

private:
  T input_old_;
  T output_old_;
  T coeff_;

public:

  explicit dot()
  {
    coeff_ = T(2.0) / Ts;
    input_old_ = T(0);
    output_old_ = T(0);
  }

  T process(T input)
  {
    T output = coeff_ * (input - input_old_) - output_old_;

    input_old_ = input;     // x[n] becomes x[n-1]
    output_old_ = output;   // y[n] becomes y[n-1]
    return output;          // Return the current integral value y[n]
  }



};

template <typename T>
class tustin_derivate {

private:
  T input_old_ = T(0);
  T output_old_ = T(0);
  T time_constant_ = T(0);
  T cutoff_freq_ = T(0);
  T Ts_ = Ts;

public:

  explicit tustin_derivate()
  {
    input_old_ = T(0);
    output_old_ = T(0);
  }

  T process(T input, T cutoff_freq)
  {
    time_constant_ = T(1.0) / (T(2.0) * M_PI * cutoff_freq);

    T output = (2 * (input - input_old_) - (Ts - 2 * time_constant_) * output_old_) / (Ts + 2 * time_constant_);

    // Update states for the next iteration
    input_old_ = input;
    output_old_ = output;
    return output;
  }

};


template <typename T>
class Feedforward_1st {

private:
  T J_;
  T B_;

  tustin_derivate<T> vel_dot_;
  LPF<T> tau_lpf_;

public:
  Feedforward_1st(){}

  T control( T J, T B, T Vel_command, T ff_cutoff_freq) {

    T vel_cmd_dot = vel_dot_.process(Vel_command,30);

    T tau_ff = J_ * vel_cmd_dot + B_*Vel_command;

    T tau_ff_lpf = tau_lpf_.process(tau_ff,ff_cutoff_freq);

    return tau_ff_lpf;

  }

};

template <typename T>
class Feedforward_2nd {

private:

  // dot<T> pos_to_vel_;
  // dot<T> vel_to_acc_;
  // LPF_2nd<T> tau_lpf_;

  tustin_derivate<T> pos_to_vel_;
  tustin_derivate<T> vel_to_acc_;


  T vel_to_acc_2;

public:
  Feedforward_2nd(){
    vel_to_acc_2 = T(0);
  }

  T control( T J, T B, T K, T Pos_cmd, T ff_cutoff_freq, T ff_zeta){

    T pos_to_vel_cmd = pos_to_vel_.process(Pos_cmd,ff_cutoff_freq);
    T vel_to_acc_cmd = vel_to_acc_.process(pos_to_vel_cmd,ff_cutoff_freq);

    vel_to_acc_2 = pos_to_vel_cmd;

    T tau_ff_raw = J * vel_to_acc_cmd + B * pos_to_vel_cmd + K * Pos_cmd;

    // T tau_ff_lpf = tau_lpf_.process(tau_ff_raw,ff_cutoff_freq, ff_zeta);

    return tau_ff_raw;

  }

  T cmd_check()
  {
    return vel_to_acc_2;
  }

};


template <typename T>
class DOB {

private:
  T Jn_;
  T Bn_;

  tustin_derivate<T> vel_dot_;
  LPF<T> q_filter_;

public:

  DOB(){}

  T control(T J , T B, T ctrl_input, T measured_vel, T dob_cut_off_freq) {

    T acc_est = vel_dot_.process(measured_vel, 20);

    T nominal_torque = J * acc_est + B * measured_vel;

    T d_hat_raw = ctrl_input - nominal_torque;

    T d_hat_filtered = q_filter_.process(d_hat_raw, dob_cut_off_freq);


    return d_hat_filtered;

  }

};

template <typename T>
class Antiwindup {

private:
  T gain_;

public:

  explicit Antiwindup(T Anti_windup_gain) : gain_(Anti_windup_gain) {
    if (gain_ <= T(0)) {
      std::cerr << "Warning: Invalid parameters for LPF1. Check Ts > 0 and cutoff_freq > 0. Using pass-through." << std::endl;
      // Set coefficients for pass-through behavior y[n] = x[n]
      // This happens if denominator is 0 or parameters are invalid
      return;
    }
  }

  T cal_saturation_error(T saturation_error) const{

    return gain_ * saturation_error;

  }


};

template <typename T>
class PIcontroller {

  //! PI Controller need to be fix

private:

  T kp_;
  T ki_;
  T i_error_;

  Integrator<T> integrator_;

  T current_error_;

public:

  PIcontroller() : integrator_() {
    kp_ = T(0);
    ki_ = T(0);
    current_error_ = T(0);
    i_error_ = T(0);
  }

  T contorl(T kp, T ki, T vel_ref, T act_vel, T Anti_windup_error) {

    kp_ = kp;
    ki_ = ki;

    current_error_ = vel_ref - act_vel;

    T p_term = kp_ * current_error_;

    i_error_ = current_error_ - Anti_windup_error;

    T new_i_error = integrator_.process(i_error_);

    T i_term = ki_ * new_i_error;

    T PI_output = p_term + i_term;

    return PI_output;

  }

};

template <typename T>
class PDcontroller {

private:
  T kp_;
  T kd_;

  tustin_derivate<T> error_dot;

public:
  PDcontroller(){}

  T control(T kp, T kd, T pos_ref, T pos_act, T err_cut_off)
  {
    kp_ = kp;
    kd_ = kd;

    T error = pos_ref - pos_act;

    T error_dot_ = error_dot.process(error, err_cut_off);

    T PD_output = kp_ * error + kd_ * error_dot_;

    return PD_output;
  }

};
















};

#endif