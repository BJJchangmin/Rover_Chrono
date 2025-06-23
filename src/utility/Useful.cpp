#include "Useful.hpp"

using namespace Eigen;

double Ts = 0.001;
double pi = 3.141592;

double saturation_block(double up_limit, double low_limit, double input)
{
 if(input > up_limit)
 {
    return up_limit;
 }
 else if(input < low_limit)
 {
    return low_limit;
 }
 else
 {
    return input;
 }
}

double Max(double input, double compare)
{
  if(input > compare)
  {
    return input;
  }
  else
  {
    return compare;
  }
}


Vector3d PD_Gain_posctrl(double J, double B, double zeta, double W_n) // J, B, zeta, W_n
{
    //! Gain Calculate by Pole Placement
    //* J : Inertia
    //* B : Damping
    //* zeta : Damping Ratio
    //* W_n : Natural Frequency
    //* K[0] : Kp, K[1] : Ki, K[2] : Kd

    Vector3d K;
    W_n = W_n*2*pi;

    K[0] = J*W_n*W_n;
    K[1] = 0;
    K[2] = 2*zeta*W_n*J - B;
    return K;
}

Vector3d PI_Gain_velctrl(double J, double B, double zeta, double W_n) // J, B, zeta, W_n
{
    //! Gain Calculate by Pole Placement
    //* J : Inertia
    //* B : Damping
    //* zeta : Damping Ratio
    //* W_n : Natural Frequency
    //* K[0] : Kp, K[1] : Ki, K[2] : Kd

    Vector3d K;
    W_n = W_n*2*pi;

    K[0] = 2*zeta*W_n*J - B; //Kp
    K[1] = J*W_n*W_n; //Ki
    K[2] = 0; //Kd
    return K;
}

double fst_order_model_gain(double freq1, double w_n)
{
  double Kp = 2*M_PI*freq1*w_n-1;
  return Kp;
}





