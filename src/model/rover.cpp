#include "rover.hpp"
#include "ControlUtils.hpp"
using namespace ControlUtils
;


template <typename T>
Mclrover<T>::Mclrover()
{
  model_ptr_ = std::make_shared<model>();
  traj_ptr_ = std::make_shared<Traj>();
  ctrl_ptr_ = std::make_shared<ctrl>();

}

template <typename T>
void Mclrover<T>::BodyTrajectory(T time)
{

  //! 종방향 속도 , Yaw Rate control
  double Max_speed = 0.2;
  double time_limit = 3;
  double acc = Max_speed/(time_limit-1);
  double Drive_Gear_ratio = 1;
  double f = 0.1;
  double pi = 3.141592;

  // double V_y_mag = 10*pi/(180*400);
  double Yaw_rate_mag = 0.15;


  if (0 <= time && time < 1)
  {
    traj_ptr_->traj_body_vel_[0] = 0;
    traj_ptr_->traj_body_omega_[2] = 0;
  }
  else if (1 <= time && time < time_limit)
  {
    traj_ptr_->traj_body_vel_[0] = acc * (time - 1);
    traj_ptr_->traj_body_omega_[2] = 0;
  }
  else
  {
    traj_ptr_->traj_body_vel_[0] = Max_speed;
    traj_ptr_->traj_body_omega_[2] = Yaw_rate_mag * sin(2 * M_PI * f * (time - time_limit));
  }

}



template <typename T>
std::shared_ptr<typename Mclrover<T>::model> Mclrover<T>::set_model_ptr()
{
  return model_ptr_;
}

template <typename T>
std::shared_ptr<typename Mclrover<T>::Traj> Mclrover<T>::set_traj_ptr()
{
  return traj_ptr_;
}

template <typename T>
std::shared_ptr<typename Mclrover<T>::ctrl> Mclrover<T>::set_ctrl_ptr()
{
  return ctrl_ptr_;
}

template class Mclrover<double>;
template class Mclrover<float>;
