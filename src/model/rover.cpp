#include "rover.hpp"

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

  //! 종방향 속도 Reference는 종방향 속도로 생각

  double Max_speed = 0.3/0.25;
  double time_limit = 3;
  double acc = Max_speed/(time_limit-1);
  double Drive_Gear_ratio = 1;


  if(0<=time && time<1)
  {
    traj_ptr_->traj_body_pos_[0] = 0;
    traj_ptr_->traj_body_pos_[1] = 0;
    traj_ptr_->traj_body_vel_[2] = 0;
  }
  else if(1 <= time && time < time_limit )
  {
    traj_ptr_->traj_body_pos_[0] = 0;
    traj_ptr_->traj_body_pos_[1] = 0;
    traj_ptr_->traj_body_vel_[2] = acc*(time-1);
  }
  else
  {
    traj_ptr_->traj_body_pos_[0] = 0;
    traj_ptr_->traj_body_pos_[1] = 0;
    traj_ptr_->traj_body_vel_[2] = Max_speed;
  }


  traj_ptr_->traj_body_vel_[2] = traj_ptr_->traj_body_vel_[2]/Drive_Gear_ratio;



}

template <typename T>
void Mclrover<T>::MotorTrajectory(T time)
{
  for(int i=0; i<4; i++)
  {
    for(int j=0; j<2; j++)
    {
      traj_ptr_->traj_motor_pos_[i][j] = 0;
    }

    traj_ptr_->traj_motor_vel_[i][2] = traj_ptr_->traj_body_vel_[2];
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
