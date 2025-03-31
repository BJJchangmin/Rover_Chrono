#include "data_logging.hpp"

using namespace std;

template <typename T>
DataLogging<T>::DataLogging()
{
  cout << "DataLogging object created" << endl;
  fout_[0].open("../plot/data_FL.csv");
  fout_[1].open("../plot/data_FR.csv");
  fout_[2].open("../plot/data_RL.csv");
  fout_[3].open("../plot/data_RR.csv");
  fout_[4].open("../plot/data_trunk.csv");

  model_ptr_ = nullptr;
  traj_ptr_ = nullptr;
  ctrl_ptr_ = nullptr;
  
  for (int i = 0; i < 5; i++)
  {
    if (!fout_[i])
    {
      std::cerr << "Cannot open file" << endl;
      exit(1);
    }
  }

  init_data();

}

template <typename T>
void DataLogging<T>::init_data()
{
  for (int i = 0; i < 4; i++)
  {
    if (!fout_[i])
    {
      std::cerr << "Cannot open file: " << i <<std::endl;
      exit(1);
    }
    else
    {
      fout_[i] << "Time, " ;
      fout_[i] << "sus_pos_ref, sus_pos, sus_vel, sus_torque, ";
      fout_[i] << "steer_pos_ref, steer_pos, steer_vel, steer_torque, ";
      fout_[i] << "drive_vel_ref, drive_pos, drive_vel, drive_torque ";
      fout_[i] << "slip_ratio, grf_x, grf_y, grf_z, ";
      fout_[i] << "grt_x, grt_y, grt_z " << std::endl;
    }
  }

  //************************************Trunk Data ********************************************** */
  if (!fout_[4])
  {
    std::cerr << "Cannot open file: " << 4 <<std::endl;
    exit(1);
  }
  else
  {
    fout_[4] << "trunk_x_vel, trunk_y_vel, trunk_z_vel";
    fout_[4] << "trunk_x_ang_vel, trunk_y_ang_vel, trunk_z_ang_vel";
    fout_[4] << "trunk_x_pos, trunk_y_pos, trunk_z_pos, trunk_x_acc, trunk_y_acc, trunk_z_acc" << std::endl;

  }
}

template <typename T>
void DataLogging<T>::save_data()
{
  //*********************  Leg Data  *************************** */
  for (int i = 0; i < 4; i++)
  {
    if (!fout_[i])
    {
      std::cerr << "Cannot open file" << std::endl;
      exit(1);
    }
    else
    {

      fout_[i] << time_ << ","; // time
      fout_[i] << traj_ptr_->traj_motor_pos_[i][0] << ","; // sus pos ref
      fout_[i] << model_ptr_->joint_pos_act_[i][0] << ","; // sus pos
      fout_[i] << model_ptr_->joint_vel_act_[i][0] << ","; // sus vel
      fout_[i] << ctrl_ptr_->sus_torque_[i] << ","; // sus torque //!부호 생각해야함
      fout_[i] << traj_ptr_->traj_motor_pos_[i][1] << ","; // steer pos ref
      fout_[i] << model_ptr_->joint_pos_act_[i][1] << ","; // steer pos
      fout_[i] << model_ptr_->joint_vel_act_[i][1] << ","; // steer vel
      fout_[i] << ctrl_ptr_->steer_torque_[i] << ","; // steer torque
      fout_[i] << traj_ptr_->traj_motor_vel_[i][2] << ","; // drive vel ref
      fout_[i] << model_ptr_->joint_pos_act_[i][2] << ","; // drive pos
      fout_[i] << model_ptr_->joint_vel_act_[i][2] << ","; // drive vel
      fout_[i] << ctrl_ptr_->drive_torque_[i] << ","; // drive torque
      fout_[i] << model_ptr_->slip_ratio_[i] << ","; // slip ratio //! 센서 좌표계나오면 세팅
      fout_[i] << model_ptr_->contact_force_[i][0] << ","; // grf x
      fout_[i] << model_ptr_->contact_force_[i][1] << ","; // grf y
      fout_[i] << model_ptr_->contact_force_[i][2] << ","; // grf z
      fout_[i] << model_ptr_->contact_torque_[i][0] << ","; // grt x
      fout_[i] << model_ptr_->contact_torque_[i][1] << ","; // grt y
      fout_[i] << model_ptr_->contact_torque_[i][2]; // grt z





      // ! Don't remove the newline
      fout_[i] << endl;
    }
  }

  if (!fout_[4])
  {
    std::cerr << "Cannot open file" << std::endl;
    exit(1);
  }
  else
  {
    fout_[4] << model_ptr_->body_vel_[0] << ","; // trunk x velocity
    fout_[4] << model_ptr_->body_vel_[1] << ","; // trunk y velocity
    fout_[4] << model_ptr_->body_vel_[2] << ","; // trunk z velocity
    fout_[4] << model_ptr_->body_ang_vel_[0] << ","; // trunk x ang velocity
    fout_[4] << model_ptr_->body_ang_vel_[1] << ","; // trunk y ang velocity
    fout_[4] << model_ptr_->body_ang_vel_[2] << ","; // trunk z ang velocity
    fout_[4] << model_ptr_->body_pos_[0] << ","; // trunk x pos
    fout_[4] << model_ptr_->body_pos_[1] << ","; // trunk y pos
    fout_[4] << model_ptr_->body_pos_[2] << ","; // trunk z pos
    fout_[4] << model_ptr_->body_acc_[0] << ","; // trunk x acc
    fout_[4] << model_ptr_->body_acc_[1] << ","; // trunk y acc
    fout_[4] << model_ptr_->body_acc_[2] ; // trunk z acc

    // ! Don't remove the newline
    fout_[4] << endl;
  }

}

template <typename T>
void DataLogging<T>::get_time(T time)
{
  time_ = time;
}

template <typename T>
void DataLogging<T>::get_Mclrovoer_ptr(
  std::shared_ptr<typename Mclrover<T>::model> model_ptr,
  std::shared_ptr<typename Mclrover<T>::Traj> traj_ptr,
  std::shared_ptr<typename Mclrover<T>::ctrl> ctrl_ptr
)
{
  model_ptr_ = model_ptr;
  traj_ptr_ = traj_ptr;
  ctrl_ptr_ = ctrl_ptr;
}



template class DataLogging<double>;
template class DataLogging<float>;
