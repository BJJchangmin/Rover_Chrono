
#ifndef DATA_LOGGING_HPP_
#define DATA_LOGGING_HPP_

// C++ standard libraries
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rover.hpp"

template <typename T>
class DataLogging
{
  private:
    std::ofstream fout_[5];
    T time_;

    std::shared_ptr<typename Mclrover<T>::model> model_ptr_;
    std::shared_ptr<typename Mclrover<T>::Traj> traj_ptr_;
    std::shared_ptr<typename Mclrover<T>::ctrl> ctrl_ptr_;

  public:
    explicit DataLogging();

    void init_data();
    void save_data();
    void get_time(T time);

    void get_Mclrovoer_ptr(
      std::shared_ptr<typename Mclrover<T>::model> model_ptr,
      std::shared_ptr<typename Mclrover<T>::Traj> traj_ptr,
      std::shared_ptr<typename Mclrover<T>::ctrl> ctrl_ptr
    );

};



#endif  // data_logging_HPP_
