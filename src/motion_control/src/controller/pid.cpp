#include "motion_control/controller/pid.hpp"

#include <iostream>

namespace motion_control{
  namespace controller{
    PID::PID(double target, double kp, double ki, double kd)
      : target_{target}, kp_{kp}, ki_{ki}, kd_{kd}
    {
      reset();
      
      // display param
      // std::cout<< target_ << " " << kp_ << " "
      //          << ki_ << " " << kd_ << "\n";
      // std::flush(std::cout);
    }

    void PID::reset()
    {
      prevError_ = 0.0;
      integral_ = 0.0; 
    }

    void PID::set_target(double target)
    {
      target_ = target;
    }

    void PID::set_kp(double kp)
    {
      kp_ = kp;
    }

    void PID::set_ki(double ki)
    {
      ki_ = ki;
    }

    void PID::set_kd(double kd)
    {
      kd_ = kd;
    }

    double PID::run(double state, long double delta)
    {
      double error{target_ - state};
      double derivative;
      double pid{0.0};
      
      if(delta > 0.0){
        integral_ += error;
        derivative = (error - prevError_)/delta;
        prevError_ = error*delta;

        pid = kp_*error + ki_*integral_ + kd_*derivative;
      }

      // std::cout<< kp_ << " " << ki_ << " " 
      //         << kd_ << "\n" ;
      // std::flush(std::cout);
  
      return pid;
    }
  } // controller
}  // namespace motion_control