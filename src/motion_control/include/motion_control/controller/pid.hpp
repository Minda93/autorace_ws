#ifndef MOTION_CONTROL_PID_HPP_
#define MOTION_CONTROL_PID_HPP_

namespace motion_control{
  namespace controller{
    class PID{

    public:
      PID(double target, double kp, double ki, double kd);
      ~PID() = default;

      double run(double state, long double delta);
      void reset();
      void set_target(double target);
      
      void set_kp(double kp);
      void set_ki(double ki);
      void set_kd(double kd);

    private:
      double target_;
      double kp_;
      double ki_;
      double kd_;

      double prevError_{0.0};
      double integral_{0.0};
    }; // class PID
  } // namespace controller
} // namespace motion_control

#endif // MOTION_CONTROL_PID_HPP_
