#ifndef MOTION_CONTROl_PARAM_HPP_
#define MOTION_CONTROl_PARAM_HPP_

namespace motion_control
{
  struct RobotVel
  {
    double linearX;
    double angularZ;
  }; // struct RobotVel

  struct PIDParam
  {
    double kp;
    double ki;
    double kd;
  }; // struct PIDParam

  class LaneCtrlParam
  {
  public:
    double maxLinearVel;
    double maxAngularVel;
    double ctrlTarget;
    PIDParam pid;
  }; // LaneCtrlParam

} // namespace motion_control

#endif // MOTION_CONTROl_PARAM_HPP_