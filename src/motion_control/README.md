<!-- TOC -->

- [motion_control](#motion_control)
- [1. motion_control plugins](#1-motion_control-plugins)
- [2. lane_control](#2-lane_control)
- [TODO](#todo)
- [Reference](#reference)

<!-- /TOC -->

# motion_control

# 1. motion_control plugins  
* lane_control
* construction_control

# 2. lane_control  
* method
  * PID
* topic 
  * input
    * lane_center_x (std_msgs/float64)
  * output
    * /cmd_vel (geometry_msgs/msg/Twist)
* parameter
  * PID
  ```yaml
    pid:
      kd: 0.1
      ki: 0.0
      kp: 0.28
    robot:
      # robot angular max vel
      maxAngularVel: 2.0

      # robot linear max vel
      maxLinearVel: 0.15
    
    # reset pid and stop robot (switch)
    test_pid_param: false
  ```

# TODO  
* lane_control
  - [ ] center_x cal adjust at the detect_lane plugin

*  

# Reference  
