<!-- TOC -->

- [autorace_ws](#autorace_ws)
- [1. Environment](#1-environment)
- [2. Setup env](#2-setup-env)
- [TODO](#todo)
- [Bug](#bug)
- [Reference](#reference)

<!-- /TOC -->

# autorace_ws
ros2 practice 2 

# 1. Environment

| names                               | version           |
| ---                                 | ---               |
| gazebo                              | 11.3.0            |
| CMake                               | 3.18.4            |
| ros2                                | foxy-release_4    |
| gazebo_ros_pkgs                     | ros2              |
| turtlebot3                          | foxy-devel        |
| turtlebot3_msgs                     | foxy-devel        |
| turtlebot3_simulation               | foxy-devel        |
| turtlebot3_simulation simple model  | foxy-devel        |
| turtlebot3_simulation complex model | 58b752a6ac commit |
| turtlebot3_simulation autorace_2020 | master            |

# 2. Setup env

* download turtlebot3_custom   
```bash
  $ cd <autorace_ws>/src
  $ ./turtlebot3_custom.sh
```

# TODO 
* env_control
  - [ ] traffic light
  - [ ] traffic bar
* cam_tools (cam_tools branch)
  - [ ] calibration (bug)
* detect_object
  - [x] detect_lane (detect lane)
  - [ ] detect_traffic_sign
* strategy
  - [ ] decider
* control
  - [x] control_lane

# Bug  
  * gazebo_ros_pkgs(ros2 branch) compile error (source foxy-release_4)
    * error
      * gtest can NOT find `INSTANTIATE_TEST_SUITE_P`
    * solution
      * `INSTANTIATE_TEST_CASE_P`
    * change file
      * gazebo_ros/test/test_plugins.cpp
      * gazebo_plugins/test/test_gazebo_ros_camera.cpp 
      * gazebo_plugins/test/test_gazebo_ros_camera_distortion.cpp
      * gazebo_plugins/test/test_gazebo_ros_depth_camera.cpp 
      * gazebo_plugins/test/test_gazebo_ros_diff_drive.cpp
      * gazebo_plugins/test/test_gazebo_ros_multi_camera.cpp
      * gazebo_plugins/test/test_gazebo_ros_multicamera.cpp

# Reference  
* [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2)
* [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
* [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
* [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
* [turtlebot3_simulations/tree/58b752a6ac](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/58b752a6ac26ae2e3be58b7b0d61af044d570c1f/turtlebot3_gazebo/models)
* QA
  * turtlebot3_simulation
    * [no rending on gazebo](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/139) 
  * gazebo_ros_pkgs
    * [Fixed Parameterized testing](https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1184/files/f9e9c4c129ad49c55b049d48e1de7a522eab4c8a)
      * change origin source code