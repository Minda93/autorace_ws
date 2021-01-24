<!-- TOC -->

- [1. cam_tools](#1-cam_tools)
- [2. cam_calibration](#2-cam_calibration)
- [3. Get chess board's images](#3-get-chess-boards-images)
- [TODO](#todo)

<!-- /TOC -->

# 1. cam_tools  
* plugins
  * cam_calibration

# 2. cam_calibration
  * use images
    * load image use ros parameter `images_path` load `.config` file
      * `images_path:="<pkg_of_cam_tools>/config/<TURTLEBOT3_MODEL>.config"`
    * folder format
      * <TURTLEBOT3_MODEL> folder : storage the images of the chess board.
      * <TURTLEBOT3_MODEL>.config : name of images in the <TURTLEBOT3_MODEL> folder
      
      ```txt
        <pkg_of_cam_tools>/
          config/
            <TURTLEBOT3_MODEL>/
            <TURTLEBOT3_MODEL>.config
      ```        
    * config file format  
      ```text
      image1.png
      image2.png
      ```

# 3. Get chess board's images
  * use `gazebo_gui` adjust board's Pose
  * use `showimage` node in the `image_tools` package save images  
  
```
  $ ros2 launch turtlebot3_gazebo cam_calibration.launch.py
  $ ros2 run image_tools showimage --ros-args -p reliability:=best_effort -r image:=/camera/image_raw
```
      
# TODO  
* cam_calibration
  - [x] ~~get path of the images file~~
  - [x] process