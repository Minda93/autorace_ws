<!-- TOC -->

- [1. detect_object](#1-detect_object)
- [2. detect_lane](#2-detect_lane)
- [TODO](#todo)
- [Issue](#issue)
- [Reference](#reference)

<!-- /TOC -->

# 1. detect_object
* plugins
  * detect_lane

# 2. detect_lane
  * method
    * Perspective Transform
    * object segmentation
      * color segmentation 
      * cal_line_reliability
        * the none_zero pixel of mask / excepted_value  
    * Lane-line pixel identification
      * sliding window
      * line fitting
    * make_lane
      * normalize centerX
        * method_1
          ```
            normCenterX = centerX - (dst_cols / 2)
          ```

  * topic
    * input
      * camera/image_raw (sensor_msgs/Image)
    * output
      * lane_center_x (std_msgs/float64)

  * parameter
    * Perspective Transform
      ```yaml
        birdView:
          # src size(640, 480)
          bottom_x: 350
          bottom_y: 240
          top_x: 90
          top_y: -60
          
          # maybe add
          # dst size (250, 300)
          # points : [50, 0], [200, 0], [200, 300], [50, 300]
      ```
      
    * object segmentation
      * param
      ```yaml
        mask_lane:
          # cal none zero for mask 
          # reliability = noneZero / excepted_value
          # straight lane : the average value of none zero pixel is 2500 
          expected_value: 2500.0
        hsv_model:
          white:
            hue_h: 41
            hue_l: 0
            saturation_h: 18
            saturation_l: 0
            value_h: 255
            value_l: 98
          yellow:
            hue_h: 37
            hue_l: 26
            saturation_h: 255
            saturation_l: 212
            value_h: 255
            value_l: 119
      ```  
      
    * Lane-line pixel identification
      * param
      ```yaml
        # maybe add
        # sliding_window:
        #   nWindows: 20
        #   # Set the width of the windows +/- margin
        #   margin: 50 
        #   # Set minimum number of pixels found to recenter window
        #   minpix: 50
        #   
        # line_fitting:
        #   # Set the width of the windows +/- margin
        #   margin: 100
      ```  
        
    * make_lane
      * param
      ```yaml
        # maybe add
        # make_lane:
        #   # get centerX for datum
        #   # range [0, dst_rows-1]
        #   datumY: 150
        #   # centerX - (dst_cols / 2)
        #   normalize: 124
        #   line_reliability: 0.15
      ```  
      
# TODO  
* detect_lane
  - [x] param slider bar(~~int~~, ~~double~~, int_array, double_array)
    - [x] ~~for detect_lane hsv bar (yellow, white)~~
  - [x] ~~Perspective Transform~~
  - [x] object segmentation
    - [x] ~~color segmentation~~
  - [x] Lane-line pixel identification
    - [x] ~~sliding window algorithm~~
    - [x] ~~lane_fitting~~
    - [x] condition optimization
  - [x] make_lane
    - [x] normalize centerX
    - [x] condition change 
  - [ ] create find_lane object
    - [ ] Lane-line pixel identification
    - [ ] make_lane
    - [ ] maybe use thread
# Issue  
* rqt_gui dynamic_reconfigure has no implementation of an array
  * [dynamic_reconfigure issue 67](https://github.com/ros/dynamic_reconfigure/issues/67)
  
* rqt_reconfigure is NOT supported group (for ros2)
  * [rqt_reconfigure issues 69](https://github.com/ros-visualization/rqt_reconfigure/issues/69)

* ros2 get group parameters method
  * now use method
    ```cpp
      std::vector<std::string> param_names = {
        "hsv_model.yellow.hue_l",
        "hsv_model.yellow.saturation_l", ...
      }
      std::vector<rclcpp::Parameter> params = get_parameters(param_names);
    ```
  * CANNOT use group to get below parameters
    ```cpp
      std::vector<int> params = get_parameters("hsv_model");
    ```

# Reference  
* [画像の幾何変換](http://labs.eecs.tottori-u.ac.jp/sd/Member/oyamada/OpenCV/html/py_tutorials/py_imgproc/py_geometric_transformations/py_geometric_transformations.html)
* [polyfit function c++](https://www.programmersought.com/article/26353620262/)
* [Advanced Lane Finding](https://medium.com/typeiqs/advanced-lane-finding-c3c8305f074)