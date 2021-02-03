<!-- TOC -->

- [1. detect_object](#1-detect_object)
- [2. detect_lane](#2-detect_lane)
- [TODO](#todo)
- [Reference](#reference)

<!-- /TOC -->

# 1. detect_object
* plugins
  * detect_lane

# 2. detect_lane
  * method
    * Perspective Transform
      * param 
      ```yaml
        # Perspective Transform param
        bottom_x: 330.0
        bottom_y: 240.0
        top_x: 70.0
        top_y: -40.0
        
        # maybe add
        # dst size (250, 300)
        # points : [50, 0], [200, 0], [200, 300], [50, 300]
      ```
      
    * Lane-line pixel identification
    
# TODO  
* detect_lane
  - [x] Perspective Transform
  - [x] Lane-line pixel identification

# Reference  
* [画像の幾何変換](http://labs.eecs.tottori-u.ac.jp/sd/Member/oyamada/OpenCV/html/py_tutorials/py_imgproc/py_geometric_transformations/py_geometric_transformations.html)