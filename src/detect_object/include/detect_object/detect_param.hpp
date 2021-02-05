#ifndef DETECT_OBJECT_DETECT_PARAM_HPP_
#define DETECT_OBJECT_DETECT_PARAM_HPP_

#include <vector>

namespace detect_object
{
  enum BaseMode{
    INIT,
    INACTIVE,
    START_UP,
    FINISH
  };

  struct PerspectiveParam
  {
    float top_x;
    float top_y;
    float bottom_x;
    float bottom_y;
  };

  struct DetectLaneParam
  {
    bool showImage;
    PerspectiveParam birdView;

    // color model []
    std::vector<int> yellowHSV;
    std::vector<int> whiteHSV;
  }; // struct DetectLaneParam
} // detect_object

#endif // DETECT_OBJECT_DETECT_PARAM_HPP_