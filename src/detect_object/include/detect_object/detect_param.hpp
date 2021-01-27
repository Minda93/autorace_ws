#ifndef DETECT_OBJECT_DETECT_PARAM_HPP_
#define DETECT_OBJECT_DETECT_PARAM_HPP_

namespace detect_object
{
  enum BaseMode{
    INIT,
    INACTIVE,
    START_UP,
    FINISH
  };

  struct DetectLaneParam
  {
    bool showImage;
  }; // struct DetectLaneParam
} // detect_object

#endif // DETECT_OBJECT_DETECT_PARAM_HPP_