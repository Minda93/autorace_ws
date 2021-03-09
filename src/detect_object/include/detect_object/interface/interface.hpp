#ifndef DETECT_OBJECT_INTERFACE_HPP_
#define DETECT_OBJECT_INTERFACE_HPP_

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

namespace detect_object
{
  namespace interface
  {
    enum RangeType
    {
      INTEGER = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
      DOUBLE = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
      INTEGER_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY,
      DOUBLE_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY,
    };

    template <class T>
    rcl_interfaces::msg::ParameterDescriptor set_num_range(
      const std::string &name,
      RangeType type,
      const T &from_value,
      const T &to_value,
      const T &step
    );

    template <class T>
    void set_range_(
      rcl_interfaces::msg::ParameterDescriptor &descriptor,
      RangeType type,
      const T &from_value,
      const T &to_value,
      const T &step
    );

  } // namespace interface
} // namespace detect_object

#endif // DETECT_OBJECT_INTERFACE_HPP_