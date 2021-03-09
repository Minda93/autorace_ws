#include "detect_object/interface/interface.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"

namespace detect_object
{
  namespace interface
  {
    // rqt dynamic parameter silder
    template <class T>
    rcl_interfaces::msg::ParameterDescriptor set_num_range(
      const std::string &name,
      RangeType type,
      const T &from_value,
      const T &to_value,
      const T &step)
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.name = name;
      descriptor.type = type;
      set_range_(descriptor, type, from_value, to_value, step);
      return descriptor;
    }

    template <class T>
    void set_range_(
      rcl_interfaces::msg::ParameterDescriptor &descriptor,
      RangeType type,
      const T &from_value,
      const T &to_value,
      const T &step)
    {
      switch (type)
      {
      case INTEGER:
        descriptor.integer_range.push_back(rcl_interfaces::msg::IntegerRange{});
        descriptor.integer_range[0].from_value = from_value;
        descriptor.integer_range[0].to_value = to_value;
        descriptor.integer_range[0].step = step;
        return;
      case DOUBLE:
        descriptor.floating_point_range.push_back(rcl_interfaces::msg::FloatingPointRange{});
        descriptor.floating_point_range[0].from_value = from_value;
        descriptor.floating_point_range[0].to_value = to_value;
        descriptor.floating_point_range[0].step = step;
        return;
      default:
        return;
      }
    }
  } // namespace interface
} // namespace detect_object