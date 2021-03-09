#include "detect_object/interface/interface.hpp"
#include "interface.cpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

// int declare
template rcl_interfaces::msg::ParameterDescriptor detect_object::interface::set_num_range<int>(
  const std::string &name,
  RangeType type,
  const int &from_value,
  const int &to_value,
  const int &step);

template void detect_object::interface::set_range_<int>(
  rcl_interfaces::msg::ParameterDescriptor &descriptor,
  RangeType type,
  const int &from_value,
  const int &to_value,
  const int &step);

// double declare
template rcl_interfaces::msg::ParameterDescriptor detect_object::interface::set_num_range<double>(
  const std::string &name,
  RangeType type,
  const double &from_value,
  const double &to_value,
  const double &step);

template void detect_object::interface::set_range_<double>(
  rcl_interfaces::msg::ParameterDescriptor &descriptor,
  RangeType type,
  const double &from_value,
  const double &to_value,
  const double &step);

