// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from test_interface:msg/Buckets.idl
// generated code does not contain a copyright notice

#ifndef TEST_INTERFACE__MSG__DETAIL__BUCKETS__TRAITS_HPP_
#define TEST_INTERFACE__MSG__DETAIL__BUCKETS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "test_interface/msg/detail/buckets__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'buckets'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace test_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const Buckets & msg,
  std::ostream & out)
{
  out << "{";
  // member: buckets
  {
    if (msg.buckets.size() == 0) {
      out << "buckets: []";
    } else {
      out << "buckets: [";
      size_t pending_items = msg.buckets.size();
      for (auto item : msg.buckets) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Buckets & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: buckets
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.buckets.size() == 0) {
      out << "buckets: []\n";
    } else {
      out << "buckets:\n";
      for (auto item : msg.buckets) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Buckets & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace test_interface

namespace rosidl_generator_traits
{

[[deprecated("use test_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const test_interface::msg::Buckets & msg,
  std::ostream & out, size_t indentation = 0)
{
  test_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use test_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const test_interface::msg::Buckets & msg)
{
  return test_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<test_interface::msg::Buckets>()
{
  return "test_interface::msg::Buckets";
}

template<>
inline const char * name<test_interface::msg::Buckets>()
{
  return "test_interface/msg/Buckets";
}

template<>
struct has_fixed_size<test_interface::msg::Buckets>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<test_interface::msg::Buckets>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<test_interface::msg::Buckets>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TEST_INTERFACE__MSG__DETAIL__BUCKETS__TRAITS_HPP_
