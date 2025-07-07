// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from test_interface:msg/Buckets.idl
// generated code does not contain a copyright notice

#ifndef TEST_INTERFACE__MSG__DETAIL__BUCKETS__BUILDER_HPP_
#define TEST_INTERFACE__MSG__DETAIL__BUCKETS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "test_interface/msg/detail/buckets__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace test_interface
{

namespace msg
{

namespace builder
{

class Init_Buckets_buckets
{
public:
  Init_Buckets_buckets()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::test_interface::msg::Buckets buckets(::test_interface::msg::Buckets::_buckets_type arg)
  {
    msg_.buckets = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_interface::msg::Buckets msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_interface::msg::Buckets>()
{
  return test_interface::msg::builder::Init_Buckets_buckets();
}

}  // namespace test_interface

#endif  // TEST_INTERFACE__MSG__DETAIL__BUCKETS__BUILDER_HPP_
