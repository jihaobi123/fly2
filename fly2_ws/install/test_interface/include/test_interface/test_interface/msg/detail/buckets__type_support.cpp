// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from test_interface:msg/Buckets.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "test_interface/msg/detail/buckets__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace test_interface
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Buckets_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) test_interface::msg::Buckets(_init);
}

void Buckets_fini_function(void * message_memory)
{
  auto typed_message = static_cast<test_interface::msg::Buckets *>(message_memory);
  typed_message->~Buckets();
}

size_t size_function__Buckets__buckets(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Buckets__buckets(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__Buckets__buckets(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__Buckets__buckets(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::Point *>(
    get_const_function__Buckets__buckets(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__Buckets__buckets(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::Point *>(
    get_function__Buckets__buckets(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__Buckets__buckets(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Buckets_message_member_array[1] = {
  {
    "buckets",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(test_interface::msg::Buckets, buckets),  // bytes offset in struct
    nullptr,  // default value
    size_function__Buckets__buckets,  // size() function pointer
    get_const_function__Buckets__buckets,  // get_const(index) function pointer
    get_function__Buckets__buckets,  // get(index) function pointer
    fetch_function__Buckets__buckets,  // fetch(index, &value) function pointer
    assign_function__Buckets__buckets,  // assign(index, value) function pointer
    resize_function__Buckets__buckets  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Buckets_message_members = {
  "test_interface::msg",  // message namespace
  "Buckets",  // message name
  1,  // number of fields
  sizeof(test_interface::msg::Buckets),
  Buckets_message_member_array,  // message members
  Buckets_init_function,  // function to initialize message memory (memory has to be allocated)
  Buckets_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Buckets_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Buckets_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace test_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<test_interface::msg::Buckets>()
{
  return &::test_interface::msg::rosidl_typesupport_introspection_cpp::Buckets_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, test_interface, msg, Buckets)() {
  return &::test_interface::msg::rosidl_typesupport_introspection_cpp::Buckets_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
