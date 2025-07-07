// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from test_interface:msg/Buckets.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "test_interface/msg/detail/buckets__rosidl_typesupport_introspection_c.h"
#include "test_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "test_interface/msg/detail/buckets__functions.h"
#include "test_interface/msg/detail/buckets__struct.h"


// Include directives for member types
// Member `buckets`
#include "geometry_msgs/msg/point.h"
// Member `buckets`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void test_interface__msg__Buckets__rosidl_typesupport_introspection_c__Buckets_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  test_interface__msg__Buckets__init(message_memory);
}

void test_interface__msg__Buckets__rosidl_typesupport_introspection_c__Buckets_fini_function(void * message_memory)
{
  test_interface__msg__Buckets__fini(message_memory);
}

size_t test_interface__msg__Buckets__rosidl_typesupport_introspection_c__size_function__Buckets__buckets(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * test_interface__msg__Buckets__rosidl_typesupport_introspection_c__get_const_function__Buckets__buckets(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * test_interface__msg__Buckets__rosidl_typesupport_introspection_c__get_function__Buckets__buckets(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void test_interface__msg__Buckets__rosidl_typesupport_introspection_c__fetch_function__Buckets__buckets(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    test_interface__msg__Buckets__rosidl_typesupport_introspection_c__get_const_function__Buckets__buckets(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void test_interface__msg__Buckets__rosidl_typesupport_introspection_c__assign_function__Buckets__buckets(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    test_interface__msg__Buckets__rosidl_typesupport_introspection_c__get_function__Buckets__buckets(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool test_interface__msg__Buckets__rosidl_typesupport_introspection_c__resize_function__Buckets__buckets(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember test_interface__msg__Buckets__rosidl_typesupport_introspection_c__Buckets_message_member_array[1] = {
  {
    "buckets",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(test_interface__msg__Buckets, buckets),  // bytes offset in struct
    NULL,  // default value
    test_interface__msg__Buckets__rosidl_typesupport_introspection_c__size_function__Buckets__buckets,  // size() function pointer
    test_interface__msg__Buckets__rosidl_typesupport_introspection_c__get_const_function__Buckets__buckets,  // get_const(index) function pointer
    test_interface__msg__Buckets__rosidl_typesupport_introspection_c__get_function__Buckets__buckets,  // get(index) function pointer
    test_interface__msg__Buckets__rosidl_typesupport_introspection_c__fetch_function__Buckets__buckets,  // fetch(index, &value) function pointer
    test_interface__msg__Buckets__rosidl_typesupport_introspection_c__assign_function__Buckets__buckets,  // assign(index, value) function pointer
    test_interface__msg__Buckets__rosidl_typesupport_introspection_c__resize_function__Buckets__buckets  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers test_interface__msg__Buckets__rosidl_typesupport_introspection_c__Buckets_message_members = {
  "test_interface__msg",  // message namespace
  "Buckets",  // message name
  1,  // number of fields
  sizeof(test_interface__msg__Buckets),
  test_interface__msg__Buckets__rosidl_typesupport_introspection_c__Buckets_message_member_array,  // message members
  test_interface__msg__Buckets__rosidl_typesupport_introspection_c__Buckets_init_function,  // function to initialize message memory (memory has to be allocated)
  test_interface__msg__Buckets__rosidl_typesupport_introspection_c__Buckets_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t test_interface__msg__Buckets__rosidl_typesupport_introspection_c__Buckets_message_type_support_handle = {
  0,
  &test_interface__msg__Buckets__rosidl_typesupport_introspection_c__Buckets_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_test_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, test_interface, msg, Buckets)() {
  test_interface__msg__Buckets__rosidl_typesupport_introspection_c__Buckets_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!test_interface__msg__Buckets__rosidl_typesupport_introspection_c__Buckets_message_type_support_handle.typesupport_identifier) {
    test_interface__msg__Buckets__rosidl_typesupport_introspection_c__Buckets_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &test_interface__msg__Buckets__rosidl_typesupport_introspection_c__Buckets_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
