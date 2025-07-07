// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from test_interface:msg/Buckets.idl
// generated code does not contain a copyright notice

#ifndef TEST_INTERFACE__MSG__DETAIL__BUCKETS__STRUCT_H_
#define TEST_INTERFACE__MSG__DETAIL__BUCKETS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'buckets'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/Buckets in the package test_interface.
typedef struct test_interface__msg__Buckets
{
  geometry_msgs__msg__Point__Sequence buckets;
} test_interface__msg__Buckets;

// Struct for a sequence of test_interface__msg__Buckets.
typedef struct test_interface__msg__Buckets__Sequence
{
  test_interface__msg__Buckets * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} test_interface__msg__Buckets__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TEST_INTERFACE__MSG__DETAIL__BUCKETS__STRUCT_H_
