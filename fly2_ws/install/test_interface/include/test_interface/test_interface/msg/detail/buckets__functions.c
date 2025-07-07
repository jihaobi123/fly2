// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from test_interface:msg/Buckets.idl
// generated code does not contain a copyright notice
#include "test_interface/msg/detail/buckets__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `buckets`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
test_interface__msg__Buckets__init(test_interface__msg__Buckets * msg)
{
  if (!msg) {
    return false;
  }
  // buckets
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->buckets, 0)) {
    test_interface__msg__Buckets__fini(msg);
    return false;
  }
  return true;
}

void
test_interface__msg__Buckets__fini(test_interface__msg__Buckets * msg)
{
  if (!msg) {
    return;
  }
  // buckets
  geometry_msgs__msg__Point__Sequence__fini(&msg->buckets);
}

bool
test_interface__msg__Buckets__are_equal(const test_interface__msg__Buckets * lhs, const test_interface__msg__Buckets * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // buckets
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->buckets), &(rhs->buckets)))
  {
    return false;
  }
  return true;
}

bool
test_interface__msg__Buckets__copy(
  const test_interface__msg__Buckets * input,
  test_interface__msg__Buckets * output)
{
  if (!input || !output) {
    return false;
  }
  // buckets
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->buckets), &(output->buckets)))
  {
    return false;
  }
  return true;
}

test_interface__msg__Buckets *
test_interface__msg__Buckets__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  test_interface__msg__Buckets * msg = (test_interface__msg__Buckets *)allocator.allocate(sizeof(test_interface__msg__Buckets), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(test_interface__msg__Buckets));
  bool success = test_interface__msg__Buckets__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
test_interface__msg__Buckets__destroy(test_interface__msg__Buckets * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    test_interface__msg__Buckets__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
test_interface__msg__Buckets__Sequence__init(test_interface__msg__Buckets__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  test_interface__msg__Buckets * data = NULL;

  if (size) {
    data = (test_interface__msg__Buckets *)allocator.zero_allocate(size, sizeof(test_interface__msg__Buckets), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = test_interface__msg__Buckets__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        test_interface__msg__Buckets__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
test_interface__msg__Buckets__Sequence__fini(test_interface__msg__Buckets__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      test_interface__msg__Buckets__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

test_interface__msg__Buckets__Sequence *
test_interface__msg__Buckets__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  test_interface__msg__Buckets__Sequence * array = (test_interface__msg__Buckets__Sequence *)allocator.allocate(sizeof(test_interface__msg__Buckets__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = test_interface__msg__Buckets__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
test_interface__msg__Buckets__Sequence__destroy(test_interface__msg__Buckets__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    test_interface__msg__Buckets__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
test_interface__msg__Buckets__Sequence__are_equal(const test_interface__msg__Buckets__Sequence * lhs, const test_interface__msg__Buckets__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!test_interface__msg__Buckets__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
test_interface__msg__Buckets__Sequence__copy(
  const test_interface__msg__Buckets__Sequence * input,
  test_interface__msg__Buckets__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(test_interface__msg__Buckets);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    test_interface__msg__Buckets * data =
      (test_interface__msg__Buckets *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!test_interface__msg__Buckets__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          test_interface__msg__Buckets__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!test_interface__msg__Buckets__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
