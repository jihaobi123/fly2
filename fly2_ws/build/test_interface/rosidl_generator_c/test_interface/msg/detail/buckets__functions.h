// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from test_interface:msg/Buckets.idl
// generated code does not contain a copyright notice

#ifndef TEST_INTERFACE__MSG__DETAIL__BUCKETS__FUNCTIONS_H_
#define TEST_INTERFACE__MSG__DETAIL__BUCKETS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "test_interface/msg/rosidl_generator_c__visibility_control.h"

#include "test_interface/msg/detail/buckets__struct.h"

/// Initialize msg/Buckets message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * test_interface__msg__Buckets
 * )) before or use
 * test_interface__msg__Buckets__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_test_interface
bool
test_interface__msg__Buckets__init(test_interface__msg__Buckets * msg);

/// Finalize msg/Buckets message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_test_interface
void
test_interface__msg__Buckets__fini(test_interface__msg__Buckets * msg);

/// Create msg/Buckets message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * test_interface__msg__Buckets__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_test_interface
test_interface__msg__Buckets *
test_interface__msg__Buckets__create();

/// Destroy msg/Buckets message.
/**
 * It calls
 * test_interface__msg__Buckets__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_test_interface
void
test_interface__msg__Buckets__destroy(test_interface__msg__Buckets * msg);

/// Check for msg/Buckets message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_test_interface
bool
test_interface__msg__Buckets__are_equal(const test_interface__msg__Buckets * lhs, const test_interface__msg__Buckets * rhs);

/// Copy a msg/Buckets message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_test_interface
bool
test_interface__msg__Buckets__copy(
  const test_interface__msg__Buckets * input,
  test_interface__msg__Buckets * output);

/// Initialize array of msg/Buckets messages.
/**
 * It allocates the memory for the number of elements and calls
 * test_interface__msg__Buckets__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_test_interface
bool
test_interface__msg__Buckets__Sequence__init(test_interface__msg__Buckets__Sequence * array, size_t size);

/// Finalize array of msg/Buckets messages.
/**
 * It calls
 * test_interface__msg__Buckets__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_test_interface
void
test_interface__msg__Buckets__Sequence__fini(test_interface__msg__Buckets__Sequence * array);

/// Create array of msg/Buckets messages.
/**
 * It allocates the memory for the array and calls
 * test_interface__msg__Buckets__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_test_interface
test_interface__msg__Buckets__Sequence *
test_interface__msg__Buckets__Sequence__create(size_t size);

/// Destroy array of msg/Buckets messages.
/**
 * It calls
 * test_interface__msg__Buckets__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_test_interface
void
test_interface__msg__Buckets__Sequence__destroy(test_interface__msg__Buckets__Sequence * array);

/// Check for msg/Buckets message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_test_interface
bool
test_interface__msg__Buckets__Sequence__are_equal(const test_interface__msg__Buckets__Sequence * lhs, const test_interface__msg__Buckets__Sequence * rhs);

/// Copy an array of msg/Buckets messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_test_interface
bool
test_interface__msg__Buckets__Sequence__copy(
  const test_interface__msg__Buckets__Sequence * input,
  test_interface__msg__Buckets__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TEST_INTERFACE__MSG__DETAIL__BUCKETS__FUNCTIONS_H_
