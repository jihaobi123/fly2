// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from test_interface:msg/Buckets.idl
// generated code does not contain a copyright notice

#ifndef TEST_INTERFACE__MSG__DETAIL__BUCKETS__STRUCT_HPP_
#define TEST_INTERFACE__MSG__DETAIL__BUCKETS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'buckets'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__test_interface__msg__Buckets __attribute__((deprecated))
#else
# define DEPRECATED__test_interface__msg__Buckets __declspec(deprecated)
#endif

namespace test_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Buckets_
{
  using Type = Buckets_<ContainerAllocator>;

  explicit Buckets_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Buckets_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _buckets_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _buckets_type buckets;

  // setters for named parameter idiom
  Type & set__buckets(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->buckets = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    test_interface::msg::Buckets_<ContainerAllocator> *;
  using ConstRawPtr =
    const test_interface::msg::Buckets_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<test_interface::msg::Buckets_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<test_interface::msg::Buckets_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      test_interface::msg::Buckets_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<test_interface::msg::Buckets_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      test_interface::msg::Buckets_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<test_interface::msg::Buckets_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<test_interface::msg::Buckets_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<test_interface::msg::Buckets_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__test_interface__msg__Buckets
    std::shared_ptr<test_interface::msg::Buckets_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__test_interface__msg__Buckets
    std::shared_ptr<test_interface::msg::Buckets_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Buckets_ & other) const
  {
    if (this->buckets != other.buckets) {
      return false;
    }
    return true;
  }
  bool operator!=(const Buckets_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Buckets_

// alias to use template instance with default allocator
using Buckets =
  test_interface::msg::Buckets_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace test_interface

#endif  // TEST_INTERFACE__MSG__DETAIL__BUCKETS__STRUCT_HPP_
