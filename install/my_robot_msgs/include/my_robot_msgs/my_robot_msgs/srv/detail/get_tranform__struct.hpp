// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_robot_msgs:srv/GetTranform.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_MSGS__SRV__DETAIL__GET_TRANFORM__STRUCT_HPP_
#define MY_ROBOT_MSGS__SRV__DETAIL__GET_TRANFORM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__my_robot_msgs__srv__GetTranform_Request __attribute__((deprecated))
#else
# define DEPRECATED__my_robot_msgs__srv__GetTranform_Request __declspec(deprecated)
#endif

namespace my_robot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTranform_Request_
{
  using Type = GetTranform_Request_<ContainerAllocator>;

  explicit GetTranform_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id = "";
      this->child_frame_id = "";
    }
  }

  explicit GetTranform_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : frame_id(_alloc),
    child_frame_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id = "";
      this->child_frame_id = "";
    }
  }

  // field types and members
  using _frame_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _frame_id_type frame_id;
  using _child_frame_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _child_frame_id_type child_frame_id;

  // setters for named parameter idiom
  Type & set__frame_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->frame_id = _arg;
    return *this;
  }
  Type & set__child_frame_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->child_frame_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_robot_msgs::srv::GetTranform_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_robot_msgs::srv::GetTranform_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_robot_msgs::srv::GetTranform_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_robot_msgs::srv::GetTranform_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_robot_msgs::srv::GetTranform_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_robot_msgs::srv::GetTranform_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_robot_msgs::srv::GetTranform_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_robot_msgs::srv::GetTranform_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_robot_msgs::srv::GetTranform_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_robot_msgs::srv::GetTranform_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_robot_msgs__srv__GetTranform_Request
    std::shared_ptr<my_robot_msgs::srv::GetTranform_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_robot_msgs__srv__GetTranform_Request
    std::shared_ptr<my_robot_msgs::srv::GetTranform_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTranform_Request_ & other) const
  {
    if (this->frame_id != other.frame_id) {
      return false;
    }
    if (this->child_frame_id != other.child_frame_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTranform_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTranform_Request_

// alias to use template instance with default allocator
using GetTranform_Request =
  my_robot_msgs::srv::GetTranform_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace my_robot_msgs


// Include directives for member types
// Member 'tranform'
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__my_robot_msgs__srv__GetTranform_Response __attribute__((deprecated))
#else
# define DEPRECATED__my_robot_msgs__srv__GetTranform_Response __declspec(deprecated)
#endif

namespace my_robot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTranform_Response_
{
  using Type = GetTranform_Response_<ContainerAllocator>;

  explicit GetTranform_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : tranform(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit GetTranform_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : tranform(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _tranform_type =
    geometry_msgs::msg::TransformStamped_<ContainerAllocator>;
  _tranform_type tranform;
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__tranform(
    const geometry_msgs::msg::TransformStamped_<ContainerAllocator> & _arg)
  {
    this->tranform = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_robot_msgs::srv::GetTranform_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_robot_msgs::srv::GetTranform_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_robot_msgs::srv::GetTranform_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_robot_msgs::srv::GetTranform_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_robot_msgs::srv::GetTranform_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_robot_msgs::srv::GetTranform_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_robot_msgs::srv::GetTranform_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_robot_msgs::srv::GetTranform_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_robot_msgs::srv::GetTranform_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_robot_msgs::srv::GetTranform_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_robot_msgs__srv__GetTranform_Response
    std::shared_ptr<my_robot_msgs::srv::GetTranform_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_robot_msgs__srv__GetTranform_Response
    std::shared_ptr<my_robot_msgs::srv::GetTranform_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTranform_Response_ & other) const
  {
    if (this->tranform != other.tranform) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTranform_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTranform_Response_

// alias to use template instance with default allocator
using GetTranform_Response =
  my_robot_msgs::srv::GetTranform_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace my_robot_msgs

namespace my_robot_msgs
{

namespace srv
{

struct GetTranform
{
  using Request = my_robot_msgs::srv::GetTranform_Request;
  using Response = my_robot_msgs::srv::GetTranform_Response;
};

}  // namespace srv

}  // namespace my_robot_msgs

#endif  // MY_ROBOT_MSGS__SRV__DETAIL__GET_TRANFORM__STRUCT_HPP_
