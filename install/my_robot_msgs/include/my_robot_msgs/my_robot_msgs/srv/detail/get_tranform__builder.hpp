// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_robot_msgs:srv/GetTranform.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_MSGS__SRV__DETAIL__GET_TRANFORM__BUILDER_HPP_
#define MY_ROBOT_MSGS__SRV__DETAIL__GET_TRANFORM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_robot_msgs/srv/detail/get_tranform__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_robot_msgs
{

namespace srv
{

namespace builder
{

class Init_GetTranform_Request_child_frame_id
{
public:
  explicit Init_GetTranform_Request_child_frame_id(::my_robot_msgs::srv::GetTranform_Request & msg)
  : msg_(msg)
  {}
  ::my_robot_msgs::srv::GetTranform_Request child_frame_id(::my_robot_msgs::srv::GetTranform_Request::_child_frame_id_type arg)
  {
    msg_.child_frame_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_msgs::srv::GetTranform_Request msg_;
};

class Init_GetTranform_Request_frame_id
{
public:
  Init_GetTranform_Request_frame_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTranform_Request_child_frame_id frame_id(::my_robot_msgs::srv::GetTranform_Request::_frame_id_type arg)
  {
    msg_.frame_id = std::move(arg);
    return Init_GetTranform_Request_child_frame_id(msg_);
  }

private:
  ::my_robot_msgs::srv::GetTranform_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_msgs::srv::GetTranform_Request>()
{
  return my_robot_msgs::srv::builder::Init_GetTranform_Request_frame_id();
}

}  // namespace my_robot_msgs


namespace my_robot_msgs
{

namespace srv
{

namespace builder
{

class Init_GetTranform_Response_success
{
public:
  explicit Init_GetTranform_Response_success(::my_robot_msgs::srv::GetTranform_Response & msg)
  : msg_(msg)
  {}
  ::my_robot_msgs::srv::GetTranform_Response success(::my_robot_msgs::srv::GetTranform_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_msgs::srv::GetTranform_Response msg_;
};

class Init_GetTranform_Response_tranform
{
public:
  Init_GetTranform_Response_tranform()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTranform_Response_success tranform(::my_robot_msgs::srv::GetTranform_Response::_tranform_type arg)
  {
    msg_.tranform = std::move(arg);
    return Init_GetTranform_Response_success(msg_);
  }

private:
  ::my_robot_msgs::srv::GetTranform_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_msgs::srv::GetTranform_Response>()
{
  return my_robot_msgs::srv::builder::Init_GetTranform_Response_tranform();
}

}  // namespace my_robot_msgs

#endif  // MY_ROBOT_MSGS__SRV__DETAIL__GET_TRANFORM__BUILDER_HPP_
