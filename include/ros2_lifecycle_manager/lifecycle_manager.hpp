// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_lifecycle_manager_msgs/srv/manage_lifecycle_nodes.hpp"
#include "ros2_utils/lifecycle_service_client.hpp"
#include "ros2_utils/node_thread.hpp"

namespace ros2_lifecycle_manager
{

using ros2_lifecycle_manager_msgs::srv::ManageLifecycleNodes;

// The LifecycleManager provides services to transition the states of managed nodes
class LifecycleManager
{
public:
  LifecycleManager() = delete;

  explicit LifecycleManager(rclcpp::Node::SharedPtr node);
  explicit LifecycleManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  LifecycleManager(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services);

  bool startup();
  bool shutdown();
  bool reset();
  bool pause();
  bool resume();

protected:
  void manager_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManageLifecycleNodes::Request> request,
    std::shared_ptr<ManageLifecycleNodes::Response> response);

  void create_lifecycle_service_clients();
  void destroy_lifecycle_service_clients();

  bool change_state_for_node(const std::string & node_name, std::uint8_t transition);
  bool change_state_for_all_nodes(std::uint8_t transition);

  template<typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr
  create_service(
    const std::string & service_name,
    CallbackT && callback,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return rclcpp::create_service<ServiceT, CallbackT>(
      node_base_,
      node_services_,
      service_name,
      std::forward<CallbackT>(callback),
      qos_profile,
      group);
  }

protected:
  // Whether to automatically start up the system
  bool autostart_{false};

  // The names of the nodes to be managed, in the order of desired bring-up
  std::vector<std::string> node_names_;

  // The required node interfaces
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;

  // The node to use when calling lifecycle services
  rclcpp::Node::SharedPtr service_client_node_;

  // The service provided by this node
  rclcpp::Service<ManageLifecycleNodes>::SharedPtr manager_srv_;

  rclcpp::TimerBase::SharedPtr init_timer_;

  // A map of all nodes to be controlled
  std::map<std::string, std::shared_ptr<ros2_utils::LifecycleServiceClient>> node_map_;
  std::map<std::uint8_t, std::string> transition_label_map_;

  // A map of the expected transitions to primary states
  std::unordered_map<std::uint8_t, std::uint8_t> transition_state_map_;
};

}  // namespace ros2_lifecycle_manager

#endif  // ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
