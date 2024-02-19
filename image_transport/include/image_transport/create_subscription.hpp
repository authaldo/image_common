// Copyright (c) 2009, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef IMAGE_TRANSPORT__CREATE_SUBSCRIPTION_HPP_
#define IMAGE_TRANSPORT__CREATE_SUBSCRIPTION_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/subscription_options.hpp"

#include "image_transport/subscriber.hpp"
#include "image_transport/node_interfaces.hpp"
#include "image_transport/visibility_control.hpp"

namespace image_transport
{

/**
 * \brief Subscribe to an image topic, free function version.
 */
IMAGE_TRANSPORT_PUBLIC
Subscriber create_subscription(
  std::shared_ptr<RequiredInterfaces> node_interfaces,
  const std::string & base_topic,
  const Subscriber::Callback & callback,
  const std::string & transport,
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
  rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions());

template<typename NodeT>
Subscriber create_subscription(
  NodeT && node,
  const std::string & base_topic,
  const Subscriber::Callback & callback,
  const std::string & transport,
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
  rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions())
{
  return create_subscription(
    create_node_interfaces(std::forward<NodeT>(node)),
    base_topic, callback, transport, custom_qos, options);
}

}  // namespace image_transport

#endif  // IMAGE_TRANSPORT__CREATE_SUBSCRIPTION_HPP_
