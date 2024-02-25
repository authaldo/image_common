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

#ifndef IMAGE_TRANSPORT__NODE_INTERFACES_HPP_
#define IMAGE_TRANSPORT__NODE_INTERFACES_HPP_

#include <memory>
#include <utility>

#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>

namespace image_transport
{
/**
 * Collection of node interfaces which are required for the image transport functionality.
 */
using RequiredInterfaces = rclcpp::node_interfaces::NodeInterfaces<rclcpp::node_interfaces::NodeBaseInterface,
                                                                   rclcpp::node_interfaces::NodeLoggingInterface,
                                                                   rclcpp::node_interfaces::NodeTimersInterface,
                                                                   rclcpp::node_interfaces::NodeTopicsInterface,
                                                                   rclcpp::node_interfaces::NodeParametersInterface>;

/*
 * Note: three versions of this function are unfortunately necessary since the constructor taking a shared_ptr
 *       of the 'node_like' has been removed in the rclcpp pull request 2075.
 */
template<typename NodeTPtr>
std::shared_ptr<RequiredInterfaces> create_node_interfaces(std::shared_ptr<NodeTPtr> node)
{
  return std::make_shared<RequiredInterfaces>(*node);
}

template<typename NodeT>
std::shared_ptr<RequiredInterfaces> create_node_interfaces(NodeT* && node)
{
  return std::make_shared<RequiredInterfaces>(*node);
}

template<typename NodeT>
std::shared_ptr<RequiredInterfaces> create_node_interfaces(NodeT && node)
{
    return std::make_shared<RequiredInterfaces>(node);
}

}  // namespace image_transport

#endif  // IMAGE_TRANSPORT__NODE_INTERFACES_HPP_
