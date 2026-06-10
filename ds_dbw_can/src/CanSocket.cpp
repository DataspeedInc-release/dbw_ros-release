/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include "CanSocket.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cstring>

namespace ds_dbw_can {

CanSocket::~CanSocket() {
  stopReceiving();
  if (sockfd_ >= 0) {
    close(sockfd_);
  }
}

bool CanSocket::init(const std::string &interface) {
  // Create socket
  sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sockfd_ < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("can_interface"), "Error creating CAN socket");
    return false;
  }

  // Find interface
  struct ifreq ifr;
  std::strcpy(ifr.ifr_name, interface.c_str());
  if (ioctl(sockfd_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("can_interface"), "Error getting interface index for " << interface);
    close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  // Bind socket to CAN interface
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sockfd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("can_interface"), "Error binding socket to " << interface);
    close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  // Make socket non-blocking
  int flags = fcntl(sockfd_, F_GETFL, 0);
  fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("can_interface"), "CAN socket initialized on " << interface);
  return true;
}

bool CanSocket::sendFrame(const can_frame &frame) {
  if (sockfd_ < 0) {
    return false;
  }

  ssize_t bytes_sent = write(sockfd_, &frame, sizeof(struct can_frame));
  if (bytes_sent != sizeof(struct can_frame)) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("can_interface"), "Error sending CAN frame");
    return false;
  }

  return true;
}

void CanSocket::startReceiving() {
  if (receiving_.load() || sockfd_ < 0) {
    return;
  }

  receiving_.store(true);
  receive_thread_ = std::thread(&CanSocket::receiveLoop, this);
}

void CanSocket::stopReceiving() {
  if (!receiving_.load()) {
    return;
  }

  receiving_.store(false);
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}

void CanSocket::receiveLoop() {
  struct can_frame frame;
  while (receiving_.load()) {
    ssize_t bytes_read = read(sockfd_, &frame, sizeof(struct can_frame));

    if (bytes_read == sizeof(struct can_frame)) {
      frame_callback_(frame);
    } else if (bytes_read < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // No data available, continue
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("can_interface"), "Error reading CAN frame: " << strerror(errno));
        break;
      }
    }
  }
}

}  // namespace ds_dbw_can
