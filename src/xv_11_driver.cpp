/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Eric Perko, Chad Rockey
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
 *   * Neither the name of Case Western Reserve University nor the names of its
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


/*********************************************************************
 *
 * This xv-11 lidar driver started from a popular XV-11 or 'Neato' Lidar driver.
 *
 * https://github.com/ros2/demos/blob/master/dummy_robot/dummy_sensors/src/dummy_laser.cpp
 * And xv11 code:  https://github.com/rohbotics/xv_11_laser_driver
 * Also refer to conversion from ROS to ROS2  at https://index.ros.org/doc/ros2/Contributing/Migration-Guide/
 *
 * Launch:  ros2 run xv_11_driver xv_11_driver
 *********************************************************************/

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

// Make files need to change to support project include in a clean way
#include "../include/xv_11_driver/xv11_laser.h"

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#define DEG2RAD (M_PI/180.0)

// default xv11 dev name.  Suggest you use symbolic link for an easier to ricognize name on your system
// example:  sudo ln -s /dev/yourTtyDevName /dev/ttyXV11
// Here I assume you have setup symbolic link to your actual serial port tty driver to the lidar
#define XV11_PORT_DEFAULT "/dev/ttyUSB0"           // Serial device driver name (sym link to real dev)
#define XV11_BAUD_RATE_DEFAULT 115200              // Serial baud rate
#define XV11_FRAME_ID_DEFAULT "laser_link"        // frame_id in LaserScan messages
#define XV11_FIRMWARE_VERSION_DEFAULT 2            // XV-11 firmware rev. 1 is oldest


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("xv11_laser");

  // Declare parameters with default values
  node->declare_parameter<std::string>("port", XV11_PORT_DEFAULT);
  node->declare_parameter<int>("baud_rate", XV11_BAUD_RATE_DEFAULT);
  node->declare_parameter<std::string>("frame_id", XV11_FRAME_ID_DEFAULT);
  node->declare_parameter<int>("firmware_version", XV11_FIRMWARE_VERSION_DEFAULT);

  // Get parameters
  std::string port = node->get_parameter("port").as_string();
  int baud_rate = node->get_parameter("baud_rate").as_int();
  std::string frame_id = node->get_parameter("frame_id").as_string();
  int firmware_number = node->get_parameter("firmware_version").as_int();

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

  boost::asio::io_service io;

  try {
    RCLCPP_INFO(node->get_logger(), "Connecting to serial %s : %d", port.c_str(), baud_rate);
    xv_11_driver::XV11Laser laser(port, baud_rate, firmware_number, io);
    RCLCPP_DEBUG(node->get_logger(), "Serial connected");

    while (rclcpp::ok()) {
      auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
      scan->header.frame_id = frame_id;
      scan->header.stamp = node->now();
      laser.poll(scan.get());
      laser_pub->publish(*scan);
    }
    laser.close();
    return 0;
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "Error instantiating laser object. Check correct port and baud rate!");
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}
