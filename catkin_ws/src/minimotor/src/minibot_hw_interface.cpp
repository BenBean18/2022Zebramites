/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the MiniBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <minibot_control/minibot_hw_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <ros/ros.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/write.hpp>
#include <minibot_control/interpolating_map.h>
using boost::asio::serial_port;

namespace minibot_control
{

MiniBotJoint::MiniBotJoint(JointType t) {
  type = t;
}

MiniBotJoint::MiniBotJoint() {

}

MiniBotMotorJoint::MiniBotMotorJoint(serial_port *p, uint8_t port, double velocity_mult, double velocity_y_intercept, ros::NodeHandle &nh, bool inverted) : MiniBotJoint(motor) {
  this->type = motor;
  this->inverted = inverted;
  this->port = port;
  this->velocity_mult = velocity_mult;
  this->velocity_y_intercept = velocity_y_intercept;
  this->nh = nh;
  this->pub = nh.advertise<std_msgs::Float64>("/motor"+std::to_string(port)+"/command", 5);
  this->p = p;
}

void MiniBotMotorJoint::sendCommand(double cmd) {
  if (this->lastCmdSent != cmd) {
    double output = (inverted ? -1.0d : 1.0d) * ((cmd >= 0 ? 1 : -1) * velocity_y_intercept + velocity_mult * cmd);
    if (cmd == 0) {
      output = 0.0;
    }
    this->lastCmdSent = cmd;
    std::string toWrite = "z" + std::to_string(port) + ";" + std::to_string(output) + ";";
    auto cs = toWrite.c_str();
    ROS_INFO_STREAM(toWrite);
    //boost::asio::write(*p, const_buffer(cs, strlen(cs)));
    p->write_some(const_buffer(cs, strlen(cs)));
  }
  // ROS_INFO_STREAM("Setting port " << std::to_string(port) << " to " << output);
}

JointType getType(std::string s) {
  if (s == "motor") {
    return motor;
  } else {
    return motor;
  }
}

MiniBotJoint* parseJoint(ros::NodeHandle nh, const std::string &n, serial_port *p) {
  ros::NodeHandle rpnh(
      nh, n);
  std::size_t error = 0;
  std::string jointType;
  error += !rosparam_shortcuts::get(n, rpnh, "type", jointType);
  JointType t;
  t = getType(jointType);
  if (t == motor) {
    bool inverted;
    int port;
    double velocity_mult;
    double vyi;
    error += !rosparam_shortcuts::get(n, rpnh, "inverted", inverted);
    error += !rosparam_shortcuts::get(n, rpnh, "port", port);
    error += !rosparam_shortcuts::get(n, rpnh, "velocity_mult", velocity_mult);
    error += !rosparam_shortcuts::get(n, rpnh, "velocity_y_intercept", vyi);
    MiniBotMotorJoint* j = new MiniBotMotorJoint(p, port, velocity_mult, vyi, nh, inverted);
    ROS_INFO_STREAM("Motor joint, name = " << n << ", port = " << port << ", inverted = " << inverted << ", vmult = " << velocity_mult);
    rosparam_shortcuts::shutdownIfError(n, error);
    return j;
  }
  return new MiniBotJoint(t);
}

MiniBotHWInterface::MiniBotHWInterface(ros::NodeHandle& nh, serial_port *port, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model), p(port)
{
  ROS_INFO_NAMED("minibot_hw_interface", "MiniBotHWInterface Ready.");
  for (std::string n : joint_names_) {
    std::shared_ptr<MiniBotJoint> jp = std::shared_ptr<MiniBotJoint>(parseJoint(nh, n, p));
    joints_[n] = jp;
  }
}

void MiniBotHWInterface::read(ros::Duration& elapsed_time)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR READ COMMAND FROM USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  
}

void MiniBotHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
  // VELOCITY FROM POSITION WITH SMOOTHING, SEE
  // sim_hw_interface.cpp IN THIS PACKAGE
  //
  // DUMMY PASS-THROUGH CODE
  boost::asio::write(*p, const_buffer("\n", 1));
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) {
    auto thisJoint = joints_[joint_names_[joint_id]];
    thisJoint->sendCommand(joint_velocity_command_[joint_id]);
    joint_velocity_[joint_id] = joint_velocity_command_[joint_id];
    joint_position_[joint_id] += joint_velocity_command_[joint_id] * elapsed_time.toSec();
  }
  // END DUMMY CODE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void MiniBotHWInterface::enforceLimits(ros::Duration& period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

}  // namespace minibot_control
