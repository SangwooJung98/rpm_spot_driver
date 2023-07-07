/*
Software License Agreement (BSD License)

Authors : Brighten Lee <shlee@roas.co.kr>

Copyright (c) 2021, ROAS Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef SPOT_TELEOP__TELEOP_JOY_H_
#define SPOT_TELEOP__TELEOP_JOY_H_

#define POWER_ON 0
#define POWER_OFF 1
#define STAND 2
#define SIT 3
#define STAIR_MODE 4
#define WALK_MODE 5
#define SPEED_UP 6
#define SPEED_DOWN 7

#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"

using namespace std;

class SpotTeleop
{
public:
  SpotTeleop(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  virtual ~SpotTeleop() = default;

  /**
   * \brief Joystick callback
   * \param msg Joystick message
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  /**
   * \brief Power on/off, stand/sit the spot
   * \param msg Joystick message
   */
  void base(const sensor_msgs::Joy::ConstPtr& msg);

  /**
   * \brief Set stair or walk mode
   * \param msg Joystick message
   */
  bool mode(const sensor_msgs::Joy::ConstPtr& msg);

  /**
   * \brief Increase or decrease the maximum speed according to the button being pressed
   * \param msg Joystick message
   */
  void changeSpeed(const sensor_msgs::Joy::ConstPtr& msg);

  /**
   * \brief Publish the velocity command according to joystick axis values
   * \param msg Joystick message
   */
  void publishCmdVel(const sensor_msgs::Joy::ConstPtr& msg);

private:
  /// ROS parameters
  ros::NodeHandle nh_, nh_priv_;
  ros::Publisher pub_vel_;
  ros::Subscriber sub_joy_;

  /// rosservice
  ros::ServiceClient srv_power_on_, srv_power_off_, srv_stand_, srv_sit_, srv_stair_;
  std_srvs::Trigger dummy_;

  /// Joystick axes
  int axis_linear_x_, axis_linear_y_, axis_angular_;

  /// Joystick buttons
  int base_deadman_, operating_deadman_, power_on_, power_off_, stand_, sit_, stair_mode_, walk_mode_, speed_up_,
      speed_down_;

  /// Maximum values
  double max_linear_x_, max_linear_y_, max_angular_, cur_scale_;

  /// Button state
  bool pressedButton_[8];
};

#endif  // SPOT_TELEOP__TELEOP_JOY_H_