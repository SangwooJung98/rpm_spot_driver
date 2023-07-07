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

#include "spot_teleop/teleop_joy.h"

SpotTeleop::SpotTeleop(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : nh_(nh)
  , nh_priv_(nh_priv)
  , axis_linear_x_(0)
  , axis_linear_y_(0)
  , axis_angular_(0)
  , base_deadman_(0)
  , operating_deadman_(0)
  , power_on_(0)
  , power_off_(0)
  , stand_(0)
  , sit_(0)
  , stair_mode_(0)
  , walk_mode_(0)
  , speed_up_(0)
  , speed_down_(0)
  , max_linear_x_(1.0)
  , max_linear_y_(0.5)
  , max_angular_(1.5)
  , cur_scale_(0.4)
{
  nh_priv_.getParam("axis_linear_x", axis_linear_x_);
  nh_priv_.getParam("axis_linear_y", axis_linear_y_);
  nh_priv_.getParam("axis_angular", axis_angular_);
  nh_priv_.getParam("base_deadman", base_deadman_);
  nh_priv_.getParam("operating_deadman", operating_deadman_);
  nh_priv_.getParam("power_on", power_on_);
  nh_priv_.getParam("power_off", power_off_);
  nh_priv_.getParam("stand", stand_);
  nh_priv_.getParam("sit", sit_);
  nh_priv_.getParam("stair_mode", stair_mode_);
  nh_priv_.getParam("walk_mode", walk_mode_);
  nh_priv_.getParam("speed_up", speed_up_);
  nh_priv_.getParam("speed_down", speed_down_);
  nh_priv_.getParam("max_linear_x", max_linear_x_);
  nh_priv_.getParam("max_linear_y", max_linear_y_);
  nh_priv_.getParam("max_angular", max_angular_);

  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  sub_joy_ = nh_.subscribe("/joy", 10, &SpotTeleop::joyCallback, this);

  srv_power_on_ = nh_.serviceClient<std_srvs::Trigger>("spot/power_on");
  srv_power_off_ = nh_.serviceClient<std_srvs::Trigger>("spot/power_off");
  srv_stand_ = nh_.serviceClient<std_srvs::Trigger>("spot/stand");
  srv_sit_ = nh_.serviceClient<std_srvs::Trigger>("spot/sit");
  srv_stair_ = nh_.serviceClient<std_srvs::SetBool>("spot/stair_mode");

  for (int i = 0; i < 8; i++)
    pressedButton_[i] = false;
}

void SpotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (msg->buttons[base_deadman_])
  {
    base(msg);
  }
  else if (msg->buttons[operating_deadman_])
  {
    if (msg->buttons[stair_mode_] || msg->buttons[walk_mode_])
    {
      mode(msg);
      return;
    }
    else if (msg->buttons[speed_up_] || msg->buttons[speed_down_])
    {
      changeSpeed(msg);
      return;
    }
    else
      publishCmdVel(msg);

    pressedButton_[POWER_ON] = false;
    pressedButton_[POWER_OFF] = false;
    pressedButton_[STAND] = false;
    pressedButton_[SIT] = false;
    pressedButton_[STAIR_MODE] = false;
    pressedButton_[WALK_MODE] = false;
    pressedButton_[SPEED_UP] = false;
    pressedButton_[SPEED_DOWN] = false;
  }
}

void SpotTeleop::base(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (msg->buttons[power_on_])
  {
    if (!pressedButton_[POWER_ON])
    {
      pressedButton_[POWER_ON] = true;
      srv_power_on_.call(dummy_);
    }
  }
  else if (msg->buttons[power_off_])
  {
    if (!pressedButton_[POWER_OFF])
    {
      pressedButton_[POWER_OFF] = true;
      srv_power_off_.call(dummy_);
    }
  }
  else if (msg->buttons[stand_])
  {
    if (!pressedButton_[STAND])
    {
      pressedButton_[STAND] = true;
      srv_stand_.call(dummy_);
    }
  }
  else if (msg->buttons[sit_])
  {
    if (!pressedButton_[SIT])
    {
      pressedButton_[SIT] = true;
      srv_sit_.call(dummy_);
    }
  }
  else
  {
    pressedButton_[POWER_ON] = false;
    pressedButton_[POWER_OFF] = false;
    pressedButton_[STAND] = false;
    pressedButton_[SIT] = false;
  }
}

bool SpotTeleop::mode(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (msg->buttons[stair_mode_])
  {
    if (!pressedButton_[STAIR_MODE])
    {
      pressedButton_[STAIR_MODE] = true;
      std_srvs::SetBool cmd;
      cmd.request.data = true;
      srv_stair_.call(cmd);
    }
  }
  else if (msg->buttons[walk_mode_])
  {
    if (!pressedButton_[WALK_MODE])
    {
      pressedButton_[WALK_MODE] = true;
      std_srvs::SetBool cmd;
      cmd.request.data = false;
      srv_stair_.call(cmd);
    }
  }
  else
  {
    pressedButton_[STAIR_MODE] = false;
    pressedButton_[WALK_MODE] = false;
  }
}

void SpotTeleop::changeSpeed(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (msg->buttons[speed_up_])
  {
    if (!pressedButton_[SPEED_UP])
    {
      pressedButton_[SPEED_UP] = true;
      cur_scale_ += 0.2;
      if (cur_scale_ >= 1.0)
        cur_scale_ = 1.0;
    }
  }
  else if (msg->buttons[speed_down_])
  {
    if (!pressedButton_[SPEED_DOWN])
    {
      pressedButton_[SPEED_DOWN] = true;
      cur_scale_ -= 0.2;
      if (cur_scale_ <= 0.0)
        cur_scale_ = 0.0;
    }
  }
  else
  {
    pressedButton_[SPEED_UP] = false;
    pressedButton_[SPEED_DOWN] = false;
  }
}

void SpotTeleop::publishCmdVel(const sensor_msgs::Joy::ConstPtr& msg)
{
  geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

  vel.linear.x = cur_scale_ * max_linear_x_ * msg->axes[axis_linear_x_];
  vel.linear.y = cur_scale_ * max_linear_y_ * msg->axes[axis_linear_y_];
  vel.angular.z = cur_scale_ * max_angular_ * msg->axes[axis_angular_];

  pub_vel_.publish(vel);
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "teleop_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  ros::Rate rate(30.0);

  SpotTeleop teleop(nh, nh_priv);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return EXIT_FAILURE;
}