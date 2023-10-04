/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <customized_rotate_recovery/limited_angle_rotate_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>
#include <tf2/utils.h>


// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(customized_rotate_recovery::LimitedAngleRotateRecovery, nav_core::RecoveryBehavior)

namespace customized_rotate_recovery
{
LimitedAngleRotateRecovery::LimitedAngleRotateRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void LimitedAngleRotateRecovery::initialize(std::string name, tf2_ros::Buffer* tf,
                                  costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;
    
    name_ = name;

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    // we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);
    private_nh.param("rotate_direction", rotate_direction_, 0); // 0: left, 1: right
    private_nh.param("rotate_angle", rotate_angle_, 90.);

    blp_nh.param("acc_lim_th", acc_lim_th_, 3.2);
    blp_nh.param("max_rotational_vel", max_rotational_vel_, 1.0);
    blp_nh.param("min_in_place_rotational_vel", min_rotational_vel_, 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

LimitedAngleRotateRecovery::~LimitedAngleRotateRecovery()
{
  delete world_model_;
}

void LimitedAngleRotateRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if (local_costmap_ == NULL)
  {
    ROS_ERROR("The costmap passed to the RotateRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Limited angle rotate recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::NodeHandle pnh("~/" + name_);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  
  pnh.getParam("rotate_direction", rotate_direction_);
  pnh.getParam("rotate_angle", rotate_angle_);

  geometry_msgs::PoseStamped global_pose;
  local_costmap_->getRobotPose(global_pose);

  double current_angle = tf2::getYaw(global_pose.pose.orientation);
  double start_angle = current_angle;
  double end_angle;
  if(rotate_direction_ == 0)
  {
      end_angle = start_angle + rotate_angle_ / 180. * M_PI;
  }
  else
  {
      end_angle = start_angle - rotate_angle_ / 180. * M_PI;
  }

//   bool got_180 = false;

//   while (n.ok() &&
//          (!got_180 ||
//           std::fabs(angles::shortest_angular_distance(current_angle, start_angle)) > tolerance_))
  while (n.ok() &&
          std::fabs(angles::shortest_angular_distance(current_angle, end_angle)) > 1. / 180. * M_PI)
  {
    // Update Current Angle
    local_costmap_->getRobotPose(global_pose);
    current_angle = tf2::getYaw(global_pose.pose.orientation);

    // compute the distance left to rotate
//     double dist_left;
//     if (!got_180)
//     {
//       // If we haven't hit 180 yet, we need to rotate a half circle plus the distance to the 180 point
//       double distance_to_180 = std::fabs(angles::shortest_angular_distance(current_angle, start_angle + M_PI));
//       dist_left = M_PI + distance_to_180;
// 
//       if (distance_to_180 < tolerance_)
//       {
//         got_180 = true;
//       }
//     }
//     else
//     {
//       // If we have hit the 180, we just have the distance back to the start
//       dist_left = std::fabs(angles::shortest_angular_distance(current_angle, start_angle));
//     }
    double dist;
    dist = std::fabs(angles::shortest_angular_distance(current_angle, end_angle));

    double x = global_pose.pose.position.x, y = global_pose.pose.position.y;

    // check if that velocity is legal by forward simulating
    double sim_angle = 0.0;
    while (sim_angle < dist)
    {
      double theta = current_angle + sim_angle;

      // make sure that the point is legal, if it isn't... we'll abort
      double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
      if (footprint_cost < 0.0)
      {
        ROS_ERROR("Rotate recovery can't rotate in place because there is a potential collision. Cost: %.2f",
                  footprint_cost);
        return;
      }

      sim_angle += sim_granularity_;
    }

    // compute the velocity that will let us stop by the time we reach the goal
    double vel = sqrt(2 * acc_lim_th_ * dist);

    // make sure that this velocity falls within the specified limits
    vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    
    // 0: left, 1: right
    if(rotate_direction_ == 0)
    {
        cmd_vel.angular.z = vel;
    }
    else
    {
        cmd_vel.angular.z = -vel;
    }

    vel_pub.publish(cmd_vel);

    r.sleep();
  }
}
};  // namespace customized_rotate_recovery
