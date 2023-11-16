// Copyright 2019 ROBOTIS CO., LTD.
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
//
// Authors: Taehun Lim (Darby), Ryan Shim
//
// Modified by Claude Sammut for COMP3431
// Use this code as the basis for a wall follower


#include "wall_follower/wall_follower.hpp"

#include <memory>


using namespace std::chrono_literals;

WallFollower::WallFollower()
: Node("wall_follower_node")
{
	/************************************************************
	** Initialise variables
	************************************************************/
	scan_data_[0] = 0.0;
	scan_data_[1] = 0.0;
	scan_data_[2] = 0.0;
	for (int i = 0; i < 4; i++ ) {
		tf2Scalar zero = 0.0;
		robot_odom_[i] = zero;
	}	
	moved_ = false;
	robot_pose_ = 0.0;
	prev_robot_pose_ = 0.0;
	prev_robot_pose_ = robot_pose_; 

	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	// Initialise publishers
	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

	// Initialise subscribers
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", \
		rclcpp::SensorDataQoS(), \
		std::bind(
			&WallFollower::scan_callback, \
			this, \
			std::placeholders::_1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", qos, std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));

	/************************************************************
	** Initialise ROS timers
	************************************************************/
	update_timer_ = this->create_wall_timer(10ms, std::bind(&WallFollower::update_callback, this));

	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}

WallFollower::~WallFollower()
{
	RCLCPP_INFO(this->get_logger(), "Wall follower node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	tf2::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	robot_odom_ = q;
	robot_pose_ = yaw;
}

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	uint16_t scan_angle[3] = {0, 30, 330};

	for (int num = 0; num < 3; num++)
	{
		if (std::isinf(msg->ranges.at(scan_angle[num])))
		{
			scan_data_[num] = msg->range_max;
		}
		else
		{
			scan_data_[num] = msg->ranges.at(scan_angle[num]);
		}
	}
}

void WallFollower::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/

void WallFollower::update_callback()
{
	static uint8_t turtlebot3_state_num = 0;
	// double escape_range = 30.0 * DEG2RAD;
	double dist = 0.5;
	double f_dist = 0.5; 
	switch (turtlebot3_state_num)
	{
		case GET_TB3_DIRECTION:
			if (robot_odom_.length() > 0.3) 
			{
				moved_ = true;
				RCLCPP_INFO(this->get_logger(), "Moved from start!");
			} 
			if (moved_ && robot_odom_.length() <= 0.2)
			{
				turtlebot3_state_num = TB3_FINISH;
				RCLCPP_INFO(this->get_logger(), "Finished!");
				break;
			}

			if (scan_data_[FRONT] > f_dist && scan_data_[LEFT] > dist && scan_data_[RIGHT] > dist) {
				turtlebot3_state_num = FIND_WALL; // no wall found 1
				RCLCPP_INFO(this->get_logger(), "no wall found on right. finding wall");
			} else if (scan_data_[FRONT] < f_dist && scan_data_[LEFT] > dist && scan_data_[RIGHT] > dist) {
				turtlebot3_state_num = TB3_LEFT_TURN; // obstacle front 2
				RCLCPP_INFO(this->get_logger(), "obstacle in front. turning left");
			} else if (scan_data_[FRONT] > f_dist && scan_data_[LEFT] > dist && scan_data_[RIGHT] < dist) {
				turtlebot3_state_num = TB3_DRIVE_FORWARD; // obstacle right case 3
				RCLCPP_INFO(this->get_logger(), "obstacle on right. following wall");
			} else if (scan_data_[FRONT] > f_dist && scan_data_[LEFT] < dist && scan_data_[RIGHT] > dist) {
				turtlebot3_state_num = FIND_WALL; // obstacle left case 4
				RCLCPP_INFO(this->get_logger(), "obstacle on left. finding wall on right");
			} else if (scan_data_[FRONT] < f_dist && scan_data_[LEFT] > dist && scan_data_[RIGHT] < dist) {
				turtlebot3_state_num = TB3_LEFT_TURN; // obstacle right and front 5
				RCLCPP_INFO(this->get_logger(), "obstacle on right and front. turning left");
			} else if (scan_data_[FRONT] < f_dist && scan_data_[LEFT] < dist && scan_data_[RIGHT] > dist) {
				turtlebot3_state_num = TB3_LEFT_TURN; // obstacle front and left
				RCLCPP_INFO(this->get_logger(), "obstacle on left and front. turning left");
			} else if (scan_data_[FRONT] < f_dist && scan_data_[LEFT] < dist && scan_data_[RIGHT] < dist) {
				turtlebot3_state_num = TB3_LEFT_TURN; // obstacle front left and right
				RCLCPP_INFO(this->get_logger(), "obstacle on left and front and right. turning left");
			} else if (scan_data_[FRONT] > f_dist && scan_data_[LEFT] < dist && scan_data_[RIGHT] < dist) {
				turtlebot3_state_num = FIND_WALL; // obstacle left and right
				RCLCPP_INFO(this->get_logger(), "obstacle on left and right. finding wall");
			} 
			break;

		case TB3_DRIVE_FORWARD:
			update_cmd_vel(LINEAR_VELOCITY, 0.0);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;

		case FIND_WALL:
			update_cmd_vel(0.2, -0.3);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;
			
		case TB3_LEFT_TURN:
			/*
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
			{
				turtlebot3_state_num = GET_TB3_DIRECTION;
			}
			*/
		
			update_cmd_vel(0.0, ANGULAR_VELOCITY);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;
		
		case TB3_RIGHT_TURN: 
			/*
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
				turtlebot3_state_num = GET_TB3_DIRECTION;
			}
			*/
			update_cmd_vel(0.0, ANGULAR_VELOCITY);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;
		case TB3_FINISH:
			turtlebot3_state_num = TB3_FINISH;
			break;
	
		default:
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;
		
		
	}
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();
	return 0;
}