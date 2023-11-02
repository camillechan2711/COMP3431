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
#define END 10
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

	robot_pose_ = 0.0;
	prev_robot_pose_ = 0.0;

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

	robot_pose_ = yaw;
}

double averageDistance(int start_range, const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
       double total = 0;
       int curr = 0; 
       for (int i = start_range; i < start_range + END; i++)
       {
	        if (i < 0) {
		       curr = 360 + i; 
		} else {
		       curr = i;
		}

	       	if (std::isinf(msg->ranges.at(curr)))
                {
			total += msg->range_max;
                } else {
        		total += msg->ranges.at(curr);
		}
       }
       return total/ (END - start_range);
}

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	// uint16_t scan_angle[6] = {0, 90, 270, 315, 225, 337};
/*
	for (int num = 0; num < 6; num++)
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
*/
	scan_data_[CENTER] = averageDistance(-5, msg);
	scan_data_[RIGHT] = averageDistance(265, msg);
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
	double escape_range = 75.0 * DEG2RAD;
	double check_forward_dist = 0.37;
	double check_side_dist = 0.3;
//	double check_f_b_right_dist = 0.5;
	double start = 0.09;

	switch (turtlebot3_state_num)
	{         

		case GET_TB3_DIRECTION:
			RCLCPP_INFO(this->get_logger(), "CENTER: %f", scan_data_[CENTER]);
			RCLCPP_INFO(this->get_logger(), "RIGHT: %f", scan_data_[RIGHT]);
	//		RCLCPP_INFO(this->get_logger(), "F_RIGHT: %f", scan_data_[F_RIGHT]);
	//		RCLCPP_INFO(this->get_logger(), "B_RIGHT: %f", scan_data_[B_RIGHT]);

			if (scan_data_[CENTER] < start && scan_data_[RIGHT] < start) 
				
				// && scan_data_[F_RIGHT] < start && scan_data_[B_RIGHT] < start)
			{
		        	turtlebot3_state_num = TB3_DRIVE_FORWARD;
				RCLCPP_INFO(this->get_logger(), "START!!!!!!!!!!!!!!!");
			}


			else if (scan_data_[CENTER] > check_forward_dist
					&& scan_data_[RIGHT] > check_side_dist)
				//	&& scan_data_[F_RIGHT] > check_f_b_right_dist)
			{
					prev_robot_pose_ = robot_pose_;
                                        turtlebot3_state_num = FIND_WALL;
					RCLCPP_INFO(this->get_logger(), "FIND WALL");
			}
			else if (scan_data_[CENTER] > check_forward_dist 
					&& scan_data_[RIGHT] < check_side_dist)
				//		|| scan_data_[F_RIGHT] < check_f_b_right_dist
				//		|| scan_data_[B_RIGHT] < check_f_b_right_dist))
			{

				
					if (scan_data_[RIGHT] < 0.23) {
					       turtlebot3_state_num = GET_TB3_DIRECTION;
				               update_cmd_vel(0.0,0.2);
					       RCLCPP_INFO(this->get_logger(), "obstacle on right too close. turning left slightly");
					}
				
				        else {
				               turtlebot3_state_num = TB3_DRIVE_FORWARD;
				               RCLCPP_INFO(this->get_logger(), "follow right wall>>>>>>>>>>>");
				        }
		       	}
			
	                else if (scan_data_[CENTER] < check_forward_dist 
					//	|| scan_data_[R_CENTER] < check_forward_dist) 
					&& scan_data_[RIGHT] < check_side_dist)
				//	    || scan_data_[F_RIGHT] < check_f_b_right_dist))
			{
				        prev_robot_pose_ = robot_pose_;
                                        turtlebot3_state_num = TB3_LEFT_TURN;
					RCLCPP_INFO(this->get_logger(), "forward wall and right wall detected");
			}

			else if (scan_data_[CENTER] < check_forward_dist && scan_data_[RIGHT] > check_side_dist)
			{
				  prev_robot_pose_ = robot_pose_;
                                  turtlebot3_state_num = TB3_LEFT_TURN;
                                  RCLCPP_INFO(this->get_logger(), "forward wall detected but no RIGHT wall");
			}			  
			else
			{
				RCLCPP_INFO(this->get_logger(), "no condition");
			
			}
			/*
			if (scan_data_[CENTER] > check_forward_dist && scan_data_[RIGHT] > check_side_dist)
			{
				turtlebot3_state_num = FIND_WALL;
				RCLCPP_INFO(this->get_logger(), "find wall");
			}
			*/
			break;

		case FIND_WALL:
			update_cmd_vel(0.2, -1 * 0.9);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;

		case TB3_DRIVE_FORWARD:
			update_cmd_vel(LINEAR_VELOCITY, 0.0);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;

		case TB3_RIGHT_TURN:
			
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
			{
				turtlebot3_state_num = GET_TB3_DIRECTION;
			}
			else
			{
				update_cmd_vel(0.0, -1 * ANGULAR_VELOCITY);
			}
			break;
			
			/*
			update_cmd_vel(0.0,-1* ANGULAR_VELOCITY);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			
			break;
			*/
		case TB3_LEFT_TURN:
			
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
			{
				turtlebot3_state_num = GET_TB3_DIRECTION;
			}
			else
			{
				update_cmd_vel(0.0, ANGULAR_VELOCITY);
			}
			break;
			

			/*
			update_cmd_vel(0.0, ANGULAR_VELOCITY);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			
			break;
			*/
		/*	
		case TB3_SEARCH_LEFT:
                        update_cmd_vel(0.2, -0.4);
                        turtlebot3_state_num = GET_TB3_DIRECTION;
                        break;
		*/

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
