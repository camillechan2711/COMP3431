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
	for (int i = 0; i < RANGE; i ++)
       	{
		scan_data_center[i] = 0.0;
		scan_data_right[i] = 0.0;
	}

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
/*
double averageDistance(int start_range, const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
       double total = 0; 
       for (int curr = start_range; curr < start_range + END; curr++)
       {
	        if (curr < 0) {
		       curr += 360; 
		}
	       	if (std::isinf(msg->ranges.at(curr)))
                {
			total += msg->range_max;
                } else {
        		total += msg->ranges.at(curr);
		}
	//	RCLCPP_INFO(WallFollower<-get_logger(), "total: %f", total);
       }
       return total/ END;
}
*/

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	uint16_t scan_angle_center[RANGE] = {330, 335, 340, 345, 350, 355, 0, 10, 15, 20};
	uint16_t scan_angle_right[RANGE] = {315, 310, 305,300, 295, 290, 285, 280, 275, 270};

	for (int num = 0; num < RANGE; num++) {
        	if (std::isinf(msg->ranges.at(scan_angle_center[num]))) {
            		scan_data_center[num] = msg->range_max;
        	} else {
            		scan_data_center[num] = msg->ranges.at(scan_angle_center[num]);
       		}
    	}

	for (int num = 0; num < RANGE; num++) {
        	if (std::isinf(msg->ranges.at(scan_angle_right[num]))) {
            		scan_data_right[num] = msg->range_max;
        	} else {
            		scan_data_right[num] = msg->ranges.at(scan_angle_right[num]);
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

bool check_distance_center(double *scan_data_ptr, double dist) {
    	for (int i = 0; i < RANGE; i++) {
        	if (scan_data_ptr[i] < dist) {
            		return true;
        	}
    	}
    	return false;
}

bool check_distance_right(double *scan_data_ptr, double dist) {
        double extreme = 0.5;

        for (int i = 0; i < RANGE; i++) {
                if (scan_data_ptr[i] < dist || scan_data_ptr[0] < extreme) {
                        return true;
                }
        }
        return false;
}

bool check_distance_right_too_close(double *scan_data_ptr) {
	double f_right = 0.33;
	double right = 0.23;
	for (int i = 0; i < RANGE - 5; i++) {
		if (scan_data_ptr[i] < f_right)
	       	{
			return true;
		}
	}
	for (int i = 5; i < RANGE; i++) {
		if (scan_data_ptr[i] < right)
	       	{
			return true;
		}
	}
	return false;
}

/********************************************************************************
** Update functions
********************************************************************************/
void WallFollower::update_callback()
{
	static uint8_t turtlebot3_state_num = 0;
	double escape_range = 75.0 * DEG2RAD;
	double forward = 0.37;
	double side = 0.27;
	double start = 0.09;

	switch (turtlebot3_state_num)
	{         

		case GET_TB3_DIRECTION:
		       	if (check_distance_center(center_ptr, start) && check_distance_right(right_ptr, start)) 			
			{
		        		turtlebot3_state_num = TB3_DRIVE_FORWARD;
					RCLCPP_INFO(this->get_logger(), "START!!!!!!!!!!!!!!!");
			}


			else if (!check_distance_center(center_ptr, forward) && !check_distance_right(right_ptr, side))
			{
					prev_robot_pose_ = robot_pose_;
                                        turtlebot3_state_num = FIND_WALL;
					RCLCPP_INFO(this->get_logger(), "NO wall");
			}
			else if (!check_distance_center(center_ptr, forward) && check_distance_right(right_ptr, side))
			{
				        turtlebot3_state_num = TB3_DRIVE_FORWARD;
                                        RCLCPP_INFO(this->get_logger(), "follow right wall>>>>>>>>>>>");


				
					if (check_distance_right_too_close(right_ptr)) {
					       turtlebot3_state_num = GET_TB3_DIRECTION;
				               update_cmd_vel(0.15,0.2);
					       RCLCPP_INFO(this->get_logger(), "obstacle on right too close. turning left slightly");
					}	        
		       	}
			
	                else if (check_distance_center(center_ptr, forward) && check_distance_right(right_ptr, side))
			{
				        prev_robot_pose_ = robot_pose_;
                                  	turtlebot3_state_num = TB3_LEFT_TURN;
					RCLCPP_INFO(this->get_logger(), "forward wall and right wall detected");
			}

			else if (check_distance_center(center_ptr, forward) && !check_distance_right(right_ptr, side))
			{
				  	prev_robot_pose_ = robot_pose_;
                                  	turtlebot3_state_num = FIND_WALL;
                                  	RCLCPP_INFO(this->get_logger(), "forward wall detected but no RIGHT wall");
			}			  
			else
			{
					RCLCPP_INFO(this->get_logger(), "no condition");
			
			}
			
			break;

		case FIND_WALL:
			update_cmd_vel(0.15, -1 * 0.9);
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
