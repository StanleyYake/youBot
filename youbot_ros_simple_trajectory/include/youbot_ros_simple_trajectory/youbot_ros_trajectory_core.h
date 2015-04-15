#ifndef SR_YOUBOT_ROS_TRAJECTORY_CORE_H
#define SR_YOUBOT_ROS_TRAJECTORY_CORE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
#include "youbot_ros_simple_trajectory/youbot_ros_simple_trajectory_Data.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <youbot_ros_simple_trajectory/youbot_arm_gripperConfig.h>

class Youbot
{
public:
	//! Constructor.
	Youbot();
	
	//! Destructor.
	~Youbot();
	
	//! Callback function for dynamic reconfigure server.
  void configCallback(youbot_ros_simple_trajectory::youbot_arm_gripperConfig &config, uint32_t level);
  
  //! Publish the message.
  void publishMessage(ros::Publisher *pub_message);
  
  //! Callback function for subscriber.
  void messageCallback(const youbot_ros_simple_trajectory::youbot_ros_simple_trajectory_Data::ConstPtr &msg);
  
  double arm1_;
  double arm2_;
  double arm3_;
  double arm4_;
  double arm5_;
  
  double gripper_l_param_;
  double gripper_r_param_;
  
};
#endif // SR_YOUBOT_ROS_TRAJECTORY_CORE_H
