#include <dynamic_reconfigure/server.h>
#include "youbot_ros_simple_trajectory/youbot_arm_gripperConfig.h"
#include <ros/ros.h>
// dynamic callback
void callback(youbot_ros_simple_trajectory::youbot_arm_gripperConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request : %f %f %lf %f %f %f %f ",
           config.arm1,
           config.arm2,
           config.arm3,
           config.arm4,
           config.arm5,
           config.gripper_l_param,
           config.gripper_r_param
		  );
  // do nothing for now
}

// Dynamic server
  int main(int argc, char **argv) {
  ros::init(argc, argv, "youbot_dy_server");

  dynamic_reconfigure::Server<youbot_ros_simple_trajectory::youbot_arm_gripperConfig> server;
  dynamic_reconfigure::Server<youbot_ros_simple_trajectory::youbot_arm_gripperConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
