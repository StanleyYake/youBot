//
// Simple demo program that calls the youBot ROS wrapper with a trajectory
//

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "youbot_ros_simple_trajectory");
	ros::NodeHandle n;

	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("arm_1/arm_controller/follow_joint_trajectory", true);

	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();

	ROS_INFO("Action server started, sending trajectory.");

	// setup trajectory message
	control_msgs::FollowJointTrajectoryGoal msg;

	// some arbitrary points of a not too useful trajectory
	const int nPoints = 4;
	double values[nPoints][5] = {
		{1.5, 0.2, -0.3, 0.3, 1.5},
		{2.0, 0.2, -0.3, 0.8, 1.5},
		{2.5, 0.2, -0.3, 0.6, 1.5},
		{1.5, 0.2, -0.3, 0.3, 1.5} };

	// set values for all points of trajectory
	for (int p = 0; p < nPoints; p++) { // iterate over all points
		trajectory_msgs::JointTrajectoryPoint point;
		for (int i = 0; i < 5; i++) { // 5 DOF
			point.positions.push_back(values[p][i]);
			point.velocities.push_back(0);
			point.accelerations.push_back(0);
		}
		point.time_from_start = ros::Duration((p+1) / 2.0);
		msg.trajectory.points.push_back(point);
	}

	// set joint names
	for (int i = 0; i < 5; i++) {
		std::stringstream jointName;
		jointName << "arm_joint_" << (i + 1);
		msg.trajectory.joint_names.push_back(jointName.str());
	}
	
	// fill message header and sent it out
	msg.trajectory.header.frame_id = "arm_link_0";
	msg.trajectory.header.stamp = ros::Time::now();
	ac.sendGoal(msg);

	// wait for reply that action was completed (or cancel after 10 sec)
	ac.waitForResult(ros::Duration(10));
	
	return 0;
}

