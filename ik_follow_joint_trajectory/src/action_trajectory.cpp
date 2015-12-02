//http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action
//2015年04月16日 20时34分43秒 

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTolerance.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("arm_1/arm_controller/follow_joint_trajectory", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
  {
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("arm_joint_1");
    goal.trajectory.joint_names.push_back("arm_joint_2");
    goal.trajectory.joint_names.push_back("arm_joint_3");
    goal.trajectory.joint_names.push_back("arm_joint_4");
    goal.trajectory.joint_names.push_back("arm_joint_5");
//    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
//    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(4);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(5);
    goal.trajectory.points[ind].positions[0] = 0.15;
    goal.trajectory.points[ind].positions[1] = 0.15;
    goal.trajectory.points[ind].positions[2] = -0.21;
    goal.trajectory.points[ind].positions[3] = 0.15;
    goal.trajectory.points[ind].positions[4] = 0.15;
//    goal.trajectory.points[ind].positions[5] = 0.0;
//    goal.trajectory.points[ind].positions[6] = 0.0;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(5);
    for (size_t a = 0; a < 5; ++a)
    {
      goal.trajectory.points[ind].velocities[a] = 0.0;
    }
    
    goal.trajectory.points[ind].accelerations.resize(5);
    for (size_t b = 0; b < 5; ++b)
    {
      goal.trajectory.points[ind].accelerations[b] = 0.001;
    }
    
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(5.0);
    
//	goal.goal_tolerance.push_back('arm_joint_1', 0.1, 0.01, 0.001);

//	control_msgs::JointTolerance_<std::allocator<void> > jtolerance;
//	jtolerance.name  nm;
//	nm = "arm_joint_1";
//	goal.goal_tolerance.push_back(&jtolerance);

//	control_msgs::JointTolerance jt;
//	jt.name.resize[5];
//	for (size_t q = 0; q < 5; ++q)
//    {
//      jt.name[q] = goal.trajectory.joint_names[q];
//    }
//	jt.position.resize[5];
//	for (size_t l = 0; l < 5; ++l)
//    {
//      jt.position[l] = 0.01;
//    }
//    jt.acceleration.resize[5];
//	for (size_t m = 0; m < 5; ++m)
//    {
//      jt.acceleration[m] = 0.001;
//    }

	control_msgs::JointTolerance jt;
	jt.name = "arm_joint_5";
	jt.position = 0.1;
	jt.velocity = 0.01;
	jt.acceleration = 0.001;
	goal.goal_tolerance.push_back(jt);
	
//	control_msgs::JointTolerance jt2;
//	jt.name = "arm_joint_2";
//	jt.position = 0.1;
//	jt.velocity = 0.01;
//	jt.acceleration = 0.001;
//	goal.goal_tolerance.push_back(jt2);
//	std::cout<< "______________________jt_____________"<<jt;
	
	// 2 trajectory point
    // Positions
	ind += 1;
    goal.trajectory.points[ind].positions.resize(5);
 
goal.trajectory.points[ind].positions[0] = 2.562;
    goal.trajectory.points[ind].positions[1] = 1.05;
    goal.trajectory.points[ind].positions[2] = -2.43;
    goal.trajectory.points[ind].positions[3] = 1.73;
    goal.trajectory.points[ind].positions[4] = 0.12;

    // Velocities
    goal.trajectory.points[ind].velocities.resize(5);
    for (size_t c = 0; c < 5; ++c)
    {
      goal.trajectory.points[ind].velocities[c] = 0.005;
    }
    // Acceleration
    goal.trajectory.points[ind].accelerations.resize(5);
    for (size_t d = 0; d < 5; ++d)
    {
      goal.trajectory.points[ind].accelerations[d] = 0.0;
    }
	// To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(20.0);

	// 3 trajectory point
    // Positions
	ind += 1;
    goal.trajectory.points[ind].positions.resize(5);
 
goal.trajectory.points[ind].positions[0] = 2.94961;
    goal.trajectory.points[ind].positions[1] = 1.352;
    goal.trajectory.points[ind].positions[2] = -2.591;
    goal.trajectory.points[ind].positions[3] = 0.1;
    goal.trajectory.points[ind].positions[4] = 0.12;

    // Velocities
    goal.trajectory.points[ind].velocities.resize(5);
    for (size_t e = 0; e < 5; ++e)
    {
      goal.trajectory.points[ind].velocities[e] = 0.005;
    }
    // Acceleration
    goal.trajectory.points[ind].accelerations.resize(5);
    for (size_t f = 0; f < 5; ++f)
    {
      goal.trajectory.points[ind].accelerations[f] = 0.0;
    }
	// To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(30.0);
    // 4 trajectory point
    // Positions
    ind += 1;
    goal.trajectory.points[ind].positions.resize(5);
 
goal.trajectory.points[ind].positions[0] = 0.11;
    goal.trajectory.points[ind].positions[1] = 0.11;
    goal.trajectory.points[ind].positions[2] = -0.11;
    goal.trajectory.points[ind].positions[3] = 0.11;
    goal.trajectory.points[ind].positions[4] = 0.12;

    // Velocities
    goal.trajectory.points[ind].velocities.resize(5);
    for (size_t j = 0; j < 5; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // Acceleration
    goal.trajectory.points[ind].accelerations.resize(5);
    for (size_t h = 0; h < 5; ++h)
    {
      goal.trajectory.points[ind].accelerations[h] = -0.001;
    }
    
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(40.0);

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
 
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "youbot_arm_action_trajectory");

  RobotArm arm;
  // Start the trajectory
  arm.startTrajectory(arm.armExtensionTrajectory());
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
}
