#include <iostream>
#include <assert.h>

#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"
#include "brics_actuator/JointPositions.h"
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

using namespace std;

static const double INIT_POS[] = {2.94961, 1.352, -2.591, 0.1, 0.12}; // Twist fold
//static const double INIT_POS[] = {2.56, 1.05, -2.43, 1.73, 0.12}; // Upstraight
//static const double INIT_POS[] = {0.11, 0.11, -0.11, 0.11, 0.12}; // Home 
static const double Gripper_Init_Pos[] = {0.008, 0.008};

static const string JOINTNAME_PRE = "arm_joint_";
static const string GRIPPER_JOINTNAME_PRE = "gripper_finger_joint_";

static const uint NUM_ARM_JOINTS = 5;
static const uint NUM_GRIPPER_JOINTS = 2;

ros::Publisher armPositionsPublisher;
ros::Publisher gripperPositionPublisher;

vector<control_msgs::FollowJointTrajectoryActionGoal::ConstPtr> trajectories;


void trajectoryCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
  ROS_INFO("callback: Trajectory received");
  //cout << "Msg-Header" << endl << msg->header << endl;
  trajectories.push_back(msg);
}

void moveToInitPos(brics_actuator::JointPositions &command)
{
  std::stringstream jointName;
	
  for (int i = 0; i < NUM_ARM_JOINTS; ++i)
  {
    jointName.str("");
    jointName << JOINTNAME_PRE << (i + 1);

    command.positions[i].joint_uri = jointName.str();
    command.positions[i].value = INIT_POS[i];

    command.positions[i].unit = boost::units::to_string(boost::units::si::radians);
    cout << "Joint " << command.positions[i].joint_uri << " = " << command.positions[i].value << " " << command.positions[i].unit << endl;
  }
  
  cout << command << endl;
  armPositionsPublisher.publish(command); // trully send
  
  cout << "sending command for arm joint init position... and wait" << endl;

  ROS_INFO("End of arm_joint_init-pos");
  // ros::Duration(5).sleep();
}


void moveToGripperInitPos(brics_actuator::JointPositions &command)
{
  std::stringstream gripperJointName;
  
  for( int k = 0; k < NUM_GRIPPER_JOINTS; ++k)
  {
  	gripperJointName.str(""); 
  	(k == 0) ?	gripperJointName<< GRIPPER_JOINTNAME_PRE << "l": (gripperJointName<< GRIPPER_JOINTNAME_PRE << "r");
  		
    command.positions[k].joint_uri = gripperJointName.str();
    command.positions[k].value = Gripper_Init_Pos[k];
    command.positions[k].unit = boost::units::to_string(boost::units::si::meter);
    
    cout << "Joint " << command.positions[k].joint_uri << " = " << command.positions[k].value << " " << command.positions[k].unit << endl;
  }
  
  cout << command << endl;
  gripperPositionPublisher.publish(command); // trully send
  
  cout << "command for gripper joint init position... and wait" << endl;
  
  ROS_INFO("End of gripper_joint_init-pos");
}



int main(int argc, char **argv) {

  ros::init(argc, argv, "youbot_trajectory_controller");
	ros::NodeHandle n;
  uint loop_counter = 0;
  
  brics_actuator::JointPositions arm_command;
  brics_actuator::JointPositions gripper_command;
  
  vector <brics_actuator::JointValue> armJointPositions;
  vector <brics_actuator::JointValue> gripperJointPositions;
  
  armJointPositions.resize(NUM_ARM_JOINTS);
  gripperJointPositions.resize(NUM_GRIPPER_JOINTS);
  
  arm_command.positions = armJointPositions;
  gripper_command.positions = gripperJointPositions;
  
  armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
  
  gripperPositionPublisher = n.advertise<brics_actuator::JointPositions> ("arm_1/gripper_controller/position_command", 1);
  ros::spinOnce();

  ros::Subscriber armTrajectory;
  armTrajectory = n.subscribe("/arm_controller/follow_joint_trajectory/goal", 1, trajectoryCallback);

  ros::Duration(1).sleep();
  ros::spinOnce();

  moveToInitPos(arm_command);
  moveToGripperInitPos(gripper_command);
  //ros::Duration(10).sleep();
  ROS_INFO("Init Pos should be reached");

/*
  while (n.ok())
  {
  //ROS_INFO("yake, in while.");
    if(!trajectories.empty())
    {
      control_msgs::FollowJointTrajectoryActionGoal::ConstPtr act_msg;
      ROS_INFO("new Trajectory");
      act_msg = trajectories.front();
      trajectories.erase(trajectories.begin());
      cout << "Msg-Header" << endl << *act_msg << endl;

      armJointPositions.resize(act_msg->goal.trajectory.joint_names.size());
      for(int i = 0; i<act_msg->goal.trajectory.joint_names.size(); i++)
      {
        armJointPositions[i].joint_uri = act_msg->goal.trajectory.joint_names[i];
        armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
      }
      arm_command.positions = armJointPositions;

      uint pos_count = 0;

      /// prepare first point
      for(int i = 0; i<act_msg->goal.trajectory.joint_names.size(); i++)
      {
        arm_command.positions[i].value = act_msg->goal.trajectory.points[pos_count].positions[i];
      }

      ros::Time start_time = ros::Time::now();

      do
      {
        // send point
        armPositionsPublisher.publish(arm_command);

        // prepare next point
        pos_count++;
        for(int i = 0; i<act_msg->goal.trajectory.joint_names.size();i++)
        {
          arm_command.positions[i].value = act_msg->goal.trajectory.points[pos_count].positions[i];
        }

        //sleep
        ros::Duration((start_time+act_msg->goal.trajectory.points[pos_count].time_from_start)-ros::Time::now()).sleep();

      }
      while(pos_count < act_msg->goal.trajectory.points.size()-1);
      armPositionsPublisher.publish(arm_command);
      ros::Duration(0.5).sleep();

    }																											*/
    
   /* brics_actuator::JointPositions command2;
		vector <brics_actuator::JointValue> armJointPositions2;
		vector <brics_actuator::JointValue> gripperJointPositions2;
		armJointPositions2.resize(NUM_ARM_JOINTS); 
		gripperJointPositions2.resize(NUM_GRIPPER_JOINTS);
		std::stringstream jointName;
		double readValue;
		// ::io::base_unit_info <boost::units::si::angular_velocity>).name();
	try {	
		for (int i = 0; i < NUM_ARM_JOINTS; ++i) {
			cout << "Please type in value for joint " << i + 1 << endl;
			cin >> readValue;
			jointName.str("");
			jointName << "arm_joint_" << (i + 1);
			armJointPositions2[i].joint_uri = jointName.str();
			armJointPositions2[i].value = readValue;
			armJointPositions2[i].unit = boost::units::to_string(boost::units::si::radians);
			cout << "Joint " << armJointPositions2[i].joint_uri << " = " << armJointPositions2[i].value << " " << armJointPositions2[i].unit << endl;
		}
		
		for(int j = 0; j < NUM_GRIPPER_JOINTS; ++j)
		{
			cin >> readValue;
			jointName.str("");
			jointName << "gripper_joint_" << (j+1);
			gripperJointPositions2[j].joint_uri = jointName.str();
			gripperJointPositions2[j].value = readValue;
			gripperJointPositions2[j].unit = boost::units::to_string(boost::units::si::meters);
			cout << "Joint "<< gripperJointPositions2[j].joint_uri << " = "<<gripperJointPositions2[j].value <<" "<< gripperJointPositions2[j].unit<<endl;
			
			armJointPositions2.push_back(gripperJointPositions2[j]);
		}
		
		cout << "sending command ..." << endl;
		
		command2.positions = armJointPositions2;
		
		if(n.ok())
		{
			armPositionsPublisher.publish(command2);
			cout << "--------------------" << endl;
			loop_counter++;
		}
	}catch(...)
	{
		std::cout<<"error in 202"<<std::endl;
	}
		//rate.sleep();
    
    ros::Duration(1).sleep();
    ros::spinOnce();
    ROS_INFO("Loop count %d", loop_counter);
    //armPositionsPublisher.publish(command);
    */
	//}

	ros::Duration(10).sleep();
	return 0;
}

