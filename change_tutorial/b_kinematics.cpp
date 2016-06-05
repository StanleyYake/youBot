/********
 *PlanningScene Interface
*******/
#include <moveit/planning_scene_interface/planning_scene_interface.h>

/********
 *MoveGroup Interface
*******/
#include <moveit/move_group_interface/move_group.h>


#include <moveit/planning_interface/planning_interface.h>

//#include <moveit/planning_interface/planning_request.h>
//#include <moveit/planning_interface/planning_response.h>

#include <moveit/move_group/capability_names.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit/move_group/move_group_context.h>
#include <moveit/move_group/node_name.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

// Robot state publishing

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ObjectColor.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <moveit_visual_tools/visual_tools.h>

// For function constructGoalConstraints()
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
// PI
#include <boost/math/constants/constants.hpp>

#include <iostream>
using namespace std;


//class Move_Visual
//{

//public:
//    Move_Visual()
//    {
//        visual_tools_.reset(new moveit_visual_tools::VisualTools("base_link", "/moveit_help_markers"));

//        visual_tools_->setMuted(false);
//        visual_tools_->setLifetime(20.0);
//    //    visual_tools_->setEEGroupName(grasp_data_.ee_group_);
//    //    visual_tools_->setPlanningGroupName("right_arm");
//        visual_tools_->setFloorToBaseHeight(0.0);

//        geometry_msgs::Pose grasp_pose_to_eef_pose;
//        grasp_pose_to_eef_pose.position.x = 0.0;
//        grasp_pose_to_eef_pose.position.y = 0.0;
//        grasp_pose_to_eef_pose.position.z = 0.0;

//        grasp_pose_to_eef_pose.orientation.x = 0.0;
//        grasp_pose_to_eef_pose.orientation.y = 0.0;
//        grasp_pose_to_eef_pose.orientation.z = 0.0;
//        grasp_pose_to_eef_pose.orientation.w = 1.0;

//        visual_tools_->setGraspPoseToEEFPose(grasp_pose_to_eef_pose);
//        visual_tools_->setAlpha(1.0);
//        visual_tools_->setGlobalScale(1.0);
//        visual_tools_->setBaseFrame("base_link");
//    }

//private:
//    moveit_visual_tools::VisualToolsPtr visual_tools_;

//};



int main(int argc, char **argv)
{
//    ros::init(argc, argv, "constraint_demo", ros::init_options::AnonymousName);
    ros::init(argc, argv, "youbot_moveit_kinematics_demo");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle node_handle("~");

    ros::Publisher robot_state_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>( "display_robot_state", 1 );
    ros::Rate loop_rate(1);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);
    planning_scene::PlanningScenePtr planning_scene_ptr(new planning_scene::PlanningScene(kinematic_model) );

    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

//    kinematic_state->setToDefaultValues(joint_model_group, "resting");
    kinematic_state->setToDefaultValues();

    moveit::planning_interface::MoveGroup right_arm("right_arm");
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

//    string reference_frame = "base_link";
//    string eef_link = "arm_link_5";

//    right_arm.setPlannerId("RRTConnectkConfigDefault");
////    right_arm.setPlannerId("LBKPIECEkConfigDefault");

////    right_arm.setGoalJointTolerance(0.02);
//    right_arm.setGoalPositionTolerance(0.05);
//    right_arm.setGoalOrientationTolerance(0.1);

//    right_arm.setPoseReferenceFrame(reference_frame);
//    right_arm.setEndEffectorLink(eef_link);

//    right_arm.setNumPlanningAttempts(5);
//    right_arm.allowReplanning(false);
//    right_arm.setPlanningTime(60);

//    ROS_INFO("Reference frame: %s", right_arm.getPlanningFrame().c_str());
//    ROS_INFO("End_effector_link is: %s", right_arm.getEndEffectorLink().c_str());

/******************************************************
*
* Two different ways to get joint values.
*
******************************************************/

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    vector<double>joint_value3 = right_arm.getCurrentJointValues();
    ROS_INFO("Current joint values1: %f %f %f %f %f", joint_value3[0], joint_value3[1], joint_value3[2], joint_value3[3], joint_value3[4]);

    geometry_msgs::PoseStamped curpose3 = right_arm.getCurrentPose("arm_link_5");
    ROS_INFO("Current pose1\n x: %f y: %f z: %f ox: %f oy: %f oz: %f w: %f", curpose3.pose.position.x, curpose3.pose.position.y, curpose3.pose.position.z, curpose3.pose.orientation.x, curpose3.pose.orientation.y, curpose3.pose.orientation.z, curpose3.pose.orientation.w);

    vector<double>curRPY3 = right_arm.getCurrentRPY("arm_link_5");
    ROS_INFO("Current RPY values1: R: %f P: %f Y: %f ", curRPY3[0], curRPY3[1], curRPY3[2]);

/*************************************
*
* Enforce joint limits.
*
**************************************/

    /* Set one joint in the right arm outside its joint limit */
    joint_values[0] = -0.5;
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

    /* Check whether any joint is outside its joint limits */
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    /* Enforce the joint limits for this state and check again*/
    kinematic_state->enforceBounds();
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

/***************************************************
*
* Get the Translation and Rotation of the eef_link.
*
****************************************************/

    kinematic_state->setToRandomPositions(joint_model_group);
//    kinematic_state->setVariablePositions();
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("arm_link_5");

    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation());

/***************************************************
*
* Get the Jacobian of the eef_link.
*
****************************************************/

    // Inverse Kinematics
    // ^^^^^^^^^^^^^^^^^^
    // We can now solve inverse kinematics (IK) for the right arm
    // To solve IK, we will need the following:
    //  * The desired pose of the end-effector (by default, this is the last link in the "right_arm" chain): end_effector_state that we computed in the step above.
    //  * The number of attempts to be made at solving IK: 10
    //  * The timeout for each attempt: 0.1 s
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);


    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      for(std::size_t i=0; i < joint_names.size(); ++i)
      {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
    }
    else
    {
      ROS_INFO("Did not find IK solution");
    }

    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                 reference_point_position,
                                 jacobian);
    ROS_INFO_STREAM("Jacobian: \n" << jacobian);

/***************************************************
*
* Print the robot state.
*
****************************************************/

    for (int cnt=0; cnt<5 && ros::ok(); cnt++)
      {
        kinematic_state->setToRandomPositions(joint_model_group);

        /* get a robot state message describing the pose in kinematic_state */
        moveit_msgs::DisplayRobotState msg;
        robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);

        /* send the message to the RobotState display */
        robot_state_publisher.publish( msg );

        /* let ROS send the message, then wait a while */
        ros::spinOnce();
        loop_rate.sleep();
      }

/***************************************************
*
* Position end effector at specific locations.
*
****************************************************/

      /* Find the default pose for the end effector */
      kinematic_state->setToDefaultValues();

      const Eigen::Affine3d end_effector_default_pose = kinematic_state->getGlobalLinkTransform("arm_link_5");

      const double PI = boost::math::constants::pi<double>();
      const double RADIUS = 0.15;
      int fail = 0;
      int succss = 0;

      for (double angle=0; angle<=2*PI && ros::ok(); angle+=2*PI/20)
      {

        /* calculate a position for the end effector */
        Eigen::Affine3d end_effector_pose =
          Eigen::Translation3d(RADIUS * cos(angle), RADIUS * sin(angle), 0.0) * end_effector_default_pose;

        ROS_INFO_STREAM("End effector translated position with angle: "<<angle<<"\n" << end_effector_pose.translation());

        /* use IK to get joint angles satisfying the calculated position */
        bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_pose, 1, 0.1);



        if (!found_ik)
        {
            ++fail;
          ROS_INFO_STREAM("Could not solve IK for pose\n" << end_effector_pose.translation());
          continue;
        }
        ++succss;

        /* get a robot state message describing the pose in kinematic_state */
        moveit_msgs::DisplayRobotState msg;
        robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);

        /* send the message to the RobotState display */
        robot_state_publisher.publish( msg );

        /* let ROS send the message, then wait a while */
        ros::spinOnce();
        loop_rate.sleep();
      }
      ROS_INFO("success %d, and fail %d in total: %d", succss, fail, (succss + fail) );

      ros::shutdown();
      return 0;
    }



