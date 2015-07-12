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

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ObjectColor.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <moveit_visual_tools/visual_tools.h>

#include <moveit/kinematic_constraints/utils.h> // For function constructGoalConstraints()
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/robot_model/robot_model.h>

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
    ros::init(argc, argv, "youbot_moveit_planningScene_about_constraint_check_contact_state_pub");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle node_handle("~");
    ros::Publisher robot_state_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>( "display_robot_state", 1 );
    ros::Rate loop_rate(1);

    moveit_visual_tools::VisualToolsPtr Pvt;


    moveit::planning_interface::MoveGroup right_arm("right_arm");
// Advised to use the below method.

//    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//    planning_scene::PlanningScene planning_scene(kinematic_model);
//    planning_scene::PlanningScenePtr planning_scene_ptr(new planning_scene::PlanningScene(kinematic_model) );

    planning_scene_monitor::PlanningSceneMonitor planning_scene_monitr("robot_description");
    robot_model::RobotModelConstPtr kinematic_model = planning_scene_monitr.getRobotModel();
    planning_scene::PlanningScenePtr planning_scene_ptr = planning_scene_monitr.getPlanningScene();


    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));


    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    string reference_frame = "base_link";
    string eef_link = "arm_link_5";

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

//    kinematic_state->setToDefaultValues(joint_model_group, "resting");

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


    kinematic_state->setToDefaultValues();

    sleep(3.0);
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

/**************************************************/
//  Collision Checking
/**************************************************/

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    planning_scene_ptr->checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1 Self-conlllision check: Current state is "
                    <<(collision_result.collision ? "IN" : "not in")
                    <<" self collision");

    robot_state::RobotState& current_state = planning_scene_ptr->getCurrentStateNonConst();
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene_ptr->checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 2 change and self check: Current state is "
                    <<(collision_result.collision ? "IN" : "not in")
                    <<" self collision");

    collision_request.group_name = "right_arm";
    current_state.setToRandomPositions();

    collision_result.clear();
    planning_scene_ptr->checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 3 check Group Collision: Current state is "
                    <<(collision_result.collision ? "IN" : "not in")
                    <<" self collision");

/*************************************************
*
*  Getting contact information
*
**************************************************/

    std::vector<double> joint_values_col;
//    const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("right_arm");
    current_state.copyJointGroupPositions(joint_model_group, joint_values_col);
    joint_values_col[0] = 2.95;
    joint_values_col[1] = 1.35;
    joint_values_col[2] = -5.00;
    joint_values_col[3] = 0.03;
    joint_values_col[4] = 0.12;

    current_state.setJointGroupPositions(joint_model_group, joint_values_col);

    vector<const double*> state_values;


    const vector<const moveit::core::JointModel*> jm = joint_model_group->getJointModels();


    for( vector<const moveit::core::JointModel*>::const_iterator it0 = jm.begin(); it0 != jm.end(); ++it0)
    {
        state_values.push_back( current_state.getJointPositions(*it0));
    }
//    state_values[0] = current_state.getJointPositions(jm[0]);
//    state_values[1] = current_state.getJointPositions(jm[1]);
//    state_values[2] = current_state.getJointPositions(jm[2]);
//    state_values[3] = current_state.getJointPositions(jm[3]);
//    state_values[4] = current_state.getJointPositions(jm[4]);

    ROS_INFO("Current joint values: %f %f %f %f %f", *state_values[0], *state_values[1],* state_values[2], *state_values[3], *state_values[4]);


    ROS_INFO_STREAM("Test 4 Getting Contact info: Current state is "
                    <<(current_state.satisfiesBounds(joint_model_group) ? "valid" : "NOT VALID"));

    collision_request.contacts = true;

    collision_request.max_contacts = 1000;

    collision_result.clear();
    planning_scene_ptr->checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 4 Getting Contact info: Current state is "
                    << (collision_result.collision ? "IN" : "not in")
                    << " self collision");

    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for(it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
    {
      ROS_INFO("Contact between: %s and %s",
               it->first.first.c_str(),
               it->first.second.c_str());
    }

/*************************************************
*
* Change the Allowed Collision Matrix
*
**************************************************/

    collision_detection::AllowedCollisionMatrix acm = planning_scene_ptr->getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = planning_scene_ptr->getCurrentState();

    collision_detection::CollisionResult::ContactMap::const_iterator it2;

    for(it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
    {
        acm.setEntry(it2->first.first, it2->first.second, true);

    }

    collision_result.clear();
    planning_scene_ptr->checkSelfCollision(collision_request, collision_result, copied_state, acm);

    ROS_INFO_STREAM("Test 5 modify the Allowed Collision Matrix: Current state is "
                    << (collision_result.collision ? "IN" : "not in")
                    << " self collision");
/*************************************************
*
* Full check (including environment)
*
**************************************************/
    collision_result.clear();
    planning_scene_ptr->checkCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 6 Full check with the environment : Current state is "
                    << (collision_result.collision ? "IN" : "not in")
                    << " self collision");

/***************************************************/
//  Check constraint
/**************************************************/

       std::string end_effctor_name = joint_model_group->getLinkModelNames().back();

//       cout<< "end_effctor_name : "<< end_effctor_name<<endl;

        geometry_msgs::PoseStamped desired_pose ;

        desired_pose.header.frame_id = "base_link";
        desired_pose.pose.orientation.x = -0.995;
        desired_pose.pose.orientation.y = -0.0295;
        desired_pose.pose.orientation.z = 0.02736;
        desired_pose.pose.orientation.w  = 0.08717;

        desired_pose.pose.position.x = -0.067;
        desired_pose.pose.position.y = 0.2634;
        desired_pose.pose.position.z = 0.1460;

        moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(end_effctor_name, desired_pose);

        copied_state.setToRandomPositions();
        copied_state.update();
        bool constrained = planning_scene_ptr->isStateConstrained(copied_state, goal_constraint);

        ROS_INFO_STREAM("Test 7 checking constraint: current state is "
                        << (constrained ? "CONSTRAINED!!" : "not constrained"));

/**************************************************
*
*  Efficient check constraint
*
**************************************************/

       kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(kinematic_model);
       kinematic_constraint_set.add(goal_constraint, planning_scene_ptr->getTransforms());

       bool constraint_result2 = planning_scene_ptr->isStateConstrained(planning_scene_ptr->getCurrentState(), kinematic_constraint_set);

       ROS_INFO_STREAM("Test 8 checking constraint_set: current state is "
                       << (constraint_result2 ? "CONSTRAINED!!" : "not constrained"));
/**************************************************
*
*  Most efficient check constraint
*
**************************************************/

       kinematic_constraints::ConstraintEvaluationResult constraint_eval_result = kinematic_constraint_set.decide(copied_state);
       ROS_INFO_STREAM("Test 9 Most efficient checking constraint_set: current state is "
                       << (constraint_eval_result.satisfied ? "CONSTRAINED!!" : "not constrained"));

//ros::waitForShutdown();
ros::shutdown();
  return 0;

}
