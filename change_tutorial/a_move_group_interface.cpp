
#include <moveit/planning_scene_interface/planning_scene_interface.h>


#include <moveit/move_group_interface/move_group.h>


#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>

#include <moveit/move_group/capability_names.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit/move_group/move_group_context.h>
#include <moveit/move_group/node_name.h>


#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ObjectColor.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <moveit_visual_tools/visual_tools.h>

#include <iostream>
using namespace std;


//class Move_Visual
//{
//private:
//    moveit_visual_tools::VisualToolsPtr visual_tools_;
//public:
//    Move_Visual(moveit_visual_tools::VisualToolsPtr &);

//};


//Move_Visual::Move_Visual(moveit_visual_tools::VisualToolsPtr& vi_tools)
//{
//    vi_tools.reset(new moveit_visual_tools::VisualTools("base_link", "/moveit_help_markers"));
//    vi_tools->setMuted(false);
//    vi_tools->setLifetime(20.0);
//    vi_tools->setBaseFrame("base_link");
////    vi_tools->setPlanningSceneMonitor();
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_move_group_interface", ros::init_options::AnonymousName);
//    ros::init(argc, argv, "constraint_demo");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;


    moveit::planning_interface::MoveGroup right_arm("right_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroup::Plan my_plan;
    moveit_msgs::DisplayTrajectory display_trajectory;

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    string reference_frame = "base_link";
    string eef_link = "arm_link_5";

    right_arm.setPlannerId("RRTConnectkConfigDefault");
//    right_arm.setPlannerId("LBKPIECEkConfigDefault");

    right_arm.setGoalJointTolerance(0.01);
    right_arm.setGoalPositionTolerance(0.01);
    right_arm.setGoalOrientationTolerance(0.01);

    right_arm.setPoseReferenceFrame(reference_frame);
    right_arm.setEndEffectorLink(eef_link);

    right_arm.setNumPlanningAttempts(5);
    right_arm.allowReplanning(false);
    right_arm.setPlanningTime(60);


//    while(nh.ok()){
//    ros::Publisher scene_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene_yake",1);

//// Create a publisher for visualizing plans in Rviz.
//    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

//    moveit_msgs::DisplayTrajectory display_trajectory;

ROS_INFO("Reference frame: %s", right_arm.getPlanningFrame().c_str());
ROS_INFO("End_effector_link is: %s", right_arm.getEndEffectorLink().c_str());
right_arm.setNamedTarget("resting");
right_arm.move();
sleep(2.0);


/**************************************************/
//  Pose Move
/**************************************************/

//    right_arm.setNamedTarget("straight_forward");
    geometry_msgs::PoseStamped target_pose0;
    target_pose0.header.frame_id = "base_link";
    target_pose0.pose.position.x = 0.038;
    target_pose0.pose.position.y = 0.006;
    target_pose0.pose.position.z = 0.534;

    target_pose0.pose.orientation.x = 0.016;
    target_pose0.pose.orientation.y = 0.006;
    target_pose0.pose.orientation.z = -1.0;
    target_pose0.pose.orientation.w = 0.034;

//    right_arm.setStartStateToCurrentState();
    right_arm.setPoseTarget(target_pose0, "arm_link_5");

    bool success = right_arm.plan(my_plan);

    ROS_INFO("Test 1 pose Move plan %s", success ? "Success" : "Failed");
    sleep(1.5);

    ROS_INFO("visualizing plan 1 (again)");

    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    sleep(3.0);

    right_arm.move();
/**************************************************/
//  Joint space Move

/**************************************************/
    std::vector<double>group_variable_values;
    right_arm.getCurrentState()->copyJointGroupPositions(right_arm.getCurrentState()->getRobotModel()->getJointModelGroup(right_arm.getName()),group_variable_values);

    group_variable_values[0] = 2.0;
    right_arm.setJointValueTarget(group_variable_values);
    success = right_arm.plan(my_plan);
    ROS_INFO("Visualizing plan2(Joint space goal) %s", success ? "Success" : "Failed");
    sleep(2.0);

/**************************************************/
//  Constraint Move
/**************************************************/

       moveit_msgs::OrientationConstraint ocm;
       ocm.header = right_arm.getCurrentPose(right_arm.getEndEffectorLink()).header;

//        ocm.header.stamp = ros::Time::now();
       ocm.link_name = "arm_link_5";
//       ocm.link_name = right_arm.getEndEffectorLink();
//       ROS_INFO("link name: %s", ocm.link_name.c_str());

       ocm.header.frame_id = "base_link";
//       ocm.header.frame_id = right_arm.getPlanningFrame();
//       ROS_INFO("frame should be base_link ?= %s", ocm.header.frame_id.c_str());

//       ocm.orientation.x = 0.706655;
//       ocm.orientation.y = 0.210861;
//       ocm.orientation.z = -0.674017;

//       ocm.orientation.w = 1;

//          ocm.orientation.x = -0.000541;
//          ocm.orientation.y = -0.001871;
//          ocm.orientation.z = 0.593788;

          ocm.orientation.w = 1.0;


//       ocm.absolute_x_axis_tolerance = 2 * M_PI;
       ocm.absolute_x_axis_tolerance = 3.14;
       ocm.absolute_y_axis_tolerance = 0.5;
       ocm.absolute_z_axis_tolerance = 0.5;
       ocm.weight = 1.0;

       moveit_msgs::Constraints test_constraints;
       test_constraints.orientation_constraints.push_back(ocm);
       right_arm.setPathConstraints(test_constraints);

//ROS_INFO("before startstate!");
// This can be used as setStartStateToCurrentState
//       robot_state::RobotState start_state2(*right_arm.getCurrentState());
//       geometry_msgs::Pose start_pose2;

/****************************when you set the start state is not the current state, use setFromIK*****************************/
//       start_pose2.position.x = right_arm.getCurrentPose().pose.position.x;
//       start_pose2.position.y = right_arm.getCurrentPose().pose.position.y;
//       start_pose2.position.z = right_arm.getCurrentPose().pose.position.z;

//       start_pose2.orientation.x = right_arm.getCurrentPose().pose.orientation.x;
//       start_pose2.orientation.y = right_arm.getCurrentPose().pose.orientation.y;
//       start_pose2.orientation.z = right_arm.getCurrentPose().pose.orientation.z;
//       start_pose2.orientation.w = right_arm.getCurrentPose().pose.orientation.w;

//       start_pose2.position.x = 0.2;
//       start_pose2.position.y = 0.1;
//       start_pose2.position.z = 0.16;

//       start_pose2.orientation.x = 0.943853;
//       start_pose2.orientation.y = 0.121885;
//       start_pose2.orientation.z = -0.305825;
//       start_pose2.orientation.w = -0.0275191;


//       const robot_state::JointModelGroup *joint_model_group = start_state2.getJointModelGroup(right_arm.getName());

//       start_state2.setFromIK(joint_model_group, start_pose2);
//       right_arm.setStartState(start_state2);



//       target_pose1.pose.position.x = target_pose1.pose.position.x + 0.4;
//       right_arm.setPoseTarget(target_pose1,right_arm.getEndEffectorLink());


//    geometry_msgs::PoseStamped target_pose1;
//    target_pose1.header.frame_id = "base_link";
//    target_pose1.pose.position.x = 0.2;
//    target_pose1.pose.position.y = 0.00971348;
//    target_pose1.pose.position.z = 0.167754;

//    target_pose1.pose.orientation.x = 0.943853;
//    target_pose1.pose.orientation.y = 0.121885;
//    target_pose1.pose.orientation.z = -0.305825;
//    target_pose1.pose.orientation.w = -0.0275191;

//    right_arm.setStartStateToCurrentState();
    right_arm.setStartState(*right_arm.getCurrentState());

ROS_INFO("after startstate!");
right_arm.setPoseTarget(target_pose0, "arm_link_5");

 ROS_INFO("after posetarget and before plan!");

        success = right_arm.plan(my_plan);
//       right_arm.execute(my_plan);
//        right_arm.move();
       sleep(1.5);

       right_arm.clearPathConstraints();
ROS_INFO("Visualizing plan 3 (constraints) %s",success?"SUCCESS!":"FAILED!");

//       right_arm.setNamedTarget("resting");

//       right_arm.move();
//       sleep(1.5);

//       vector<double>joint_value3 = right_arm.getCurrentJointValues();
//       ROS_INFO("Current joint values: %f %f %f %f %f", joint_value3[0], joint_value3[1], joint_value3[2], joint_value3[3], joint_value3[4]);

//       geometry_msgs::PoseStamped curpose3 = right_arm.getCurrentPose("arm_link_5");
//       ROS_INFO("Current pose\n x: %f y: %f z: %f ox: %f oy: %f oz: %f w: %f", curpose3.pose.position.x, curpose3.pose.position.y, curpose3.pose.position.z, curpose3.pose.orientation.x, curpose3.pose.orientation.y, curpose3.pose.orientation.z, curpose3.pose.orientation.w);

//       vector<double>curRPY3 = right_arm.getCurrentRPY("arm_link_5");
//       ROS_INFO("Current RPY values: R: %f P: %f Y: %f ", curRPY3[0], curRPY3[1], curRPY3[2]);

/***************************************************/
//  Cartesian Move
/**************************************************/

    std::vector<geometry_msgs::Pose> waypoints;

//    geometry_msgs::Pose target_pose2 = target_pose0.pose;
    geometry_msgs::Pose target_pose2 ;
    target_pose2.position.x = 0.04;
    target_pose2.position.y = 0.1;
    target_pose2.position.z = 0.5;
    waypoints.push_back(target_pose2);

    target_pose2.position.x = 0.057;
    target_pose2.position.y = 0.295;
    target_pose2.position.z = 0.334;
    waypoints.push_back(target_pose2);

    target_pose2.position.x -= 0.17;
    target_pose2.position.y -= 0.195;
    target_pose2.position.z += 0.166;
    waypoints.push_back(target_pose2);

    moveit_msgs::RobotTrajectory traj;

    double fraction = right_arm.computeCartesianPath(waypoints, 0.01, 0.0, traj);

    ROS_INFO("plan (cartesian move) (%.2f%% achieved)", fraction * 100.0);

//    if (fraction == 1.0)
////        if (fraction >= 0.6)
//        {
//            my_plan.trajectory_ = traj;

//            right_arm.execute(my_plan);
//        }
//        else
//        {
//            ROS_INFO("cartesian FAILED!");
//        }

    sleep(10);
/***************************************************/
//  Adding/Removing object
/**************************************************/
moveit_msgs::CollisionObject collision_object;
collision_object.header.frame_id = right_arm.getPlanningFrame();
collision_object.id = "box1";

shape_msgs::SolidPrimitive primitive;
primitive.type = primitive.BOX;

primitive.dimensions.resize(3);
primitive.dimensions[0] = 0.1;
primitive.dimensions[1] = 0.4;
primitive.dimensions[2] = 0.2;

geometry_msgs::Pose box_pose;
box_pose.orientation.w = 1.0;
box_pose.position.x = 0.09;
box_pose.position.y = -0.3;
box_pose.position.z = 0.33;

collision_object.primitives.push_back(primitive);
collision_object.primitive_poses.push_back(box_pose);
collision_object.operation = collision_object.ADD;

std::vector<moveit_msgs::CollisionObject>collision_objects;
collision_objects.push_back(collision_object);
ROS_INFO("Add an object to the world");

planning_scene_interface.addCollisionObjects(collision_objects);

sleep(2.0);

right_arm.setPlanningTime(10.0);

right_arm.setStartState(*right_arm.getCurrentState());

geometry_msgs::PoseStamped target_pose3;
target_pose3.header.frame_id = "base_link";
target_pose3.pose.position.x = 0.025;
target_pose3.pose.position.y = -0.0005;
target_pose3.pose.position.z = 0.341;

target_pose3.pose.orientation.x = -0.0035;
target_pose3.pose.orientation.y = 0.1319;
target_pose3.pose.orientation.z = -0.304;
target_pose3.pose.orientation.w = 0.943;

right_arm.setPoseTarget(target_pose3);

success = right_arm.plan(my_plan);
ROS_INFO("Vistualizing plan 4(pose goal around the box) %s", success ? "Success" : "Failed");
right_arm.move();
sleep(2.0);

right_arm.attachObject(collision_object.id);
ROS_INFO("Attach object to the robot.");
sleep(2.0);

right_arm.detachObject(collision_object.id);
ROS_INFO("Detach object to the robot.");

sleep(2.0);

std::vector<std::string>object_ids;
object_ids.push_back(collision_object.id);

//planning_scene_interface.removeCollisionObjects(object_ids);
//ROS_INFO("Remove the object from the world.");
sleep(2.0);
//    ros::spinOnce();

//}'
right_arm.setNamedTarget("resting");
right_arm.move();
sleep(2.0);

ros::waitForShutdown();

}
