
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
    ros::init(argc, argv, "constraint_demo");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle node_handle("~");

    moveit_visual_tools::VisualToolsPtr Pvt;


    moveit::planning_interface::MoveGroup right_arm("right_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroup::Plan my_plan;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    planning_scene::PlanningScene planning_scene(kinematic_model);
    planning_scene::PlanningScenePtr planning_scene_ptr(new planning_scene::PlanningScene(kinematic_model) );

    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));


    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    string reference_frame = "base_link";
    string eef_link = "arm_link_5";



    right_arm.setGoalJointTolerance(0.02);
//    right_arm.setGoalPositionTolerance(0.05);
//    right_arm.setGoalOrientationTolerance(0.1);

    right_arm.setPoseReferenceFrame(reference_frame);
    right_arm.setEndEffectorLink(eef_link);

    right_arm.setNumPlanningAttempts(5);
    right_arm.allowReplanning(false);
    right_arm.setPlanningTime(60);

    kinematic_state->setToDefaultValues(joint_model_group, "resting");
//    kinematic_state->setToDefaultValues();

    sleep(3.0);
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

//    vector<double>joint_value3 = right_arm.getCurrentJointValues();
//    ROS_INFO("Current joint values1: %f %f %f %f %f", joint_value3[0], joint_value3[1], joint_value3[2], joint_value3[3], joint_value3[4]);

//    geometry_msgs::PoseStamped curpose3 = right_arm.getCurrentPose("arm_link_5");
//    ROS_INFO("Current pose1\n x: %f y: %f z: %f ox: %f oy: %f oz: %f w: %f", curpose3.pose.position.x, curpose3.pose.position.y, curpose3.pose.position.z, curpose3.pose.orientation.x, curpose3.pose.orientation.y, curpose3.pose.orientation.z, curpose3.pose.orientation.w);

//    vector<double>curRPY3 = right_arm.getCurrentRPY("arm_link_5");
//    ROS_INFO("Current RPY values1: R: %f P: %f Y: %f ", curRPY3[0], curRPY3[1], curRPY3[2]);


ROS_INFO("Reference frame: %s", right_arm.getPlanningFrame().c_str());
ROS_INFO("End_effector_link is: %s", right_arm.getEndEffectorLink().c_str());


/**************************************
*  Dynamic planner
**************************************/
      boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
      planning_interface::PlannerManagerPtr planner_instance;
      std::string planner_plugin_name = "ompl_interface/OMPLPlanner";

//      right_arm.setPlannerId("RRTConnectkConfigDefault");
  //    right_arm.setPlannerId("LBKPIECEkConfigDefault");

      if (!node_handle.getParam("planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
      try
      {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
      }
      try
      {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(kinematic_model, node_handle.getNamespace()))
          ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
      }
      catch(pluginlib::PluginlibException& ex)
      {
        const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0 ; i < classes.size() ; ++i)
          ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                         << "Available plugins: " << ss.str());
      }

      /* Sleep a little to allow time to startup rviz, etc. */
      ros::WallDuration sleep_time(3.0);
      sleep_time.sleep();

      planning_interface::MotionPlanRequest req;

//      req.num_planning_attempts = 10;
      req.allowed_planning_time = 1.5;
//      req.planner_id = "RRTConnectkConfigDefault";
//      req.group_name = ;

      req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = req.workspace_parameters.min_corner.z = -2.0;
      req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.z =  2.0;

cout<< "Pose Goals"<<endl;
      planning_interface::MotionPlanResponse res;
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "base_link";
      pose.pose.position.x = 0.04;
      pose.pose.position.y = 0.1;
      pose.pose.position.z = 0.5;
      pose.pose.orientation.w = 1.0;

      double d[3] = {3.14, 0.01, 0.01};
//      std::vector<double> tolerance_pose(d, d+3);// Goal_constraints/oriention_constraints
//      std::vector<double> tolerance_angle(3, 0.02);// Position_constraints/constraint_region/premitives/dimention

      std::vector<double> tolerance_pose(3, 0.02);
      std::vector<double> tolerance_angle(d, d+3);

      req.group_name = "right_arm";


moveit_msgs::Constraints pose_goal_constraint = kinematic_constraints::constructGoalConstraints("arm_link_5", pose, tolerance_pose, tolerance_angle);
      req.goal_constraints.push_back(pose_goal_constraint);

      // We now construct a planning context that encapsulate the scene,
      // the request and the response. We call the planner using this
      // planning context
      planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene_ptr, req, res.error_code_);
ROS_INFO("Ready to solve.");

      context->solve(res);
      if(res.error_code_.val != res.error_code_.SUCCESS)
      {
          ROS_ERROR("Could not compute plan successfully");
          return 0;
      }

      // Visualize the result
      // ^^^^^^^^^^^^^^^^^^^^
      ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
      moveit_msgs::DisplayTrajectory display_trajectory;

      /* Visualize the trajectory */
      ROS_INFO("Visualizing the trajectory");
      moveit_msgs::MotionPlanResponse response;
      res.getMessage(response);

      display_trajectory.trajectory_start = response.trajectory_start;
      display_trajectory.trajectory.push_back(response.trajectory);
      ROS_INFO("Ready to publish.");

      display_publisher.publish(display_trajectory);

      sleep_time.sleep();

cout<< "Joint Space Goals"<<endl;
      // ^^^^^^^^^^^^^^^^^
      /* First, set the state in the planning scene to the final state of the last plan */
      robot_state::RobotState& robot_state = planning_scene_ptr->getCurrentStateNonConst();
      planning_scene_ptr->setCurrentState(response.trajectory_start);
      const robot_state::JointModelGroup* joint_model_group2 = robot_state.getJointModelGroup("right_arm");
      robot_state.setJointGroupPositions(joint_model_group2, response.trajectory.joint_trajectory.points.back().positions);
 ROS_INFO("Change state to last plan.");
      // Now, setup a joint space goal
      robot_state::RobotState goal_state(kinematic_model);
      std::vector<double> joint_values6(5, 0.0);
      joint_values6[0] = 2.56;
      joint_values6[1] = 1.04;
      joint_values6[2] = -2.43;
      joint_values6[3] = 1.73;
      joint_values6[4] = 0.12;
      goal_state.setJointGroupPositions(joint_model_group2, joint_values6);
      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group2);
      req.goal_constraints.clear();
      req.goal_constraints.push_back(joint_goal);

  // Call the planner and visualize the trajectory
    /* Re-construct the planning context */
    context = planner_instance->getPlanningContext(planning_scene_ptr, req, res.error_code_);
    /* Call the Planner */
    context->solve(res);
    /* Check that the planning was successful */
    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }
    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    res.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);

    /* Now you should see two planned trajectories in series*/
    display_publisher.publish(display_trajectory);

cout<< "Adding Path Constraints"<<endl;
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // Let's add a new pose goal again. This time we will also add a path constraint to the motion.
  /* Let's create a new pose goal */
  pose.pose.position.x = -0.04;
  pose.pose.position.y = -0.2;
  pose.pose.position.z = 0.3;
  moveit_msgs::Constraints pose_goal_2 = kinematic_constraints::constructGoalConstraints("arm_link_5", pose, tolerance_pose, tolerance_angle);
  /* First, set the state in the planning scene to the final state of the last plan */
  robot_state.setJointGroupPositions(joint_model_group2, response.trajectory.joint_trajectory.points.back().positions);

ROS_INFO("Change state to last plan.");

  /* Now, let's try to move to this new pose goal*/
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal_2);


  /* But, let's impose a path constraint on the motion.
     Here, we are asking for the end-effector to stay level*/
  geometry_msgs::QuaternionStamped quaternion;

  quaternion.header.frame_id = "base_link";
  quaternion.quaternion.x = -0.995;
  quaternion.quaternion.y = -0.0295;
  quaternion.quaternion.z = 0.0273;
  quaternion.quaternion.w = 0.087;
  req.path_constraints = kinematic_constraints::constructGoalConstraints("arm_link_5", quaternion);



ROS_INFO("set constraint.");

    // Call the planner and visualize all the plans created so far.
    context = planner_instance->getPlanningContext(planning_scene_ptr, req, res.error_code_);
    context->solve(res);

    /* Check that the planning was successful */
    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }

    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    res.getMessage(response);
    display_trajectory.trajectory.push_back(response.trajectory);
    // Now you should see four planned trajectories in series
    display_publisher.publish(display_trajectory);

    //END_TUTORIAL
    sleep_time.sleep();
    ROS_INFO("Done");
    planner_instance.reset();

//vector<double>joint_value4 = right_arm.getCurrentJointValues();
//ROS_INFO("Current joint values2: %f %f %f %f %f", joint_value4[0], joint_value4[1], joint_value4[2], joint_value4[3], joint_value4[4]);

//geometry_msgs::PoseStamped curpose4 = right_arm.getCurrentPose("arm_link_5");
//ROS_INFO("Current pose2\n x: %f y: %f z: %f ox: %f oy: %f oz: %f w: %f", curpose4.pose.position.x, curpose4.pose.position.y, curpose4.pose.position.z, curpose4.pose.orientation.x, curpose4.pose.orientation.y, curpose4.pose.orientation.z, curpose4.pose.orientation.w);

//vector<double>curRPY4 = right_arm.getCurrentRPY("arm_link_5");
//ROS_INFO("Current RPY values2: R: %f P: %f Y: %f ", curRPY4[0], curRPY4[1], curRPY4[2]);

       right_arm.setNamedTarget("resting");

       right_arm.move();
       sleep(1.5);

//    ros::spinOnce();

//}
ros::waitForShutdown();
//ros::shutdown();
  return 0;

}
