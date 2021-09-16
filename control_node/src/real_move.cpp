#include <ros/ros.h>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include<moveit_msgs/DisplayRobotState.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<moveit_msgs/AttachedCollisionObject.h>
#include<moveit_msgs/CollisionObject.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <ur_msgs/SetIO.h>
#include <geometry_msgs/Pose.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_with_line");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient io_client = node_handle.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");
    ur_msgs::SetIO io_service;
    //open the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);

    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::string end_effector_link = arm.getEndEffectorLink();
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);
    cout << "end_effector_link = " << end_effector_link << endl; 

    arm.setPlanningTime(10.0);

    //set obstacle
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      sleep_t.sleep();
    }
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    moveit_msgs::CollisionObject collision_object, collision_object_1, collision_object_2, collision_object_3, collision_object_4, collision_object_5;
    collision_object.header.frame_id = arm.getPlanningFrame();
    collision_object_1.header.frame_id = arm.getPlanningFrame();
    collision_object_2.header.frame_id = arm.getPlanningFrame();
    collision_object_3.header.frame_id = arm.getPlanningFrame();
    collision_object_4.header.frame_id = arm.getPlanningFrame();
    collision_object_5.header.frame_id = arm.getPlanningFrame();
    // The id of the object is used to identify it.
    collision_object.id = "box";
    collision_object_1.id = "box1";
    collision_object_2.id = "box2";
    collision_object_3.id = "box3";
    collision_object_4.id = "box4";
    collision_object_5.id = "box5";
    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive, primitive_1, primitive_2, primitive_3, primitive_4;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.2;
    primitive.dimensions[1] = 0.01;
    primitive.dimensions[2] = 0.9;

    primitive_1.type = primitive_1.BOX;
    primitive_1.dimensions.resize(3);
    primitive_1.dimensions[0] = 1.2;
    primitive_1.dimensions[1] = 0.8;
    primitive_1.dimensions[2] = 0.01;

    primitive_2.type = primitive_2.BOX;
    primitive_2.dimensions.resize(3);
    primitive_2.dimensions[0] = 0.2;
    primitive_2.dimensions[1] = 0.2;
    primitive_2.dimensions[2] = 0.01;

    primitive_3.type = primitive_3.BOX;
    primitive_3.dimensions.resize(3);
    primitive_3.dimensions[0] = 0.6;
    primitive_3.dimensions[1] = 0.8;
    primitive_3.dimensions[2] = 0.01;

    primitive_4.type = primitive_4.BOX;
    primitive_4.dimensions.resize(3);
    primitive_4.dimensions[0] = 0.01;
    primitive_4.dimensions[1] = 0.8;
    primitive_4.dimensions[2] = 0.9;
    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose, box_pose_1, box_pose_2, box_pose_3, box_pose_4, box_pose_5;
    //left
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.4;
    box_pose.position.y = -0.4;
    box_pose.position.z = 0.45;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    //collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    //collision_objects.push_back(collision_object);
    //bottom
    box_pose_1.orientation.w = 1.0;
    box_pose_1.position.x = -0.4;
    box_pose_1.position.y = 0;
    box_pose_1.position.z = 0;
    collision_object_1.primitives.push_back(primitive_1);
    collision_object_1.primitive_poses.push_back(box_pose_1);
    collision_object_1.operation = collision_object_1.ADD;
    collision_objects.push_back(collision_object_1);
    //right
    box_pose_2.orientation.w = 1.0;
    box_pose_2.position.x = -0.4;
    box_pose_2.position.y = 0.4;
    box_pose_2.position.z = 0.45;
    collision_object_2.primitives.push_back(primitive);
    collision_object_2.primitive_poses.push_back(box_pose_2);
    //collision_object_2.operation = collision_object_2.ADD;
    //collision_objects.push_back(collision_object_2);
    //up
    box_pose_4.orientation.w = 1.0;
    box_pose_4.position.x = -0.1;
    box_pose_4.position.y = 0;
    box_pose_4.position.z = 0.9;
    collision_object_4.primitives.push_back(primitive_3);
    collision_object_4.primitive_poses.push_back(box_pose_4);
    //collision_object_4.operation = collision_object_4.ADD;
    //collision_objects.push_back(collision_object_4);
    //back
    box_pose_5.orientation.w = 1.0;
    box_pose_5.position.x = 0.2;
    box_pose_5.position.y = 0;
    box_pose_5.position.z = 0.45;
    collision_object_5.primitives.push_back(primitive_4);
    collision_object_5.primitive_poses.push_back(box_pose_5);
    //collision_object_5.operation = collision_object_5.ADD;
    //collision_objects.push_back(collision_object_5);

    planning_scene_interface.addCollisionObjects(collision_objects);

    arm.allowReplanning(true);
    //arm.setMaxAccelerationScalingFactor(0.8);
    //arm.setMaxVelocityScalingFactor(0.8);

    const robot_state::JointModelGroup* joint_model_group_1 = arm.getCurrentState()->getJointModelGroup("manipulator"); 
    
    moveit::core::RobotStatePtr current_state_1 = arm.getCurrentState();
    std::vector<double> joint_group_positions_camera;
    current_state_1->copyJointGroupPositions(joint_model_group_1, joint_group_positions_camera);
    joint_group_positions_camera[0] = 2.737;
    joint_group_positions_camera[1] = -1.630;
    joint_group_positions_camera[2] = 2.519;
    joint_group_positions_camera[3] = -2.474;
    joint_group_positions_camera[4] = -1.568;
    joint_group_positions_camera[5] = 5.842;
    arm.setJointValueTarget(joint_group_positions_camera);
    arm.move();
    sleep(4);

    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = -0.00892589;
    target_pose.orientation.y = 0.9999;
    target_pose.orientation.z = 0.00340034;
    target_pose.orientation.w = -0.0105761;

    target_pose.position.x = -0.8;
    target_pose.position.y = 0.125;
    target_pose.position.z = 0.2474;

    arm.setPoseTarget(target_pose);
    arm.move();
/*
//path constrain
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = end_effector_link;
    ocm.header.frame_id = "base_link";
    ocm.orientation.x = -0.00892589;
    ocm.orientation.y = 0.9999;
    ocm.orientation.z = 0.00340034;
    ocm.orientation.w = -0.0105761;
    ocm.absolute_x_axis_tolerance = 0.785;
    ocm.absolute_y_axis_tolerance = 0.785;
    ocm.absolute_z_axis_tolerance = 0.785;
    ocm.weight = 1.0;

    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    arm.setPathConstraints(test_constraints);
*/
    geometry_msgs::Pose target_pose1 = arm.getCurrentPose().pose;
    target_pose1.position.x -= 0.028;
    arm.setPoseTarget(target_pose1);
    arm.move();

//螺栓枪开启
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 0; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);
    sleep(1);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 0; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    sleep(10);

    geometry_msgs::Pose target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x += 0.160;
    arm.setPoseTarget(target_pose2);
    arm.move();

//移动到爪夹
    const robot_state::JointModelGroup* joint_model_group = arm.getCurrentState()->getJointModelGroup("manipulator"); 
    
    moveit::core::RobotStatePtr current_state = arm.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 2.51;  // radians 144.84
    joint_group_positions[1] = -1.217;  // radians  -69.22
    joint_group_positions[2] = 2.094;  // radians   120.63
    joint_group_positions[3] = -2.436;  // radians   -143.25
    joint_group_positions[4] = -1.554;  // radians    -91.76
    joint_group_positions[5] = 4.084;  // radians
    arm.setJointValueTarget(joint_group_positions);
    arm.move();

    geometry_msgs::Pose target_pose3_1 = arm.getCurrentPose().pose;
    target_pose3_1.position.x -= 0.223;
    arm.setPoseTarget(target_pose3_1);
    arm.move();

//close the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    sleep(1);

    geometry_msgs::Pose target_pose3_2 = arm.getCurrentPose().pose;
    target_pose3_2.position.x += 0.223;
    arm.setPoseTarget(target_pose3_2);
    arm.move();
//open the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);
//p2
    geometry_msgs::Pose target_pose4;
    target_pose4.orientation.x = -0.00892589;
    target_pose4.orientation.y = 0.9999;
    target_pose4.orientation.z = 0.00340034;
    target_pose4.orientation.w = -0.0105761;

    target_pose4.position.x = -0.8;
    target_pose4.position.y = 0.0934;
    target_pose4.position.z = 0.1315;

    arm.setPoseTarget(target_pose4);
    arm.move();

    geometry_msgs::Pose target_pose_p2_1 = arm.getCurrentPose().pose;
    target_pose_p2_1.position.x -= 0.0285;
    arm.setPoseTarget(target_pose_p2_1);
    arm.move();
//螺栓枪开启
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 0; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);
    sleep(1);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 0; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    sleep(10);

    geometry_msgs::Pose target_pose_p2_2 = arm.getCurrentPose().pose;
    target_pose_p2_2.position.x += 0.160;
    arm.setPoseTarget(target_pose_p2_2);
    arm.move();

    arm.setJointValueTarget(joint_group_positions);
    arm.move();

    geometry_msgs::Pose target_pose10_1 = arm.getCurrentPose().pose;
    target_pose10_1.position.x -= 0.223;
    arm.setPoseTarget(target_pose10_1);
    arm.move();
//close the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    sleep(1);

    geometry_msgs::Pose target_pose10_2 = arm.getCurrentPose().pose;
    target_pose10_2.position.x += 0.223;
    arm.setPoseTarget(target_pose10_2);
    arm.move();
//open the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);

//p3
    geometry_msgs::Pose target_pose5;
    target_pose5.orientation.x = -0.00892589;
    target_pose5.orientation.y = 0.9999;
    target_pose5.orientation.z = 0.00340034;
    target_pose5.orientation.w = -0.0105761;

    target_pose5.position.x = -0.8;
    target_pose5.position.y = -0.0234;
    target_pose5.position.z = 0.1004;

    arm.setPoseTarget(target_pose5);
    arm.move();

    geometry_msgs::Pose target_pose_p3_1 = arm.getCurrentPose().pose;
    target_pose_p3_1.position.x -= 0.0307;
    arm.setPoseTarget(target_pose_p3_1);
    arm.move();
//螺栓枪开启
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 0; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);
    sleep(1);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 0; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    sleep(10);

    geometry_msgs::Pose target_pose_p3_2 = arm.getCurrentPose().pose;
    target_pose_p3_2.position.x += 0.160;
    arm.setPoseTarget(target_pose_p3_2);
    arm.move();

    arm.setJointValueTarget(joint_group_positions);
    arm.move();

    geometry_msgs::Pose target_pose11_1 = arm.getCurrentPose().pose;
    target_pose11_1.position.x -= 0.223;
    arm.setPoseTarget(target_pose11_1);
    arm.move();

//close the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    sleep(1);

    geometry_msgs::Pose target_pose11_2 = arm.getCurrentPose().pose;
    target_pose11_2.position.x += 0.223;
    arm.setPoseTarget(target_pose11_2);
    arm.move();
//open the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);

//p4
    geometry_msgs::Pose target_pose6;
    target_pose6.orientation.x = -0.00892589;
    target_pose6.orientation.y = 0.9999;
    target_pose6.orientation.z = 0.00340034;
    target_pose6.orientation.w = -0.0105761;

    target_pose6.position.x = -0.8;
    target_pose6.position.y = -0.1088;
    target_pose6.position.z = 0.1857;

    arm.setPoseTarget(target_pose6);
    arm.move();

    geometry_msgs::Pose target_pose_p4_1 = arm.getCurrentPose().pose;
    target_pose_p4_1.position.x -= 0.0324;
    arm.setPoseTarget(target_pose_p4_1);
    arm.move();
//螺栓枪开启
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 0; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);
    sleep(1);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 0; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    sleep(10);

    geometry_msgs::Pose target_pose_p4_2 = arm.getCurrentPose().pose;
    target_pose_p4_2.position.x += 0.160;
    arm.setPoseTarget(target_pose_p4_2);
    arm.move();

    arm.setJointValueTarget(joint_group_positions);
    arm.move();

    geometry_msgs::Pose target_pose12_1 = arm.getCurrentPose().pose;
    target_pose12_1.position.x -= 0.223;
    arm.setPoseTarget(target_pose12_1);
    arm.move();
//close the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    sleep(1);

    geometry_msgs::Pose target_pose12_2 = arm.getCurrentPose().pose;
    target_pose12_2.position.x += 0.223;
    arm.setPoseTarget(target_pose12_2);
    arm.move();

//open the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);

//p5
    geometry_msgs::Pose target_pose7;
    target_pose7.orientation.x = -0.00892589;
    target_pose7.orientation.y = 0.9999;
    target_pose7.orientation.z = 0.00340034;
    target_pose7.orientation.w = -0.0105761;

    target_pose7.position.x = -0.8;
    target_pose7.position.y = -0.0782;
    target_pose7.position.z = 0.3023;

    arm.setPoseTarget(target_pose7);
    arm.move();

    geometry_msgs::Pose target_pose_p5_1 = arm.getCurrentPose().pose;
    target_pose_p5_1.position.x -= 0.0311;
    arm.setPoseTarget(target_pose_p5_1);
    arm.move();

//螺栓枪开启
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 0; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);
    sleep(1);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 0; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    sleep(10);

    geometry_msgs::Pose target_pose_p5_2 = arm.getCurrentPose().pose;
    target_pose_p5_2.position.x += 0.160;
    arm.setPoseTarget(target_pose_p5_2);
    arm.move();

    arm.setJointValueTarget(joint_group_positions);
    arm.move();

    geometry_msgs::Pose target_pose13_1 = arm.getCurrentPose().pose;
    target_pose13_1.position.x -= 0.223;
    arm.setPoseTarget(target_pose13_1);
    arm.move();

//close the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    sleep(1);

    geometry_msgs::Pose target_pose13_2 = arm.getCurrentPose().pose;
    target_pose13_2.position.x += 0.223;
    arm.setPoseTarget(target_pose13_2);
    arm.move();

//open the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);

//p6
    geometry_msgs::Pose target_pose8;
    target_pose8.orientation.x = -0.00892589;
    target_pose8.orientation.y = 0.9999;
    target_pose8.orientation.z = 0.00340034;
    target_pose8.orientation.w = -0.0105761;

    target_pose8.position.x = -0.8;
    target_pose8.position.y = 0.0387;
    target_pose8.position.z = 0.3337;

    arm.setPoseTarget(target_pose8);
    arm.move();

    geometry_msgs::Pose target_pose_p6_1 = arm.getCurrentPose().pose;
    target_pose_p6_1.position.x -= 0.029;
    arm.setPoseTarget(target_pose_p6_1);
    arm.move();

//螺栓枪开启
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 0; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);
    sleep(1);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 0; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    sleep(10);

    geometry_msgs::Pose target_pose_p6_2 = arm.getCurrentPose().pose;
    target_pose_p6_2.position.x += 0.160;
    arm.setPoseTarget(target_pose_p6_2);
    arm.move();

    arm.setJointValueTarget(joint_group_positions);
    arm.move();

    geometry_msgs::Pose target_pose14_1 = arm.getCurrentPose().pose;
    target_pose14_1.position.x -= 0.223;
    arm.setPoseTarget(target_pose14_1);
    arm.move();

//close the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    sleep(1);

    geometry_msgs::Pose target_pose14_2 = arm.getCurrentPose().pose;
    target_pose14_2.position.x += 0.223;
    arm.setPoseTarget(target_pose14_2);
    arm.move();

//open the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);

    //arm.clearPathConstraints();

    ros::shutdown();
    return 0;
}
