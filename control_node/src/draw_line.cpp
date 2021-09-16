#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ros/ros.h>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>
#include<moveit_msgs/DisplayRobotState.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<moveit_msgs/AttachedCollisionObject.h>
#include<moveit_msgs/CollisionObject.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp> 
#include <opencv2/opencv.hpp>
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

    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 

    //set obstacle
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      sleep_t.sleep();
    }
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = arm.getPlanningFrame();
    collision_object.id = "box";
    shape_msgs::SolidPrimitive primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.2;
    primitive.dimensions[1] = 0.8;
    primitive.dimensions[2] = 0.01;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.4;
    box_pose.position.y = 0;
    box_pose.position.z = 0;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(collision_objects);

    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = -0.00892589;
    target_pose.orientation.y = 0.9999;
    target_pose.orientation.z = 0.00340034;
    target_pose.orientation.w = -0.0105761;

    target_pose.position.x = -0.7;
    target_pose.position.y = 0.0307;
    target_pose.position.z = 0.3224;

    arm.setPoseTarget(target_pose);
    arm.move();
/*
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
    collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
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
    collision_object_2.operation = collision_object_2.ADD;
    collision_objects.push_back(collision_object_2);
    //middle
    //up
    box_pose_4.orientation.w = 1.0;
    box_pose_4.position.x = -0.1;
    box_pose_4.position.y = 0;
    box_pose_4.position.z = 0.9;
    collision_object_4.primitives.push_back(primitive_3);
    collision_object_4.primitive_poses.push_back(box_pose_4);
    collision_object_4.operation = collision_object_4.ADD;
    collision_objects.push_back(collision_object_4);
    //back
    box_pose_5.orientation.w = 1.0;
    box_pose_5.position.x = 0.2;
    box_pose_5.position.y = 0;
    box_pose_5.position.z = 0.45;
    collision_object_5.primitives.push_back(primitive_4);
    collision_object_5.primitive_poses.push_back(box_pose_5);
    collision_object_5.operation = collision_object_5.ADD;
    collision_objects.push_back(collision_object_5);
    // Now, let's add the collision object into the world
    //ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
*/
    // 控制机械臂先回到初始化位置
    //arm.setNamedTarget("home");
    //arm.move();
    //sleep(1);

    std::string end_effector_link = arm.getEndEffectorLink();
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);
    cout << "end_effector_link = " << end_effector_link << endl;

    arm.allowReplanning(true);

    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.01);
    arm.setMaxAccelerationScalingFactor(0.8);
    arm.setMaxVelocityScalingFactor(0.8);



//path constrain
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = end_effector_link;
    ocm.header.frame_id = "base_link";
    ocm.orientation.x = -0.00892589;
    ocm.orientation.y = 0.9999;
    ocm.orientation.z = 0.00340034;
    ocm.orientation.w = -0.0105761;
    ocm.absolute_x_axis_tolerance = 0.2;
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = 0.2;
    ocm.weight = 1.0;

    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    arm.setPathConstraints(test_constraints);

    geometry_msgs::Pose target_pose1 = arm.getCurrentPose().pose;
    target_pose1.position.x -= 0.1;
    arm.setPoseTarget(target_pose1);
    arm.move();

    geometry_msgs::Pose target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x += 0.15;
    arm.setPoseTarget(target_pose2);
    arm.move();

    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.y -= (-0.153);
    target_pose2.position.z -= 0.09;
    arm.setPoseTarget(target_pose2);
    arm.move();

    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x += 0.205;
    arm.setPoseTarget(target_pose2);
    arm.move();

    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.y -= (0.153 + 0.031);
    target_pose2.position.z -= (0.1159 - 0.09);
    arm.setPoseTarget(target_pose2);
    arm.move();

    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x -= 0.15;
    arm.setPoseTarget(target_pose2);
    arm.move();

    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x += 0.15;
    arm.setPoseTarget(target_pose2);
    arm.move();

    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.y += (0.153 + 0.031);
    target_pose2.position.z += (0.1159 - 0.09);
    arm.setPoseTarget(target_pose2);
    arm.move();

    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x -= 0.205;
    arm.setPoseTarget(target_pose2);
    arm.move();

    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x += 0.205;
    arm.setPoseTarget(target_pose2);
    arm.move();

    //arm.clearPathConstraints();

    ros::shutdown();
    return 0;
}
