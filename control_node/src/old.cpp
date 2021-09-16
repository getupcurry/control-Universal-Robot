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
    ros::init(argc, argv, "aw");
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



    arm.allowReplanning(true);
    //arm.setMaxAccelerationScalingFactor(0.8);
    //arm.setMaxVelocityScalingFactor(0.8);


    


    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = 0.620431;
    target_pose.orientation.y = 0.784092;
    target_pose.orientation.z = 0.00994781;
    target_pose.orientation.w = -0.0121478;

    target_pose.position.x = -0.78;
    target_pose.position.y = 0.1122;
    target_pose.position.z =  0.3380;

    arm.setPoseTarget(target_pose);
    arm.move();

    geometry_msgs::Pose target_pose1 = arm.getCurrentPose().pose;
    target_pose1.position.x -= 0.0359;
    arm.setPoseTarget(target_pose1);
    arm.move();


    ros::shutdown();
    return 0;
}
