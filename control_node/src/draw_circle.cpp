#include <ros/ros.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
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
    ros::init(argc, argv, "move_with_circle");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface ur5("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 

    string eef_link = ur5.getEndEffector();
    std::string reference_frame = "base_link";
    ur5.setPoseReferenceFrame(reference_frame);

    ur5.allowReplanning(true);

    ur5.setGoalPositionTolerance(0.001);
    ur5.setGoalOrientationTolerance(0.01);
    ur5.setMaxAccelerationScalingFactor(0.5);
    ur5.setMaxVelocityScalingFactor(0.5);

    //set obstacle
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      sleep_t.sleep();
    }
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = ur5.getPlanningFrame();
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

    // 控制机械臂先回到初始化位置
    ur5.setNamedTarget("home");
    ur5.move();
    sleep(1);

    geometry_msgs::Pose target_pose;
    //target_pose.orientation.x = 0.70711;
    //target_pose.orientation.y = 0.70711;
    //target_pose.orientation.z = 0;
    //target_pose.orientation.w = 0;
    target_pose.orientation.x = -0.00892589;
    target_pose.orientation.y = 0.9999;
    target_pose.orientation.z = 0.00340034;
    target_pose.orientation.w = -0.0105761;

    target_pose.position.x = 0.070859;
    target_pose.position.y = 0.36739;
    target_pose.position.z = 0.64716;

    ur5.setPoseTarget(target_pose);
    ur5.move();

    vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);


    //在xy平面内生成一个圆周
    double centerA = target_pose.position.x;
    double centerB = target_pose.position.y;
    double radius = 0.15;

    for(double theta = 0.0; theta < M_PI; theta += 0.05)
    {
        target_pose.position.x = centerA + radius * cos(theta);
        target_pose.position.y = centerB + radius * sin(theta);
        waypoints.push_back(target_pose);
    }
/*
    target_pose.position.x += 0.1;
    waypoints.push_back(target_pose);

    target_pose.position.y += 0.1;
    waypoints.push_back(target_pose);

    target_pose.position.x -= 0.1;
    target_pose.position.y -= 0.1;
    waypoints.push_back(target_pose);
*/
    // 笛卡尔空间下的路径规划
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = ur5.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;

        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }

    if(fraction == 1)
    {
        ROS_INFO("Path computed successfully. Moving the arm.");

        // 生成机械臂的运动规划数据
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        // 执行运动
        ur5.execute(plan);
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }
    //清除限制
    ur5.clearPathConstraints();
    // 控制机械臂先回到初始化位置
/*
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = 0.70711;
    target_pose1.orientation.y = 0;
    target_pose1.orientation.z = 0.70711;
    target_pose1.orientation.w = 0;

    target_pose1.position.x = 0.0559;
    target_pose1.position.y = 0.42739;
    target_pose1.position.z = 0.47716;

    ur5.setPoseTarget(target_pose1);
    ur5.move();
*/
    ur5.setNamedTarget("home");
    ur5.move();
    sleep(1);

    ros::shutdown();
    return 0;
}
