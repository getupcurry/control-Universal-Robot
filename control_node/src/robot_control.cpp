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
//#include <opencv2/highgui.hpp>
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
//#include <ur_robot_driver/hardware_interface.h>

using namespace std;
using namespace cv;
using namespace Eigen;
using  namespace aruco;

const double d1 = 0.1273;
const double a2 = -0.612;
const double a3 = -0.5723;
const double d4 = 0.163941;
const double d5 = 0.1157;
const double d6 = 0.0922;
const double ZERO_THRESH = 0.00000001;
int SIGN(double x)
{
	return (x > 0) - (x < 0);
}
const double PI = 3.14159265;

void forward(const double* q, double* T) {
	double s1 = sin(*q), c1 = cos(*q); q++;
	double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
	double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++;
	double s4 = sin(*q), c4 = cos(*q); q234 += *q; q++;
	double s5 = sin(*q), c5 = cos(*q); q++;
	double s6 = sin(*q), c6 = cos(*q);
	double s23 = sin(q23), c23 = cos(q23);
	double s234 = sin(q234), c234 = cos(q234);

	*T = c6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * s6; T++;  //nx
	*T = -s6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * c6; T++;  //Ox
	*T = -(c234 * c1 * s5 - c5 * s1); T++;//ax
	*T = -(d6 * c234 * c1 * s5 - a3 * c23 * c1 - a2 * c1 * c2 - d6 * c5 * s1 - d5 * s234 * c1 - d4 * s1); T++;//Px

	*T = -c6 * (c1 * s5 - c234 * c5 * s1) - s234 * s1 * s6; T++;//ny
	*T = s6 * (c1 * s5 - c234 * c5 * s1) - s234 * c6 * s1; T++;//Oy
	*T = -(c1 * c5 + c234 * s1 * s5); T++;//ay
	*T = -(d6 * (c1 * c5 + c234 * s1 * s5) + d4 * c1 - a3 * c23 * s1 - a2 * c2 * s1 - d5 * s234 * s1); T++;//py


	*T = -(-c234 * s6 - s234 * c5 * c6); T++;//nz
	*T = -(s234 * c5 * s6 - c234 * c6); T++;//oz
	*T = -s234 * s5; T++;//az
	*T = d1 + a3 * s23 + a2 * s2 - d5 * (c23 * c4 - s23 * s4) - d6 * s5 * (c23 * s4 + s23 * c4); T++;//Pz
	*T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;//姿态
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_moveit");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);
 
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
    //moveit_msgs::DisplayTrajectory display_trajectory; 
    std::string end_effector_link = arm.getEndEffectorLink();
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

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
    box_pose_3.orientation.w = 1.0;
    box_pose_3.position.x = -0.9;
    box_pose_3.position.y = 0.3;
    box_pose_3.position.z = 0.35;
    collision_object_3.primitives.push_back(primitive_2);
    collision_object_3.primitive_poses.push_back(box_pose_3);
    collision_object_3.operation = collision_object_3.ADD;
    collision_objects.push_back(collision_object_3);
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

// Img2Cam

	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
			
	Mat image = imread("/home/yu/MyRobotCode/src/rgb.bmp",1);
	Mat imageCopy;
	const Mat  intrinsic_matrix = (Mat_<float>(3, 3)
	<< 958.5629, 0.0, 634.9304,
           0.0, 958.5656, 494.8522,
           0.0, 0.0, 1.0);

	const Mat  arucodistCoeffs = (Mat_<float>(1, 5) << 0.0416779, -0.0258146, -0.000342118, 0.000402092, 0.00);

	vector< int > ids;
	vector< vector< Point2f > > corners, rejected;
	vector< Vec3d > rvecs, tvecs;
	Mat  R;
	Matrix3d r(3,3);
	Matrix4d Img2Cam(4, 4);
	// detect markers and estimate pose
	detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
	image.copyTo(imageCopy);
	if (ids.size() > 0)
	{
		drawDetectedMarkers(imageCopy, corners, ids);
		std::vector <cv::Vec3d> rvecs, tvecs;
		estimatePoseSingleMarkers(corners, 0.02, intrinsic_matrix, arucodistCoeffs, rvecs, tvecs);
		Rodrigues(rvecs[0], R, noArray());
		cv2eigen(R, r);
		Img2Cam(0, 0) = r(0, 0); Img2Cam(0, 1) = r(0, 1); Img2Cam(0, 2) = r(0, 2); Img2Cam(0, 3) = tvecs[0][0];
		Img2Cam(1, 0) = r(1, 0); Img2Cam(1, 1) = r(1, 1); Img2Cam(1, 2) = r(1, 2); Img2Cam(1, 3) = tvecs[0][1];
		Img2Cam(2, 0) = r(2, 0); Img2Cam(2, 1) = r(2, 1); Img2Cam(2, 2) = r(2, 2); Img2Cam(2, 3) = tvecs[0][2];
		Img2Cam(3, 0) = 0;       Img2Cam(3, 1) = 0;       Img2Cam(3, 2) = 0;       Img2Cam(3, 3) = 1;

		cout << "Img2Cam = \n" << Img2Cam << endl;
		cout << endl;
		for (int i = 0; i < ids.size(); i++)
		{
			cv::aruco::drawAxis(imageCopy, intrinsic_matrix, arucodistCoeffs, rvecs[i], tvecs[i], 0.05);
		}
	}

// Cam2End  End2Base  Img2Base
        const robot_state::JointModelGroup* current_joint_model_group = arm.getCurrentState()->getJointModelGroup("manipulator");
        moveit::core::RobotStatePtr current_state_now = arm.getCurrentState();
        std::vector<double> current_joint_group_positions;
        current_state_now->copyJointGroupPositions(current_joint_model_group, current_joint_group_positions);
	double Q[6] = { current_joint_group_positions[0] - PI, current_joint_group_positions[1], current_joint_group_positions[2], current_joint_group_positions[3], current_joint_group_positions[4], current_joint_group_positions[5] - (PI * 1.5)};
	double t1[16];
	double * T, *q;
	q = &Q[0];
	T = &t1[0];
	forward(q, T);
	//输出q
	cout << "关节角度q:   " << "[";
	for (int i = 0; i < 6; i++) { cout << Q[i] << "  "; }
	cout << "]" << endl;
	cout << endl;

        Matrix4d End2Base(4, 4); 
        End2Base = Map<Matrix<double, 4, 4, RowMajor>>(t1);
        cout << "End2Base = \n" << End2Base << endl;
	cout << endl;

	Matrix4d Cam2End(4, 4);
		Cam2End(0, 0) = -0.00934801775081473; Cam2End(0, 1) = -0.002423193062917495; Cam2End(0, 2) = -0.9999533702625889;  Cam2End(0, 3) = -0.1348786895885715;
		Cam2End(1, 0) = -0.9995083445988659;  Cam2End(1, 1) = -0.02990655117622554;  Cam2End(1, 2) = 0.009416330175747112; Cam2End(1, 3) = 0.0654528644908631;
		Cam2End(2, 0) = -0.0299279742275575; Cam2End(2, 1) = 0.9995497618088476; Cam2End(2, 2) = -0.002142434715699082; Cam2End(2, 3) = 0.05895148731035948;
		Cam2End(3, 0) = 0;       Cam2End(3, 1) = 0;       Cam2End(3, 2) = 0;       Cam2End(3, 3) = 1;

	Vector3d eulerAngle = r.eulerAngles(2,1,0);
	cout << " pitch yaw roll: " << eulerAngle[2] << "   "<< eulerAngle[1] << "   " << eulerAngle[0] << endl;
	cout << endl;
/*	
	double pitch = -eulerAngle[2];
	double yaw = -eulerAngle[1];
	double roll = -eulerAngle[0];

	Matrix4d Img2Base(4, 4);
	Matrix4d trotx(4, 4);
		trotx(0, 0) = 1;  trotx(0, 1) = 0; trotx(0, 2) = 0;  trotx(0, 3) = 0;
		trotx(1, 0) = 0;  trotx(1, 1) = cos(pitch);  trotx(1, 2) = -sin(pitch); trotx(1, 3) = 0;
		trotx(2, 0) = 0;  trotx(2, 1) = sin(pitch);  trotx(2, 2) = cos(pitch);  trotx(2, 3) = 0;
		trotx(3, 0) = 0;  trotx(3, 1) = 0;       trotx(3, 2) = 0;       trotx(3, 3) = 1;
	Matrix4d troty(4, 4);
		troty(0, 0) = cos(yaw);  troty(0, 1) = 0; troty(0, 2) = sin(yaw);  troty(0, 3) = 0;
		troty(1, 0) = 0;  troty(1, 1) = 1;  troty(1, 2) = 0; troty(1, 3) = 0;
		troty(2, 0) = -sin(yaw);  troty(2, 1) = 0;  troty(2, 2) = cos(yaw);  troty(2, 3) = 0;
		troty(3, 0) = 0;  troty(3, 1) = 0;       troty(3, 2) = 0;       troty(3, 3) = 1;
	Matrix4d trotz(4, 4);
		trotz(0, 0) = cos(roll);  trotz(0, 1) = -sin(roll); trotz(0, 2) = 0;  trotz(0, 3) = 0;
		trotz(1, 0) = sin(roll);  trotz(1, 1) = cos(roll);  trotz(1, 2) = 0; trotz(1, 3) = 0;
		trotz(2, 0) = 0;  trotz(2, 1) = 0;  trotz(2, 2) = 1;  trotz(2, 3) = 0;
		trotz(3, 0) = 0;  trotz(3, 1) = 0;       trotz(3, 2) = 0;       trotz(3, 3) = 1;
	Img2Base = End2Base * Cam2End * Img2Cam * trotx * troty * trotz;
*/

	double pitch = PI;
	double roll = PI - eulerAngle[0];
	//if(eulerAngle[0] < PI/2)
	//	roll  = PI - eulerAngle[0];
	//else 
	//        roll = - eulerAngle[0];

	Matrix4d Img2Base(4, 4);
	Matrix4d trotx(4, 4);
		trotx(0, 0) = 1;  trotx(0, 1) = 0; trotx(0, 2) = 0;  trotx(0, 3) = 0;
		trotx(1, 0) = 0;  trotx(1, 1) = cos(pitch);  trotx(1, 2) = -sin(pitch); trotx(1, 3) = 0;
		trotx(2, 0) = 0;  trotx(2, 1) = sin(pitch);  trotx(2, 2) = cos(pitch);  trotx(2, 3) = 0;
		trotx(3, 0) = 0;  trotx(3, 1) = 0;       trotx(3, 2) = 0;       trotx(3, 3) = 1;
	Matrix4d trotz(4, 4);
		trotz(0, 0) = cos(roll);  trotz(0, 1) = -sin(roll); trotz(0, 2) = 0;  trotz(0, 3) = 0;
		trotz(1, 0) = sin(roll);  trotz(1, 1) = cos(roll);  trotz(1, 2) = 0; trotz(1, 3) = 0;
		trotz(2, 0) = 0;  trotz(2, 1) = 0;  trotz(2, 2) = 1;  trotz(2, 3) = 0;
		trotz(3, 0) = 0;  trotz(3, 1) = 0;       trotz(3, 2) = 0;       trotz(3, 3) = 1;
	Img2Base = End2Base * Cam2End * Img2Cam * trotx * trotz;
        cout << "Img2Base = \n" << Img2Base << endl;

//Qiang2End
	Matrix4d Qiang2End(4, 4);
		Qiang2End(0, 0) = 1;  Qiang2End(0, 1) = 0;  Qiang2End(0, 2) = 0;  Qiang2End(0, 3) = 0;
		Qiang2End(1, 0) = 0;  Qiang2End(1, 1) = 0;  Qiang2End(1, 2) = -1; Qiang2End(1, 3) = -0.18;
		Qiang2End(2, 0) = 0;  Qiang2End(2, 1) = 1;  Qiang2End(2, 2) = 0;  Qiang2End(2, 3) = 0.038;
		Qiang2End(3, 0) = 0;  Qiang2End(3, 1) = 0;  Qiang2End(3, 2) = 0;  Qiang2End(3, 3) = 1;

//trotz(-PI/2)
        double ang = -PI/2;
	Matrix4d trot(4, 4);
		trot(0, 0) = cos(ang);  trot(0, 1) = -sin(ang);  trot(0, 2) = 0;  trot(0, 3) = 0;
		trot(1, 0) = sin(ang);  trot(1, 1) = cos(ang);   trot(1, 2) = 0;  trot(1, 3) = 0;
		trot(2, 0) = 0;         trot(2, 1) = 0;          trot(2, 2) = 1;  trot(2, 3) = 0;
		trot(3, 0) = 0;         trot(3, 1) = 0;          trot(3, 2) = 0;  trot(3, 3) = 1;


//End2BaseRes
	Matrix4d End2BaseRes(4, 4);
	End2BaseRes = Img2Base * Qiang2End.inverse();
	End2BaseRes = End2BaseRes * trot;
        cout << "End2BaseRes = \n" << End2BaseRes << endl;
	cout << endl;

//四元数
	Matrix3d rotation_matrix(3,3);
		rotation_matrix(0, 0) = End2BaseRes(0, 0);  rotation_matrix(0, 1) = End2BaseRes(0, 1);  rotation_matrix(0, 2) = End2BaseRes(0, 2);
		rotation_matrix(1, 0) = End2BaseRes(1, 0);  rotation_matrix(1, 1) = End2BaseRes(1, 1);  rotation_matrix(1, 2) = End2BaseRes(1, 2);
		rotation_matrix(2, 0) = End2BaseRes(2, 0);  rotation_matrix(2, 1) = End2BaseRes(2, 1);  rotation_matrix(2, 2) = End2BaseRes(2, 2);
	Eigen::Quaterniond xyzw(rotation_matrix);
	cout << "rotation_x = " << xyzw.x() << "   rotation_y = " << xyzw.y() << "   rotation_z = " << xyzw.z() << "   rotation_w = " << xyzw.w() << endl;

//robot move
    geometry_msgs::Pose target_pose;

//step 1  
    target_pose.position.x = -0.7;
    target_pose.position.y = End2BaseRes(1,3);
    target_pose.position.z = End2BaseRes(2,3);

    target_pose.orientation.x= xyzw.x();
    target_pose.orientation.y = xyzw.y();
    target_pose.orientation.z = xyzw.z();
    target_pose.orientation.w = xyzw.w();
 
    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);
 
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    moveit::planning_interface::MoveItErrorCode success1 = arm.plan(plan1);
    if(success1)
      arm.execute(plan1);
    else
      cout << "falled"<<endl;
/*
//step2
    const robot_state::JointModelGroup* joint_model_group = arm.getCurrentState()->getJointModelGroup("manipulator"); 
    
    moveit::core::RobotStatePtr current_state = arm.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[5] -= (PI/2);  // radians
    arm.setJointValueTarget(joint_group_positions);
    //arm.move();
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    moveit::planning_interface::MoveItErrorCode success2 = arm.plan(plan2);
    arm.execute(plan2);
*/

//step2
    geometry_msgs::Pose target_pose1 = arm.getCurrentPose().pose;
    target_pose1.position.x -= (-0.7 - End2BaseRes(0,3));
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
//拔出螺栓枪
    geometry_msgs::Pose target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x += 0.15;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
//移动到爪夹处
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.y -= (-0.153);
    target_pose2.position.z -= 0.09;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x -= 0.205;
    arm.setPoseTarget(target_pose2);
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
//拔出螺栓枪
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x += 0.205;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
//open the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);
//point 2
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.y -= (0.153 + 0.031);
    target_pose2.position.z -= (0.1159 - 0.09);
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x -= 0.15;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
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
//拔出螺栓枪
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x += 0.15;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
//移动到爪夹处
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.y += (0.153 + 0.031);
    target_pose2.position.z += (0.1159 - 0.09);
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x -= 0.205;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
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
//拔出螺栓枪
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x += 0.205;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
//open the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);

//point 3
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.y -= (0.147 + 0.153);
    target_pose2.position.z -= (0.147 - 0.09);
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x -= 0.15;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
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
//拔出螺栓枪
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x += 0.15;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
//移动到爪夹处
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.y += (0.147 + 0.153);
    target_pose2.position.z += (0.147 - 0.09);
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x -= 0.205;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
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
//拔出螺栓枪
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x += 0.205;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
//open the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);

//point 4
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.y -= (0.2318 + 0.153);
    target_pose2.position.z += (0.09 - 0.062);
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x -= 0.15;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
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
//拔出螺栓枪
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x += 0.15;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
//移动到爪夹处
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.y += (0.2318 + 0.153);
    target_pose2.position.z -= (0.09 - 0.062);
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x -= 0.205;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
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
//拔出螺栓枪
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x += 0.205;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
//open the gripper
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 2; // Pin number
    io_service.request.state = 0;
    io_client.call(io_service);
    io_service.request.fun = 1; // Specify what type of IO you want to control, 1 is digital output
    io_service.request.pin = 3; // Pin number
    io_service.request.state = 1;
    io_client.call(io_service);

//point 5
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.y -= (0.2008 + 0.153);
    target_pose2.position.z += (0.0538 + 0.09);
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x -= 0.15;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
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
//拔出螺栓枪
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x += 0.15;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
//移动到爪夹处
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.y += (0.2008 + 0.153);
    target_pose2.position.z -= (0.0538 + 0.09);
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x -= 0.205;
    arm.setPoseTarget(target_pose2);
    arm.move();
    sleep(0.5);
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
//拔出螺栓枪
    target_pose2 = arm.getCurrentPose().pose;
    target_pose2.position.x += 0.205;
    arm.setPoseTarget(target_pose2);
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

    sleep(2.0);
    ros::shutdown();
}


