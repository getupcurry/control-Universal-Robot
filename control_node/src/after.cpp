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


// Img2Cam
    Matrix4d Img2Cam(4, 4);

		Img2Cam(0, 0) = 0.984907; Img2Cam(0, 1) = 0.00293561; Img2Cam(0, 2) = -0.173059;       Img2Cam(0, 3) = 0.0241238;
		Img2Cam(1, 0) = 0;  Img2Cam(1, 1) = 0.999856;  Img2Cam(1, 2) = 0.0169606;  Img2Cam(1, 3) = 0.0805615;
		Img2Cam(2, 0) = 0.173084; Img2Cam(2, 1) = -0.0167046; Img2Cam(2, 2) = 0.984765; Img2Cam(2, 3) = 0.356682;
		Img2Cam(3, 0) = 0;       Img2Cam(3, 1) = 0;       Img2Cam(3, 2) = 0;       Img2Cam(3, 3) = 1;

// Cam2End  End2Base  Img2Base
        const robot_state::JointModelGroup* current_joint_model_group = arm.getCurrentState()->getJointModelGroup("manipulator");
        moveit::core::RobotStatePtr current_state_now = arm.getCurrentState();
        std::vector<double> current_joint_group_positions;
        current_state_now->copyJointGroupPositions(current_joint_model_group, current_joint_group_positions);
	double Q[6] = { current_joint_group_positions[0] - PI, current_joint_group_positions[1], current_joint_group_positions[2], current_joint_group_positions[3], current_joint_group_positions[4], current_joint_group_positions[5]};
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


	//if(eulerAngle[0] < PI/2)
	//	roll  = PI - eulerAngle[0];
	//else 
	//        roll = - eulerAngle[0];

	Matrix4d Img2Base(4, 4);

	Img2Base = End2Base * Cam2End * Img2Cam;
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


    sleep(2.0);
    ros::shutdown();
}


