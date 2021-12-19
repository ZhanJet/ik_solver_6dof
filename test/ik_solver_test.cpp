#include <iostream>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_kdl/tf2_kdl.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

#include "ik_solver_6dof.h"

using namespace std;
using namespace KDL;

double ctrl_period = 0.01;

double gen_rand_angle(int l, int u, double precision=1e-6){
    return double(rand()%(u-l) + l + precision*(rand() % int(1 / precision)));
}

class IKSolverTest : public::testing::Test {
public:
    IKSolverTest();
    virtual ~IKSolverTest();

protected:
    IkSolverPos_6DOF* ik_solver;
    ChainFkSolverPos_recursive* fk_solver;

    Chain kdl_chain;
};

IKSolverTest::IKSolverTest(){
    //define robot D-H parameters
    double d_bs = 0.22;
    double d_se = 0.38;
    double d_ew = 0.42;
    double d_wt = 0.18;
    DH_Params dh_param;
    dh_param.q0 = {.0, -PI/2, -PI/2, .0, .0, .0};
    dh_param.d = {d_bs, .0, .0, d_ew, .0, d_wt};
    dh_param.a = {.0, -d_se, .0, .0, .0, .0};
    dh_param.alpha = {PI/2, PI, PI/2, -PI/2, PI/2, .0};
    //define joints' range
    JntArray min_q(6), max_q(6), max_qdot(6);
    max_q.data << 180, 120, 150, 180, 150, 180;
    max_q.data *= deg2rad;
    min_q.data = -max_q.data;
    max_qdot.data << 180, 180, 180, 180, 180, 180;
    max_qdot.data *= deg2rad;

    ik_solver = new IkSolverPos_6DOF(dh_param, min_q, max_q);
    ik_solver->enableVelLimits(max_qdot, ctrl_period);

    //construct robot chain
    // Chain kdl_chain;
    Tree kdl_tree;
    ros::NodeHandle nh("~");
    std::string robot_description, base_link, tip_link;
    // ros::param::param("robot_description", robot_desc_string, std::string());
    ros::param::param<std::string>("robot_description", robot_description, "robot_description");
    nh.param<std::string>("base_link", base_link, "base_link");
    nh.param<std::string>("tip_link", tip_link, "tip_link");
    if(!kdl_parser::treeFromString(robot_description, kdl_tree)){
        ROS_ERROR("Failed to construct kdl tree object");
    }
    if(!kdl_tree.getChain(base_link, tip_link, kdl_chain)){
        ROS_ERROR("Failed to construct kdl chain object");
    }

    // kdl_chain = make_kdl_chain(dh_param);
    // JntArray q_in(6);
    // if(q_in.rows()!=kdl_chain.getNrOfJoints())
    //     cout<<"Input size does not match internal state"<<endl;

    fk_solver = new ChainFkSolverPos_recursive(kdl_chain);
}

IKSolverTest::~IKSolverTest(){}

TEST_F(IKSolverTest, singlePoseTest){
	JntArray q_in(6);
	JntArray q_out(6);
	Frame pose_out;

	// q_in.data << 0, 30, 30, 0, 60, 30;
	q_in.data << 163.089, -109.308, 143.833, 46.0492, 99.1421, -57.9973;
	q_in.data *= deg2rad;
    // q_in.data << 0.7, 0.5, 0.95, 0.7, 0.93, -0.7;

	cout<<"q_in = "<<q_in(0)*rad2deg<<", "<<q_in(1)*rad2deg<<", "<<q_in(2)*rad2deg<<", "
			<<q_in(3)*rad2deg<<", "<<q_in(4)*rad2deg<<", "<<q_in(5)*rad2deg <<endl;
	int fk_res = fk_solver->JntToCart(q_in, pose_out);
    if(fk_res){
        cout<<"fk_res = "<<fk_res<<endl;
		cout<<"FKSolver Failed!"<<endl;
	}

    // geometry_msgs::TransformStamped pose_out_msg;
    // pose_out_msg.transform.translation.x = 0.053;
    // pose_out_msg.transform.translation.y = 0.166;
    // pose_out_msg.transform.translation.z = 0.979;
    // pose_out_msg.transform.rotation.x = -0.449;
    // pose_out_msg.transform.rotation.y = 0.406;
    // pose_out_msg.transform.rotation.z = 0.333;
    // pose_out_msg.transform.rotation.w = 0.723;
    // pose_out = tf2::transformToKDL(pose_out_msg);
	// pose_out = Frame(Rotation(  -0.97824,   -0.200676,   0.0526803,
    //                             0.202208,    -0.97901,   0.0255248,
    //                             0.0464523,   0.0356218,    0.998285),
    //                  Vector( 0.260, 0.086, 1.097));

	// cout<<"pose_out_qin = "<<endl<<pose_out<<endl;

	int res = ik_solver->CartToJnt(q_in, pose_out, q_out);
	if(res){
        cout<<"ik_res = "<<res<<endl;
		cout<<"IKSolver Failed!"<<endl;
	}
	cout<<endl<<"q_out = "<<q_out(0)*rad2deg<<", "<<q_out(1)*rad2deg<<", "<<q_out(2)*rad2deg<<", "
			<<q_out(3)*rad2deg<<", "<<q_out(4)*rad2deg<<", "<<q_out(5)*rad2deg <<endl;
	fk_solver->JntToCart(q_out, pose_out);

	JntArray q_error(6);
	Subtract(q_out, q_in, q_error);
	cout<<"q_error = "<<q_error(0)*rad2deg<<", "<<q_error(1)*rad2deg<<", "<<q_error(2)*rad2deg<<", "
			<<q_error(3)*rad2deg<<", "<<q_error(4)*rad2deg<<", "<<q_error(5)*rad2deg <<endl;
	EXPECT_TRUE(Equal(q_in, q_out, 1e-6));
}

TEST_F(IKSolverTest, LargeScaleTest){
  JntArray q_in(6);
	JntArray q_out(6);
	Frame pose_out;

	int test_num = 1e5;
	int error_num = 0;
	double precision = 1e-4;
	for(int i = 1; i < test_num; i++){
		//generate random angles
		for(unsigned int j = 0; j < 6; j++){
			q_in(j) = gen_rand_angle(int(ik_solver->getMinq()(j)*rad2deg), int(ik_solver->getMaxq()(j)*rad2deg), precision) * deg2rad;
		}
		//avoid singularity range
		double distance = 1.0 * deg2rad;
		if(fabs(q_in(2) < distance)){
			q_in(2) = sign(q_in(2)) * distance;
		}
		if(fabs(q_in(4) < distance)){
			q_in(4) = sign(q_in(4)) * distance;
		}

		fk_solver->JntToCart(q_in, pose_out);

		int res = ik_solver->CartToJnt(q_in, pose_out, q_out);

		if(!Equal(q_in, q_out, 180*0.001*deg2rad)){
			cout<<"test_num = "<<i<<":	Error !!!"<<endl;
			cout<<"q_in = "<<q_in(0)*rad2deg<<", "<<q_in(1)*rad2deg<<", "<<q_in(2)*rad2deg<<", "
					<<q_in(3)*rad2deg<<", "<<q_in(4)*rad2deg<<", "<<q_in(5)*rad2deg <<endl;

			cout<<endl<<"q_out = "<<q_out(0)*rad2deg<<", "<<q_out(1)*rad2deg<<", "<<q_out(2)*rad2deg<<", "
					<<q_out(3)*rad2deg<<", "<<q_out(4)*rad2deg<<", "<<q_out(5)*rad2deg <<endl;

			JntArray q_error(6);
			Subtract(q_out, q_in, q_error);
			cout<<endl<<"q_error = "<<q_error(0)*rad2deg<<", "<<q_error(1)*rad2deg<<", "<<q_error(2)*rad2deg<<", "
					<<q_error(3)*rad2deg<<", "<<q_error(4)*rad2deg<<", "<<q_error(5)*rad2deg <<endl;

			error_num++;
		}
		if(error_num > 0){
			cout<<"test_num = "<<i<<endl;
			cout<<"error_num = "<<error_num<<endl;
			break;
		}
	}

	EXPECT_TRUE(Equal(error_num, 0));
}

TEST_F(IKSolverTest, successiveTrajTest){
  JntArray q_in(6);
	JntArray q_out(6);
	Frame pose_out, pose_target;

	///>define start pose
	q_in.data << -90.0-10, 10.0, 115.0, 0.0, -15.0, 22.5-90;
	q_in.data *= deg2rad;
	fk_solver->JntToCart(q_in, pose_out);

	int res_t = ik_solver->CartToJnt(q_in, pose_out, q_out);
	cout<<"res = "<<res_t<<endl;
	cout<<endl<<"q_out = "<<q_out(0)*rad2deg<<", "<<q_out(1)*rad2deg<<", "<<q_out(2)*rad2deg<<", "
			<<q_out(3)*rad2deg<<", "<<q_out(4)*rad2deg<<", "<<q_out(5)*rad2deg <<endl;

	///>define trajectory
	int traj_flag = 1;	//0-line；1-circle
	//line
	Vector Vel(0.03, 0.0, -0.03);
	Vector RotVec(1, 1, 1);
	double RotVel = 0.0;
	//circle
	double R = 0.1;
	double omega = 0.5;
	double theta = 0;

	///>publish ROS topic
	ros::NodeHandle nh;
	ros::Publisher jnt_cmd_pub = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>(
																	"arm/elfin_arm_controller/follow_joint_trajectory/goal",5);

	control_msgs::FollowJointTrajectoryActionGoal traj;
	traj.goal.trajectory.header.frame_id = "elfin_base_link";
	traj.goal.trajectory.joint_names.resize(6);
	traj.goal.trajectory.points.resize(1);
	traj.goal.trajectory.points[0].positions.resize(6);

	traj.goal.trajectory.joint_names[0] = "elfin_joint1";
	traj.goal.trajectory.joint_names[1] = "elfin_joint2";
	traj.goal.trajectory.joint_names[2] = "elfin_joint3";
	traj.goal.trajectory.joint_names[3] = "elfin_joint4";
	traj.goal.trajectory.joint_names[4] = "elfin_joint5";
	traj.goal.trajectory.joint_names[5] = "elfin_joint6";

	ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
	sensor_msgs::JointState joint_state;
	joint_state.position.resize(6);
	joint_state.velocity.resize(6);
	joint_state.effort.resize(6);
	joint_state.name.push_back("elfin_joint1");
	joint_state.name.push_back("elfin_joint2");
	joint_state.name.push_back("elfin_joint3");
	joint_state.name.push_back("elfin_joint4");
	joint_state.name.push_back("elfin_joint5");
	joint_state.name.push_back("elfin_joint6");
	
	double rate = 1/ctrl_period;
	double sim_time = 60;
    ros::Rate loop_rate(rate);
	int count = 0, res = 0;
	pose_target = pose_out;
	while (ros::ok() and count < (sim_time*rate) and res == 0){
		if(traj_flag == 0){
			pose_target.p += Vel / rate;
		} else if(traj_flag == 1){
			//circle in XY-plane
			// pose_target.p[0] += -R*sin(theta)*omega/rate;
			// pose_target.p[1] += R*cos(theta)*omega/rate;
			// pose_target.p[2] += 0;
			//circle in YZ-plane
			// pose_target.p[0] += 0;
			// pose_target.p[1] += R*cos(theta)*omega/rate;
			// pose_target.p[2] += -R*sin(theta)*omega/rate;
            //circle in XZ-plane
			pose_target.p[0] += R*cos(theta)*omega/rate;
			pose_target.p[1] += 0;
			pose_target.p[2] += -R*sin(theta)*omega/rate;

			theta += omega/rate;
//			if(theta >= 2*PI or theta <= -2*PI){
//				omega *= -1;
//			}
		}

		res = ik_solver->CartToJnt(q_in, pose_target, q_out);
		q_in = q_out;

		for(unsigned int i = 0; i < 6; i++){
			traj.goal.trajectory.points[0].positions[i] = q_out(i);
		}
		traj.goal.trajectory.header.stamp = ros::Time::now();
        traj.goal.trajectory.points[0].time_from_start = ros::Duration(1/rate);
		jnt_cmd_pub.publish(traj);

    for(unsigned int i = 0; i < 7; i++){
			joint_state.position[i] = q_out(i);
		}
		joint_state.header.stamp = ros::Time::now();
		joint_state_pub.publish(joint_state);

		JntArray q_init(6);
		if(Equal(q_out, q_init, 1e-2)){
				cout<<"IKSolver failed!"<<endl;
				ros::shutdown();
		}

		count++;
		if(count%1 == 0){
			cout<<"=== count="<<count<<"; time="<<count/rate<<"s ==="<<endl;
			cout<<"res = "<<res<<endl;
			cout<<"q_out = "<<q_out(0)*rad2deg<<", "<<q_out(1)*rad2deg<<", "<<q_out(2)*rad2deg<<", "
					<<q_out(3)*rad2deg<<", "<<q_out(4)*rad2deg<<", "<<q_out(5)*rad2deg <<endl;

		}
    ros::spinOnce();
		loop_rate.sleep();
	}

	EXPECT_TRUE(Equal(count, sim_time*rate));
}

int main(int argc, char** argv) {
    cout<<"Start IKSolver tesing..."<<endl;
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ik_solver_test");
    ros::NodeHandle nh("~");
    std::string test_string;
    nh.param<std::string>("test_case_string", test_string, "singlePoseTest");
    test_string.insert(test_string.begin(), '*');
    test_string.append("*");
    std::cout<<"gtest example is "<<test_string<<std::endl;
    ::testing::GTEST_FLAG(filter) = test_string;
    //::testing::GTEST_FLAG(filter) = "*singlePoseTest*";
    return RUN_ALL_TESTS();
}
