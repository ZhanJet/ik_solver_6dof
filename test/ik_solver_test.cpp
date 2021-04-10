#include <iostream>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_kdl/tf2_kdl.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include "ik_solver_6dof.h"

using namespace std;
using namespace KDL;

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
    double ctrl_period = 0.01;

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

	q_in.data << 0, 30, 30, 0, 60, 30;
	q_in.data *= deg2rad;
    // q_in.data << 0.7, 0.5, 0.95, 0.7, 0.93, -0.7;

	cout<<"q_in = "<<q_in(0)*rad2deg<<", "<<q_in(1)*rad2deg<<", "<<q_in(2)*rad2deg<<", "
			<<q_in(3)*rad2deg<<", "<<q_in(4)*rad2deg<<", "<<q_in(5)*rad2deg <<endl;
	int fk_res = fk_solver->JntToCart(q_in, pose_out);
    if(res){
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
	// cout<<"pose_out_qout = "<<endl<<pose_out<<endl;

	JntArray q_error(6);
	Subtract(q_out, q_in, q_error);
	cout<<"q_error = "<<q_error(0)*rad2deg<<", "<<q_error(1)*rad2deg<<", "<<q_error(2)*rad2deg<<", "
			<<q_error(3)*rad2deg<<", "<<q_error(4)*rad2deg<<", "<<q_error(5)*rad2deg <<endl;
	EXPECT_TRUE(Equal(q_in, q_out, 1e-6));
}

TEST_F(IKSolverTest, LargePoseTest){
    
}

TEST_F(IKSolverTest, successiveTrajTest){
    
}

int main(int argc, char** argv) {
    cout<<"Start IKSolver tesing..."<<endl;
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ik_solver_test");
    ros::NodeHandle nh("~");
    std::string test_string;
    nh.param<std::string>("test_case_string", test_string, "singlePoseTest");
    test_string.append("*");
    // ::testing::GTEST_FLAG(filter) = "singlePoseTest*";
    ::testing::GTEST_FLAG(filter) = test_string;
    return RUN_ALL_TESTS();
}