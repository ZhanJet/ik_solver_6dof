//  Copyright (c) 2021, Zhangjie Tu, zhjtu@buaa.edu.cn
//  All rights reserved.
//   @author zhanjet

//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
 
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "ik_solver_6dof.h"
#include <kdl/utilities/utility.h>

#define EPSILON 1e-10

IkSolverPos_6DOF::IkSolverPos_6DOF(const DH_Params& dh_param, JntArray min_q, JntArray max_q):
    dh_param_(dh_param), min_q_(min_q),max_q_(max_q)
{
    d_bs_ = dh_param.d[0];
    d_se_ = -dh_param.a[1];
    d_ew_ = dh_param.d[3];
    d_wt_ = dh_param.d[5];

    is_qdot_limit_ = false;
    ctrl_period_ = 0.01;
    max_qdot_.resize(6);
    q_init_.resize(6);    
}

IkSolverPos_6DOF::~IkSolverPos_6DOF(){}

int IkSolverPos_6DOF::CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out){
    if(q_init.rows() != 6){
        return -1;
    }

    ///> Transform q_initial into range of [-PI, PI]
    int jnt_range_index[6]={0};
    // JntArray q_init_tmp(6);
    JntArray q_init_tmp(q_init);
    for(int i=0; i<6; i++) {
        // q_init_tmp(i) = q_init(i);
        while(q_init_tmp(i) > PI){
            q_init_tmp(i) -= 2*PI;
            jnt_range_index[i] += 1;
        }
        while(q_init_tmp(i) < -PI){
            q_init_tmp(i) += 2*PI;
            jnt_range_index[i] -= 1;
        }
    }
    ///> Calculate IK solutions in [-PI, PI] and select the nearest one
    // std::vector<std::vector<double> > q_solns(8, std::vector<double>(6,0));
    std::vector<std::vector<double> > q_solns;
    JntArray q_nearest(6);
    cf_3_ = sign(q_init_tmp(2));
    cf_5_ = sign(q_init_tmp(4));
    int solns_num = ik_solve(p_in, q_solns);
    if(solns_num <= 0){
        return -2;
    } else if(solns_num == 1){
        //when solutions have been filtered to one through config_data
        for(int i=0; i<6; i++){
            q_nearest(i) = q_solns[0][i];
        }
    } else {
        //when solutions have been filtered to 2 through config_data
        // or get all the 8 groups of solution without pre-selection.
        double distance;
        double length[8]={0};
        int index=0;
        for(int i=0; i<solns_num; i++){
            for(int j=0; j<6; j++){
                distance = q_solns[i][j] - q_init_tmp(j);
                distance = fabs(PI - fabs(fabs(distance) - PI));
                length[i] += distance;
            }
            if(length[i] < length[index])
                index = i;
        }
        for(int i=0; i<6; i++){
            q_nearest(i) = q_solns[index][i];
        }
    }
    ///> Transform q_soln back to inital range
    for(int i=0; i<6; i++){
        q_out(i) = q_nearest(i) + jnt_range_index[i]*2*PI;
    }
    ///> Check whether the solution is within the allowable range
    if(is_qdot_limit_){
        for(int i=0; i<6; i++){
            double allowable_range = max_qdot_(i)*ctrl_period_;
            double q_l = max(q_init(i)-allowable_range, min_q_(i));
            double q_u = min(q_init(i)+allowable_range, max_q_(i));
            if((q_out(i) < q_l-EPSILON) or (q_out(i) > q_u+EPSILON))
                return -3;
        }
    } else {
        for(int i=0; i<6; i++){
            if((q_out(i) < min_q_(i)-EPSILON) or (q_out(i) > max_q_(i)+EPSILON))
                return -3;
        }
    }

    return 0;
}

void IkSolverPos_6DOF::enableVelLimits(const JntArray& max_qdot, const double ctrl_period){
    is_qdot_limit_ = true;
    max_qdot_ = max_qdot;
    ctrl_period_ = ctrl_period;
}

/**
 * Standard D-H Parameters
 * 	            theta        d       a       alpha
	links = [
            Link([0        d_bs      0       pi/2])
			Link([-pi/2     0      -d_se       pi])
            Link([-pi/2     0        0       pi/2])
            Link([0        d_ew      0      -pi/2])
            Link([0        0         0       pi/2])
            Link([0        d_wt      0          0])
         ];
 */
int IkSolverPos_6DOF::ik_solve(const Frame& desired_pose, std::vector<std::vector<double> >& q_solns){
    Rotation Rd06 = desired_pose.M;
    Vector Xd06 = desired_pose.p;

    Vector Lbs0(0, 0, d_bs_);
    Vector Lse2(-d_se_, 0, 0);
    Vector Lew3(0, 0, d_ew_);
    Vector Lwt6(0, 0, d_wt_);

    /****** compute q3 ******/
    Vector Xsw0 = Xd06 - Lbs0 - Rd06 * Lwt6;
    if(fabs(Xsw0(0))<EPSILON and fabs(Xsw0(1))<EPSILON){
        return -1;  // alignment singularity
    }
    double num = Xsw0.Norm()*Xsw0.Norm() - d_se_*d_se_ - d_ew_*d_ew_;
    double den = 2*d_se_*d_ew_;
    double cosq3 = num / den;
    if(fabs(fabs(cosq3)-1.0)<EPSILON){
        return -1;  // elbow singularity
        // return q_solns.size();  
    }
    double q3 = cf_3_ * acos(cosq3);

    /****** compute q2 ******/
    double a = d_ew_ * sin(q3);
    double b = d_se_ + d_ew_ * cos(q3);
    double rou = sqrt(a*a + b*b);
    double div = Xsw0(2) / rou;
    if(fabs(div) > 1.0+EPSILON){
        return -1;
    }
    if(fabs(rou-fabs(Xsw0(2)))<EPSILON and fabs(div)>1.0){
        div = sign(Xsw0(2));
    }
    double q2[2]={0};
    q2[0] = atan2(a, b) + acos(div);
    q2[1] = atan2(a, b) - acos(div);
    if(q2[0] > PI)  q2[0] -= 2*PI;
    if(q2[1] < -PI)  q2[1] += 2*PI;
    int num_q2 = 2;
    if(q2[0] == q2[1])  num_q2 = 1;
    for(int i=0; i<num_q2; i++){
        /****** compute q1 ******/
        double tmp = a*cos(q2[i]) - b*sin(q2[i]);
        double q1 = atan2(sign(tmp) * Xsw0(1), sign(tmp) * Xsw0(0));

        ///> WRIST
        // Rotation R03 = Rotation::RotZ(q1) * Rotation::RotX(PI/2)
        //                 * Rotation::RotZ(q2[i] - PI/2) * Rotation::RotX(PI)
        //                 * Rotation::RotZ(q3 - PI/2) * Rotation::RotX(PI/2);
        Rotation R03 = Rotation::RotZ(q1 + dh_param_.q0[0]) * Rotation::RotX(dh_param_.alpha[0])
                        * Rotation::RotZ(q2[i] + dh_param_.q0[1]) * Rotation::RotX(dh_param_.alpha[1])
                        * Rotation::RotZ(q3 + dh_param_.q0[2]) * Rotation::RotX(dh_param_.alpha[2]);
        Rotation R36 = R03.Inverse() * Rd06;
        /****** compute q5 ******/
        double cosq5 = R36(2,2);
        if(fabs(cosq5)>1+EPSILON){
            continue;   // wrist singularity
        }
        if(fabs(fabs(cosq5)-1)<EPSILON){
            cosq5 = sign(cosq5);
        }
        double q5 = cf_5_ * acos(cosq5);
        /****** compute q4 ******/
        double q4 = atan2(sign(q5) * R36(1,2), sign(q5) * R36(0,2));
        /****** compute q6 ******/
        double q6 = atan2(sign(q5) * R36(2,1), -sign(q5) * R36(2,0));

        //save the solution
        std::vector<double> q_group = {q1, q2[i], q3, q4, q5, q6};
        q_solns.push_back(q_group);
    }
    
    return q_solns.size();
}