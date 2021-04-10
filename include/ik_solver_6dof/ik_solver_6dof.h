//  Copyright (c) 2021, Zhangjie Tu.
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

#ifndef IKSOLVER_6DOF_H_
#define IKSOLVER_6DOF_H_

#include <cmath>
#include <kdl/chainiksolver.hpp>
#include "make_kdl_chain.h"

using namespace KDL;

/**
 * Implementation of an analytical inverse position kinematics
 * algorithm to calculate the position transformation from Cartesian
 * to joint space for manipulators with a typical configuaration
 * as following(eg.ABB IRB120):
 * 
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

class IkSolverPos_6DOF : public ChainIkSolverPos
{
public:
    /**
     * @brief Construct a new IkSolverPos_6DOF object
     * @param dh_param Standard D-H parameters
     * @param min_jnt minimum joint values
     * @param max_jnt maximum joint values
     */
    IkSolverPos_6DOF(const DH_Params& dh_param, JntArray min_q, JntArray max_q);
    ~IkSolverPos_6DOF();
    /**
    * Calculate inverse position kinematics, from cartesian
    * coordinates to joint coordinates.
    *
    * @param q_init initial guess of the joint coordinates
    * @param p_in input cartesian coordinates
    * @param q_out output joint coordinates
    *
    * @return if < 0 something went wrong
    */
    int CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out);

    void updateInternalDataStructures(){}

    void enableVelLimits(const JntArray& max_qdot, const double ctrl_period);
    void disableVelLimits(){is_qdot_limit_ = false;}

private:
    int ik_solve(const Frame& desired_pose, std::vector<std::vector<double> >& q_ik_solns);

private:
    DH_Params dh_param_;
    double d_bs_, d_se_, d_ew_, d_wt_;
    JntArray max_qdot_, min_q_, max_q_;
    bool is_qdot_limit_;
    double ctrl_period_;
    
    int cf_3_, cf_5_;
    JntArray q_init_;
};


#endif /* IKSOLVER_6DOF_H_ */