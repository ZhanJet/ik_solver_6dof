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

#ifndef MAKE_KDL_CHAIN_H_
#define MAKE_KDL_CHAIN_H_

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

using namespace KDL;

struct DH_Params
{
    std::vector<double> q0;
    std::vector<double> d;
    std::vector<double> a;
    std::vector<double> alpha;
};

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
Chain make_kdl_chain(const DH_Params& dh_param){
    double d_bs = dh_param.d[0];
    double d_se = -dh_param.a[1];
    double d_ew = dh_param.d[3];
    double d_wt = dh_param.d[5];
    
    Chain chain;
    chain.addSegment(
            Segment("Segment1", Joint("Joint1",Joint::RotZ),
                    Frame(Vector(0.0, 0.0, d_bs)))
    );
    chain.addSegment(
            Segment("Segment2", Joint("Joint2",Joint::RotY, -1.0),
                    Frame(Vector(0.0, 0.0, d_se)))
    );
    chain.addSegment(
            Segment("Segment3", Joint("Joint3", Joint::RotY),
                    Frame(Vector(0.0, 0.0, 0.0)))
    );
    chain.addSegment(
            Segment("Segment4", Joint("Joint4", Joint::RotZ),
                    Frame(Vector(0.0, 0.0, d_ew)))
    );
    chain.addSegment(
            Segment("Segment5", Joint("Joint5", Joint::RotY),
                    Frame(Vector(0.0, 0.0, 0.0)))
    );
    chain.addSegment(
            Segment("Segment6", Joint("Joint6", Joint::RotZ),
                    Frame(Vector(0.0, 0.0, d_wt)))
    );

    return chain;
}

#endif // !MAKE_KDL_CHAIN_H_