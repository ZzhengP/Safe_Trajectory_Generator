//
// Created by zheng on 06/05/2021.
//

/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file examples/example1.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Very simple example for testing qpOASES using the QProblem class.
 */


#include <memory>
#include <qpOASES.hpp>
#include "robot/panda_controller.h"
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <ros/node_handle.h>

/** Example for qpOASES main function using the QProblem class. */
int main(int argc, char** argv )
{
    ros::init(argc,argv, "test");
    ros::NodeHandle node_handle;

    KDL::Chain chain ; /*! Robot kdl chain */
    KDL::JntArray ll ; /*! Joint lower limit */
    KDL::JntArray ul ; /*! Joint upper limit */

    double timeout;
    node_handle.param("timeout", timeout, 0.005);
    std::string urdf_param;
    node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));
    double eps = 1e-5;
    // Initialize the KDL Chain
    TRAC_IK::TRAC_IK trancik_solver("panda_link0", "panda_link8", urdf_param, timeout, eps);
    bool valid = trancik_solver.getKDLChain(chain);
    if (valid){
        ROS_INFO_STREAM("get kdl chain from urdf");
    }
    std::cout << "test include task " << std::endl;
    return 0;
}


/*
 *	end of file
 */
