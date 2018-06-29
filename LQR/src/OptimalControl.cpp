/*
 * LQR.cpp
 *
 *  Created on: 2 Jun 2018
 *  Author: Sebastijan Veselic
 *  Last edit: 25.6
 *  Last comment: 22.6.
 */

#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <stdio.h>
#include <math.h>


/** Takes the uNaught; this should be user input in this case from which we subtract the K matrix and current state */

Eigen::VectorXd iLQR::OptimalControl (const Containers::StateVector &x, const Containers::InputVector &uNaught, const Containers::FeedbackMatrix &K)
{

	 Util Utility;

	 Utility.SizeCheck(x, Utility.StVec());
	 Utility.SizeCheck(uNaught, Utility.InpVec());
	 Utility.SizeCheck(K, Utility.FedMat());

	Containers::InputVector uStar;

	uStar = uNaught - K*x;

	return uStar;
}

