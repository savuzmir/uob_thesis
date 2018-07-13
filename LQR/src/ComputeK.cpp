/*
 * ComputeK.cpp
 *
 *  Created: 5 Jun 2018
 *  Author: Sebastijan Veselic
 *  Last edit: 28.6
 *  Last comment: 28.6.
 */

#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <stdio.h>
#include <math.h>
#include <ctime>

  

/* 
 * TODO: Ensure the matrix size checker works correctly within this function

 */

void iLQR::ComputeK (const Containers::InputCostMatrix &Rt, const Containers::ControlMatrix &B,
					 const Containers::PMatrix &P, Containers::FeedbackMatrix &K,
					 const Containers::TransitionMatrix &A, const int Discrete)

{
	// Check size matrices
	Util Utility;
	Utility.SizeCheck(Rt, Utility.InpCoMat());
	Utility.SizeCheck(B, Utility.ConMat());
	Utility.SizeCheck(P, Utility.PMat());

    if (Discrete == 1)
    {
    	Eigen::MatrixXd tmp = B.transpose() * P * B + Rt;
    	K = tmp.llt().solve(B.transpose() * P * A);
    }
    else {
        Eigen::LLT<Eigen::MatrixXd> R_cholesky(Rt);
        K = R_cholesky.solve(B.transpose() * P);}

}


