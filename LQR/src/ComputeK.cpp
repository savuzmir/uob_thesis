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
 * TODO: Check whether Cholesky decomposition works slower with the complete setup in place as well. 
 * DONE: Checked whether substituting .inverse() with a manual Cholesky decomposition makes it any faster. 
 * DONE: compared results with Matlab implementation  of lqr and gives same results (the only difference is due to the P matrix)
 * DONE: Compared with Drake LQR implementation
 * DONE: added matrix size checker 
 */

Eigen::MatrixXd iLQR::ComputeK (const Containers::InputCostMatrix &Rt, const Containers::ControlMatrix &B, const Containers::PMatrix &P)

{
	// Check size matrices
	Util Utility;
	Utility.SizeCheck(Rt, Utility.InpCoMat());
	Utility.SizeCheck(B, Utility.ConMat());
	Utility.SizeCheck(P, Utility.PMat());

	// Initialise local variables
	Containers::FeedbackMatrix K;
	Containers::InputCostMatrix L, RInv;

	K = Eigen::MatrixXd::Zero(Containers::PARAM_M, Containers::PARAM_N);

	/** Cholesky decomposition */

//	clock_t begin = clock();
//	L = Rt.llt().matrixL();
//	RInv = L.inverse().transpose()*L.inverse();

	/* P is obtained by SolveDARE */
	K = Rt.inverse()*B.transpose()*P;

//	clock_t end = clock();
//	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
//	std::cout << elapsed_secs << std::endl;

	return K;

}


