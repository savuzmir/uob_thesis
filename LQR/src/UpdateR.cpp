/*
 * UpdateR.cpp
 *
 *  Created on: 12 Jun 2018
 *  Author: Sebastijan Veselic
 *  Last edit: 18.6
 *  Last comment: 18.6.
 */

#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <string>
#include <stdio.h>

// TODO: Matrix size check

Containers::InputCostMatrix  iLQR::UpdateR(const Containers::InputCostMatrix &R, double TrajPoint)
{

	Util Utility;
	Utility.SizeCheck(R, Utility.InpCoMat());
	Containers::InputCostMatrix RTransfTraj;
	RTransfTraj = Eigen::MatrixXd::Identity(Containers::PARAM_M, Containers::PARAM_M);

	for (int i = 0; i < R.rows(); i++)
	{
		RTransfTraj(i,i) = R(i,i)*exp(-TrajPoint);
	};


	return RTransfTraj;
}
