/*
 * UpdateQ.cpp
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

Containers::StateCostMatrix  iLQR::UpdateQ(const Containers::StateCostMatrix &Q, double TrajPoint)
{

	Util Utility;

	Utility.SizeCheck(Q, Utility.StCoMat());
	Containers::StateCostMatrix QTransfTraj;
	QTransfTraj = Eigen::MatrixXd::Identity(Containers::PARAM_N, Containers::PARAM_N);

	for (int i = 0; i < Q.rows() - 1; i++) 
	{
		QTransfTraj(i,i) = Q(i,i)*exp(-TrajPoint);
	};

	return QTransfTraj;
}
