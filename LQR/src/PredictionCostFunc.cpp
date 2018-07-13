/*
 * PredictionCostFunc.cpp
 *
 *  Created: 12 Jun 2018
 *  Author: Sebastijan Veselic
 *  Last edit: 25.6
 *  Last comment: 25.6.
 */

#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>
#include <Eigen/Dense>
#include <Eigen/Core>

void iLQR::PredictionCostFunc(const Containers::StateVector &xHat, const Containers::InputVector &uHat,
							  const Containers::InputCostMatrix &Rt, const Containers::StateCostMatrix &Qt,
							  double &PredictionCost)
{

    Util Utility;

	Utility.SizeCheck(xHat, Utility.StVec());
	Utility.SizeCheck(uHat, Utility.InpVec());

	Utility.SizeCheck(Rt, Utility.InpCoMat());
	Utility.SizeCheck(Qt, Utility.StCoMat());

	double StateCost, InputCost;
	Eigen::RowVectorXd xHatT;
	Eigen::RowVectorXd uHatT;

	uHatT = uHat.transpose();
	xHatT = xHat.transpose();

/*	std::cout << "||||==================================================||||" << std::endl;
	std::cout << "this is the uhat transpose: " << std::endl << uHatT << std::endl;
	std::cout << "this is the xhat transpose: " << std::endl << uHatT << std::endl;
	std::cout << "this is the uhat : " << std::endl << uHat << std::endl;
	std::cout << "this is the xhat : " << std::endl << xHat << std::endl;
	std::cout << "this is the Rt : " << std::endl << Rt << std::endl;
	std::cout << "this is the Qt : " << std::endl << Qt << std::endl;
*/
	StateCost = xHatT*Qt*xHat;
	InputCost = uHatT*Rt*uHat;

	PredictionCost = StateCost + InputCost;
}
