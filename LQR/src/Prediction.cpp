/*
 * Prediction.cpp
 *
 *  Created: 14 Jun 2018
 *  Author: Sebastijan Veselic
 *  Last edit: 25.6
 *  Last comment: 25.6.
 */


/** we get the user input in the state space by summing from t to t  + 1
 *
 *
 * TODO: How does this integration exactly occur? I don't think it's really the way I did it below. 
 *
 */

#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>
#include <Eigen/Dense>
#include <Eigen/Core>

void iLQR::Prediction(const Containers::TransitionMatrix &A, const Containers::ControlMatrix &B,
													 const Containers::InputVector &uUser, const Containers::StateVector &x,
													 const Containers::StateVector &TrajectoryState, const Containers::InputVector &TrajectoryInput,
													 struct Containers::SystemPrediction &Points)
{

//	struct Containers::SystemPrediction Points;

        Util Utility;

	Utility.SizeCheck(A, Utility.TranMat());
	Utility.SizeCheck(B, Utility.ConMat());
	Utility.SizeCheck(uUser, Utility.InpVec());
	Utility.SizeCheck(x, Utility.StVec());
	Utility.SizeCheck(TrajectoryInput, Utility.InpVec());
	Utility.SizeCheck(TrajectoryState, Utility.StVec());

	Containers::StateVector xHat, xPhi;
	Containers::InputVector uHat;

	/* Do we first compute the uHat from the current user input
	 * and use that to predict the next state?
	 * E.g. as below:
	 */

	uHat = uUser - TrajectoryInput;

	/** Integrating current and next */
	xPhi = A*x + B*uHat;

	xHat = xPhi - TrajectoryState;

	Points.InputHat = uHat;
	Points.StateHat = xHat;

}


