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

struct Containers::SystemPrediction iLQR::Prediction(const Containers::TransitionMatrix &A, const Containers::ControlMatrix &B,
													 const Containers::InputVector &uUser, const Containers::StateVector &x,
													 const Containers::InputVector &TrajectoryInput, const Containers::StateVector &TrajectoryState)
{

	struct Containers::SystemPrediction Points;

        Util Utility;

	Utility.SizeCheck(A, Utility.TranMat());
	Utility.SizeCheck(B, Utility.ConMat());
	Utility.SizeCheck(uUser, Utility.InpVec());
	Utility.SizeCheck(x, Utility.StVec());
	Utility.SizeCheck(TrajectoryInput, Utility.InpVec());
	Utility.SizeCheck(TrajectoryState, Utility.StVec());

	Containers::StateVector xFiCurr, xFiNext, xHat, xFi;
	Containers::InputVector uHat;

	/* Do we first compute the uHat from the current user input
	 * and use that to predict the next state?
	 * E.g. as below:
	 */

	uHat = uUser - TrajectoryInput;

	/** Integrating current and next */
	xFiCurr = A*x + B*uUser;
	xFiNext = A*x + B*uHat;

	/** Then we just sum of both the current and the next one */
	xFi = xFiCurr + xFiNext;
	xHat = xFi - TrajectoryState;

	Points.InputHat = uHat;
	Points.StateHat = xHat;

	return Points;
}


