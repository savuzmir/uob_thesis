/*
 * CostFunc.cpp
 *
 *  Created: 2 Jun 2018
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

/*
 * JGoal - penalty for not reaching the goal state
 * JTrajInput - input portion penalty for being away from uNaught at time step t
 * JTrajState - state portion penalty for being away from xNaught at time step t
 *
 *
 * DONE: Checked everything is computed correctly
 */


double iLQR::CostFunction (const Containers::StateVector &xBar, const Containers::StateCostMatrix &Q,
						   const Containers::StateCostMatrix &Qt, const Containers::InputVector &uBar, const Containers::StateVector &xBarGoal,
						   const Containers::InputCostMatrix &Rt, double TrajPoint, double GoalState, int SumVar, double SummedCost)
{

	Util Utility;

	Utility.SizeCheck(xBar, Utility.StVec());
	Utility.SizeCheck(Q, Utility.StCoMat());
	Utility.SizeCheck(Qt, Utility.StCoMat());
	Utility.SizeCheck(uBar, Utility.InpVec());
	Utility.SizeCheck(xBarGoal, Utility.StVec());
	Utility.SizeCheck(Rt, Utility.InpCoMat());

	/** Define local variables */
	double JGoal, JTrajInput, JTrajState, J;
	/** GoalW: Goal weight, TrajW: Traj weight; equally weighted in the paper. */
	double GoalW = 0.5, TrajW = 0.5;				 						
																			
	Eigen::RowVectorXd xBarT, xBarGoalT;								    
	Eigen::RowVectorXd uBarT;												
	Containers::StateCostMatrix QTransfGoal;
	QTransfGoal = Eigen::MatrixXd::Identity(Containers::PARAM_N, Containers::PARAM_N);

	// This computes the QTransf specifically for the goal state 

	for (int i = 0; i < Q.rows() - 1; i++) 
	{
		QTransfGoal(i, i) = Q(i,i)*exp(-GoalState);
	};

	// Transpose
	xBarT = xBar.transpose();
	uBarT = uBar.transpose();


	// Compute trajectory penalty
	JTrajInput = uBarT * Rt * uBar;
	JTrajState = xBarT * Qt * xBar;

	// Compute partial cost that gets returned and used as an input
	J = (JTrajInput + JTrajState);


	/* This is executed only once within the outter loop within which this is called 
	 * This is done to prevent continuous summing of the goal variable state
	 */ 
	if (SumVar == 0)
		{
	
		xBarGoalT = xBarGoal.transpose();

		JGoal = (xBarGoalT * QTransfGoal * xBarGoal);
		JGoal = JGoal*GoalW;

		// Here we take the current J + the previously computed trajectory cost, weight it and add it to the final cost 
		J = JGoal + (J + SummedCost)*TrajW;
		}


	return J;

}
