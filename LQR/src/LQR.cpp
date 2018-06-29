/*
 * LQR.cpp
 *
 *  Created: 5 Jun 2018
 *  Author: Sebastijan Veselic
 *  Last edit: 25.6
 *  Last comment: 25.6.
 */

#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <string>
#include<fstream>
#include <cmath>
#include <iomanip>
#include <map>
#include <random>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>


/** This file actually should be in the LQRControl */

int main() {


	/** initialise the classes we need */
/*	iLQR LQR;
	Util Utility;
	Containers Data;

	/** Initialising relevant variables */

/*	int P_N, P_M;
	P_N = Data.PARAM_N;
	P_M	= Data.PARAM_M;

	/** Size of our trajectories; i.e. number of waypoints */
/*	int TrajectorySize = Data.TRAJECTORY_SIZE;

	/** Container with waypoint information filled up in GetKeys */
/*	std::vector<std::string> WaypContainer;
	/** Container with Traejctory information filled up in GetKeys */
/*	std::vector<std::string> TrajContainer;
	/** Nearest waypoint computed from NearestNeighbour */
/*	std::string NearWayp;
	/** Text file with trajectories */
/*	std::ifstream TrajFile;
	/** Final map with all waypoints */
/*	Containers::WaypointMap CompleteWaypoints; // this is the map with all the waypoints
	/** Final map with all inputs */
/*	Containers::InputMap CompleteInputs; // this is the map with all the inputs
	/** State vector representing the current state of our system. This will get read out by the system */
/*	Containers::StateVector x;

	/** User input */
/*	Containers::InputVector uUser;
	/** Prediction vector based on the user input */
/*	Containers::SystemPrediction Prediction;

	/** This will be our Wayoint counter */
/*	int Waypoint;

	/** We will set GoalState to be a certain  value. This will, however, depend on the trajectory in question
	 * The question is also what exactly the GoalState is? */
	/** GoalState is a hardcoded placeholder for now */
/*  int TrajCounter = 0;
    // double GoalState = 0.5;

    /** Here, we have our waypoint struct that is used in the K computation */
/*	Containers::WaypointSelection Waypoints[TrajectorySize];

	/** If CreateTrajectory is set to 1, we set of Waypoints with states and inputs.
	 *  DONE: Works correctly from a separate function
	 *  This will become redundant once I receive trajectory information from Golem
	 */
/*	int TrajectoryCreator = 1;
/*	Data.CreateTrajectory(TrajectoryCreator, Waypoints);

	/** Arbitrary max number of iterations for the cost function
	 * I am not sure this is even needed?
	 * Cost is minimised by solving for P at a given TimeStep, but that also doesn't make much sense given the
	 *
	 *
	 */
/*	int IterationMax = 1;

	/**
	 * These are maps where we have the 1. feedback matrix, optimal input matrix, and xDot matrix
	 * the latter two are here mostly for plotting purposes for now.
	 * The int value could also be replaced by a char/string such that we would end up with trajectory information
	 *
	 */
/*	std::map<int, Eigen::MatrixXd> FeedbackMatrixMap;
	std::map<int, Eigen::MatrixXd> OptimalInputMap;
	std::map<int, Eigen::MatrixXd> xDotMap;


	Eigen::VectorXd xNaught = Eigen::VectorXd::Random(Data.PARAM_N, Data.SINGLE);
	Containers::StateVector xBar = Eigen::VectorXd::Zero(Data.PARAM_N, Data.SINGLE);
	Eigen::VectorXd x(P_N);

	/** This is a temporary placeholder, will need to be substituted with the trajectory provided by golem */
/*	Eigen::VectorXd xBarGoal(P_N);
	Eigen::VectorXd xDot = Eigen::VectorXd::Zero(Data.PARAM_N, Data.SINGLE);

	Eigen::VectorXd uNaught = Eigen::VectorXd::Random(Data.PARAM_M, Data.SINGLE);
	Containers::InputVector uBar = Eigen::VectorXd::Zero(Data.PARAM_M, Data.SINGLE);
	Eigen::VectorXd u = Eigen::VectorXd::Random(Data.PARAM_M, Data.SINGLE);
	Eigen::VectorXd uStar = Eigen::VectorXd::Zero(Data.PARAM_M, Data.SINGLE);

	/** These remain identity matrices and are computed at each time step into their exp transforms */

/*	Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(Data.PARAM_N, Data.PARAM_N);
	Eigen::MatrixXd R = Eigen::MatrixXd::Identity(Data.PARAM_M, Data.PARAM_M);

	/** These represent the Q and R matrices at time t */
/*	Eigen::MatrixXd Qt;
	Eigen::MatrixXd Rt;



	Eigen::MatrixXd K = Eigen::MatrixXd::Random(Data.PARAM_M, Data.PARAM_N);
	Eigen::MatrixXd K_new = Eigen::MatrixXd::Random(Data.PARAM_M, Data.PARAM_N);



	Eigen::MatrixXd P = Eigen::MatrixXd::Random(Data.PARAM_N, Data.PARAM_N);


	/** What should B be exactly? In Tomasz' it was the equivalent of this */
/*	Eigen::MatrixXd B(P_N, P_M);
	B <<  1, 0,
		  0, 1,
		  0, 0;




	/** Hard code for now */
/*	Eigen::MatrixXd A(P_N, P_N);
	A <<   1,   0,  0,
		 -0.9,  1,  0,
		   0, -0.9, 0;



	/** In the loop below, it's Data.TRAJECTORY_SIZE - 1 because we want to go up to the last point before the end of the trajectory */
/*	for (int TrajPoint = 0; TrajPoint < TrajectorySize - 1; TrajPoint++)
	{
		/** We say - if you are at time t+1, change xDot into the current x such that we can use it to calculate xDot at t+2 */
/*		if (TrajPoint >= 1)
		{
			x = xDot;
		}
		/** Arbitrary large value depicting the cost in the beginning */
/*		double CurrentCost = 99999999;
		double SummedCost = 0;
		double ComputedCost = 99999998; // We start of with a high value that will make the loop initialise
										// but will prevent issued within the loop in case the ComputedCost could be too low.



//		std::cout << "We are currently on Trajectory point: " << TrajPoint << std::endl << std::endl;
		/** We iteratively solve P, K, uNaught and compute the associated cost */
/*		for (int CostIteration = 0; CostIteration < IterationMax; CostIteration++)
		{
//			if (ComputedCost < CurrentCost)
//			{
				// Rt and Qt correspond to the current Waypoint
				Rt = LQR.UpdateR(R, TrajPoint);
				Qt = LQR.UpdateQ(Q, TrajPoint);

				P_new = LQR.SolveDARE(A, B, Qt, Rt); // Drake Implementation
//				P = LQR.old_SolveDARE(A, B, Qt, Rt);

				K = LQR.ComputeK (Rt, B, P);
				K_new = LQR.ComputeK (Rt, B, P_new);

//				uStar = LQR.OptimalControl (x, Waypoints[TrajPoint].Inputs, K); // the uStar is fed into the reference change and that is fed into the CostFunction
//				LQR.ReferenceChange (uBar, xBar, Waypoints[TrajPoint].Inputs, Waypoints[TrajPoint].States, uStar, x);
//				Data.System[TrajPoint].StateBar = xBar;
//				Data.System[TrajPoint].InputBar = uBar;

//				for (int SumVar = TrajPoint; SumVar > -1; SumVar--) {
				/* could run the iteration of the cost function outside
				 * To be correct the cost function needs to compute from final to current point
				 *  We need to use TrajPoint in the computation
				 *  std::cout << "In the cost function, we are summing from TrajPoint: " << SumVar << std::endl;
				 */

//				Rt = LQR.UpdateR(R, SumVar);
//				Qt = LQR.UpdateQ(Q, SumVar);

//				ComputedCost = LQR.CostFunction (Data.System[SumVar].StateBar, Q, Qt, Data.System[SumVar].InputBar, xBarGoal, Rt, TrajPoint, GoalState, SumVar, SummedCost);
//				SummedCost += ComputedCost;
//				}

				// if the computed cost is smaller than the current cost
//				CurrentCost = SummedCost;
				// Save K for this TrajPoint and compute optimal U and after that recompute the dynamics.
/*				FeedbackMatrixMap.insert({TrajPoint, K});
	//		}
		}
//		std::cout << "TrajPoint: " << TrajPoint << " has associated total cost of: " << SummedCost << std::endl;
//		xDot = A*xBar + B*uBar;
//		OptimalInputMap.insert({TrajPoint, uStar});
//		xDotMap.insert({TrajPoint, xDot});


/*		std::cout << "xDot is: " << xDot << std::endl << std::endl;
		std::cout << "This is the feedback matrix: " << std::endl;
		std::cout << FeedbackMatrixMap[TrajPoint] << std::endl;
		std::cout << "This is the optimal input matrix: " << std::endl;
		std::cout << OptimalInputMap[TrajPoint] << std::endl;
		std::cout << "xdot matrix: " << std::endl;
		std::cout << xDotMap[TrajPoint] << std::endl;
*/	//}


	/*
	 * Quick check for size of various matrices.
	 *
		std::string Word = "TransitionMatrix";
		Utility.SizeCheck(A, Word);
	*/


/*	Here, we need code that will add additional things
 *
 * std::ofstream ofile;
	ofile.open("trajectory.txt", std::ios::app);
	for (int i = 1; i < Data.TRAJECTORY_SIZE; i++)
				{
		ofile << x << " " << y << " " << theta << " " << xDot << " " << yDot << std::endl;
		}
	ofile.close();
	};
*/
	return 0;

};

