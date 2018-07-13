/*
 * LQRCtrl.cpp
 *
 *  Created: 18 Jun 2018
 *  Author: Sebastijan Veselic
 *  Last edit: 7.7.
 *  Last comment: 7.7.
 */

#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>

/** Why doesn't the include statement add these .cpp files? */
#include </home/sebastijan/eclipse-workspace/LQR/src/CreateTrajectory.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/UpdateQ.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/UpdateR.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/SolveDARE.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/SolveCARE.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/ComputeK.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/Prediction.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/PredictionCostFunc.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/OptimalControl.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/util.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/ReferenceChange.cpp>


#include<fstream>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <sstream>
#include <map>
#include <stropts.h>
#include <thread>
#include <chrono>
#include <locale>

#include <SFML/Graphics.hpp>

int main()
{

	/** Do you want to save the data for plotting [0 - no, 1 -yes] */
	int SaveFiles = 1;

	/** initialise the classes we need */
	iLQR LQR;
	Util Utility;
	Containers Data;
	FileParser FileParse;
	WaypointCompute WaypComp;
	InputSFML Input;

	Containers::WIDTH Width = sf::VideoMode::getDesktopMode().width;
	Containers::HEIGHT Height = sf::VideoMode::getDesktopMode().height;
	Containers::NORM_CONST NormFactor = 0.5;

	/** these are same data structures (i.e. a vector of strings) but I use 2 different typedefs for clarity purposes */
	Containers::WaypSeq::const_iterator WaypIt;
	Containers::TrajSeq::const_iterator TrajIt;

	/** Initialising relevant variables */
	/** Size of our trajectories; i.e. number of waypoints */
	int TrajectorySize = Data.TRAJECTORY_SIZE;

	/** Define both */
	double PredictionCost = 9e100;
	double CurrentCost = 9e100;
	double TempWayp;

	/** Container with waypoint information filled up in GetKeys */
	Containers::WaypSeq WaypContainer;

	/** This stores the history of one's path */
	Containers::WaypSeqHist UserHistory;
	Containers::WaypSeqHist UserHistoryStorage;
	Containers::InputHistory InputHistory;
	Containers::InputHistory OptimalInputHistory;
	Containers::SystemHistory StateHistory;

	/** Container with trajectory information filled up in GetKeys */
	Containers::TrajSeq TrajContainer;

	/** Nearest waypoint computed from NearestNeighbour */
	std::string NearWayp = "0";
	/** Text file with trajectories */
	std::ifstream TrajFile;

	/** Final map with all waypoints */
	Containers::WaypMap CompleteWaypoints; // this is the map with all the waypoints
	/** Final map with all inputs */
	Containers::InputMap CompleteInputs; // this is the map with all the inputs

	/** State vector representing the current state of our system. This will get read out by the system */
	Containers::StateVector x;

	x << 0.1,
		 0.2,
		 0.3;

	/** User input */
	Containers::InputVector uUser;
	/** Prediction vector based on the user input */
	Containers::SystemPrediction SystemHat;

	/** Helper variable that is used as a key */
	std::string Identifier, PredIdentifier;

	/** We will set GoalState to be a certain  value. This will, however, depend on the trajectory in question
	 * The question is also what exactly the GoalState is? */
	/** GoalState is a hardcoded placeholder for now */
    std::string OptTraj, OptWaypoint;


    /** Here, we have our waypoint struct that is used in the K computation */
	Containers::WaypSelect Waypoints[TrajectorySize];

	/** If CreateTrajectory is set to 1, we set of Waypoints with states and inputs.
	 *  DONE: Works correctly from a separate function
	 *  This will become redundant once I receive trajectory information from Golem
	 */
	int TrajectoryCreator = 1;
	Data.CreateTrajectory(TrajectoryCreator, Waypoints);

	Containers::KMap FeedbackMatrixMap;
	Containers::uStarMap OptimalInputMap;
	Containers::xDotMap StateChangeMap;

	Containers::StateVector xNaught, xBar, xBarGoal, xDot;
	/** This is a temporary placeholder, will need to be substituted with the trajectory provided by golem */

	Containers::InputVector uNaught, uBar, u, uStar;

	/** These remain identity matrices and are computed at each time step into their exp transforms */

	Containers::StateCostMatrix Q = Eigen::MatrixXd::Identity(Data.PARAM_N, Data.PARAM_N);
	Containers::InputCostMatrix R = Eigen::MatrixXd::Identity(Data.PARAM_M, Data.PARAM_M);

	/** These represent the Q and R matrices at time t */
	Containers::StateCostMatrix Qt;
	Containers::InputCostMatrix Rt;

	Containers::FeedbackMatrix K;
	Containers::PMatrix P;

	/** What should A and B be exactly?
	 * Now initialised as in Tomasz' thesis, where he mentioned how
	 * he defined them in his thesis
	 */

	Containers::ControlMatrix B;
	B <<  1, 0,
		  0, 1,
		  0, 0;

	/** Hard code for now */
	Containers::TransitionMatrix A;
	A <<   1,   0,  0,
		   -0.9,   1,  0,
		   0,   -0.9,  0;

	// We read the .txt file with trajectories and pack them into 2 maps. one has trajectory IDs, the other has inputs
	FileParse.ReadFile(TrajFile, CompleteWaypoints, CompleteInputs);

	// We obtain the individual finalised keys from both maps
	FileParse.GetKeys(CompleteWaypoints, WaypContainer, TrajContainer);

	// This prepares containers for computation
	WaypComp.PrepareContainers(WaypContainer, TrajContainer);

	// ##################################################################################### //
	// ######################### Compute feedback matrices ################################# //
	// ##################################################################################### //

	/** Are you using the discrete or continuous K computation */
	int Discrete = 0;

	/** For trajectory in trajectories; for waypoints in waypoints */
	for(TrajIt = TrajContainer.begin(); TrajIt != TrajContainer.end(); TrajIt++) {

		/** Recheck whether we want to iterate until the last element */
		for(WaypIt = WaypContainer.begin(); WaypIt != WaypContainer.end(); WaypIt++) {

			/** We iteratively solve P and K, Rt corresponds to the time (i.e. waypoint) parametrization */
			Rt = LQR.UpdateR(R, *WaypIt);
			Qt = LQR.UpdateQ(Q, *WaypIt);
			P = LQR.SolveCARE(A, B, Qt, Rt); //P_dare = LQR.SolveDARE(A, B, Qt, Rt);
			LQR.ComputeK (Rt, B, P, K, A, Discrete);

			/** K matrix for each ID: trajectory + waypoint (i.e. 'A15') is saved to a map */
			Identifier = *TrajIt + std::to_string(*WaypIt);
			FeedbackMatrixMap.insert({Identifier, K});
		}
	std::cout << "Computed K matrices for trajectory: " << *TrajIt << std::endl;

	}
	std::cout << "Computed all the K matrices." << std::endl;


	// ##################################################################################### //
	// ######################### Receive user input ######################################## //
	// ##################################################################################### //

	int i = 0;
	bool repeat = true;
	while (i < 12) // this should be while the euclidean distance of the system is far from the goal state
		{
		// when no key is hit, we are here, so this is where some of the code needs to be
	    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	    /** Read system state */
	    /** In the later version we should get this from Golem */
	    /* TrajPoint should be here somehow defined in such a way where we should be changing the waypoint that is being observed after each input */
		/* This will likely be redundant once we will be actually reading the system state from the Golem environment */

		if (std::stoi(NearWayp) >= 1)

	    {
	    	x = xDot;
	    	std::cout << "System state: " << std::endl << x << std::endl << std::endl;
	    }

		// We compute the nearest neighbour to our current state (x) from the map of all waypoints and both containers
		NearWayp = WaypComp.NearestWaypoint(x, CompleteWaypoints, WaypContainer, TrajContainer, UserHistory, NearWayp);

		std::cout << "Found nearest neighbour: " << NearWayp << std::endl;

		/** this returns data from the corresponding function */
		uUser = Input.UserInput(Width, Height, uUser, NormFactor);

		/** convert into appropriate format */

		/* for current waypoint */
	    /** look through all trajectories */

		/* convert the NearWayp to a double */
		TempWayp = std::stoi(NearWayp);

		// ##################################################################################### //
		// ######################### Select best trajectory based on input ##################### //
		// ##################################################################################### //


		for(TrajIt = TrajContainer.begin(); TrajIt != TrajContainer.end(); TrajIt++)
		{

			PredIdentifier = *TrajIt + NearWayp;

			LQR.Prediction(A, B, uUser, x, CompleteWaypoints[PredIdentifier], CompleteInputs[PredIdentifier], SystemHat);
			Rt = LQR.UpdateR(R, TempWayp);
			Qt = LQR.UpdateQ(Q, TempWayp);

			LQR.PredictionCostFunc(SystemHat.StateHat, SystemHat.InputHat, Rt, Qt, PredictionCost);

			std::cout << "The cost for trajectory point: " << PredIdentifier << " is: " << PredictionCost << std::endl;
			std::cout << "It extracted the trajectory state: " << std::endl << CompleteWaypoints[PredIdentifier] << std::endl;
			std::cout << "It extracted the trajectory input: " << std::endl << CompleteInputs[PredIdentifier] << std::endl;
			std::cout << "This was the prediction for the state: " << std::endl << SystemHat.StateHat << std::endl;
			std::cout << "This was the prediction for the input: " << std::endl << SystemHat.InputHat << std::endl;

			std::cout << "This is the current cost: " << CurrentCost << std::endl << std::endl;
			std::cout << "This is the prediction cost: " << PredictionCost << std::endl << std::endl;

			if (CurrentCost > PredictionCost)
				{CurrentCost = PredictionCost;
				OptTraj = *TrajIt;} // optimal trajectory
			}

	    std::cout << "The optimal trajectory: " << OptWaypoint << " has a cost of: " << CurrentCost << std::endl << std::endl;

		CurrentCost = 9e100;
		OptWaypoint = OptTraj + NearWayp;

	    UserHistory.push_back(OptTraj);
	    UserHistoryStorage.push_back(OptWaypoint);

	    /** Select correct trajectory + correct waypoint */

	    std::cout << "Before rereferencing, user input: " << std::endl << uUser << std::endl << std::endl;
	    std::cout << "K matrix: " << std::endl << FeedbackMatrixMap[OptWaypoint] << std::endl << std::endl;

	    LQR.OptimalControl(x, uUser, FeedbackMatrixMap[OptWaypoint], uStar);

	    std::cout << "Before rereferencing, optimal control: " << std::endl << uStar << std::endl << std::endl;

	    LQR.ReferenceChange (uBar, xBar, CompleteInputs[OptWaypoint], CompleteWaypoints[OptWaypoint], uStar, x);

	    std::cout << "After rereferencing, state: " << std::endl << xBar << std::endl << std::endl;
	    std::cout << "After rereferencing, input: " << std::endl << uBar << std::endl << std::endl;




		InputHistory.push_back(uUser);
		OptimalInputHistory.push_back(uStar);
		StateHistory.push_back(x);

	    xDot = A*xBar + B*uBar;

	    std::cout << "This is solution to the state space equation: " << std::endl << xDot << std::endl << std::endl;

	    std::cout << "|=======================================================================|" << std::endl;

	    /* Add optimal control to the dataset that will be written */


	    i += 1;
	    }

	/*	Here, we need code that will add additional things */

	if (SaveFiles == 1) {

	std::ofstream ofile;
	ofile.open("UserHistory.txt", std::ios::app);
	for (Containers::WaypSeqHist::const_iterator el = UserHistoryStorage.begin(); el != UserHistoryStorage.end(); el++)
		{
		ofile << *el << std::endl;
		}
		ofile.close();

	std::ofstream ffile;
	ffile.open("InputHistory.txt", std::ios::app);
	for (Containers::InputHistory::iterator el1 = InputHistory.begin(); el1 != InputHistory.end(); el1++)
		{
		ffile << (*el1)[0] << " " << (*el1)[1] << std::endl;
		}
		ffile.close();

	std::ofstream sfile;
	sfile.open("OptimalInputHistory.txt", std::ios::app);
	for (Containers::InputHistory::iterator el2 = OptimalInputHistory.begin(); el2 != OptimalInputHistory.end(); el2++)
		{
		sfile << (*el2)[0] << " " << (*el2)[1] << std::endl;
		}
		sfile.close();

	std::ofstream dfile;
	dfile.open("StateHistory.txt", std::ios::app);
	for (Containers::SystemHistory::iterator el3 = StateHistory.begin(); el3 != StateHistory.end(); el3++)
		{
		dfile << (*el3)[0] << " " << (*el3)[1] << " " << (*el3)[2] << std::endl;
		}
		dfile.close();
	}

return 0;
}


//	clock_t begin = clock();

//	clock_t end = clock();
//	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
//	std::cout << elapsed_secs << std::endl;


