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
#include <sys/select.h>
#include <sys/ioctl.h>
#include <stropts.h>
#include <thread>
#include <chrono>
#include <SFML/Graphics.hpp>
#include <termios.h>
#include <locale>


/** Custom khbit() and getchar() implementation for Linux obtained from:
 * https://stackoverflow.com/questions/7469139/what-is-equivalent-to-getch-getche-in-linux
 * https://stackoverflow.com/questions/29335758/using-kbhit-and-getch-on-linux
 * */

int LinuxKbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

static struct termios old, nw;

/* Initialize new terminal i/o settings */
void initTermios(int echo)
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  nw= old; /* make new settings same as old settings */
  nw.c_lflag &= ~ICANON; /* disable buffered i/o */
  if (echo) {
	  nw.c_lflag |= ECHO; /* set echo mode */
  } else {
	  nw.c_lflag &= ~ECHO; /* set no echo mode */
  }
  tcsetattr(0, TCSANOW, &nw); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void)
{
  tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo)
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void)
{
  return getch_(0);
}

/* Read 1 character with echo */
char getche(void)
{
  return getch_(1);
}


/** End of custom implementation */



//------------------------------------------------------------------------------




int main()
{
	/** initialise the classes we need */
	iLQR LQR;
	Util Utility;
	Containers Data;
	FileParser FileParse;
	WaypointCompute WaypComp;

	/** these are same data structures (i.e. a vector of strings) but I use 2 different typedefs for clarity purposes */
	Containers::WaypSeq::const_iterator WaypIt;
	Containers::TrajSeq::const_iterator TrajIt;

	/** Initialising relevant variables */
	/** Size of our trajectories; i.e. number of waypoints */
	int TrajectorySize = Data.TRAJECTORY_SIZE;

	double PredictionCost, TempWayp;

	/** Define both */
	double BestTraj = 999;
	double CurrentCost = 99999;


	/** Container with waypoint information filled up in GetKeys */
	Containers::WaypSeq WaypContainer;

	/** This stores the history of one's path */
	Containers::WaypSeqHist UserHistory;

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
		 -0.9,  1,  0,
		   0, -0.9, 0;

	// We read the .txt file with trajectories and pack them into 2 maps. one has trajectory IDs, the other has inputs
	FileParse.ReadFile(TrajFile, CompleteWaypoints, CompleteInputs);

	// We obtain the individual finalised keys from both maps
	FileParse.GetKeys(CompleteWaypoints, WaypContainer, TrajContainer);

	// This prepares containers for computation
	WaypComp.PrepareContainers(WaypContainer, TrajContainer);

	// ##################################################################################### //
	// ######################### Compute feedback matrices ################################# //
	// ##################################################################################### //

	/** For trajectory in trajectories; for waypoints in waypoints */
	for(TrajIt = TrajContainer.begin(); TrajIt != TrajContainer.end(); TrajIt++) {


		/** Recheck whether we want to iterate until the last element */
		for(WaypIt = WaypContainer.begin(); WaypIt != WaypContainer.end(); WaypIt++) {

			/** We iteratively solve P and K */
			// Rt and Qt correspond to the current Waypoint
			Rt = LQR.UpdateR(R, *WaypIt);
			Qt = LQR.UpdateQ(Q, *WaypIt);
			P = LQR.SolveDARE(A, B, Qt, Rt);
			LQR.ComputeK (Rt, B, P, K);

			/** Before we add information to the feedback matrix we need to convert it into the correct format to have a good identifier */
			/** We add identifier information that is used to add it into the map */
			Identifier = *TrajIt + std::to_string(*WaypIt);
			/** Save K for this trajectory and waypoint */
			FeedbackMatrixMap.insert({Identifier, K});
		}
	std::cout << "Computed K matrices for trajectory: " << *TrajIt << std::endl;

	}
	std::cout << "Computed all the K matrices." << std::endl;


	// ##################################################################################### //
	// ######################### Receive user input ######################################## //
	// ##################################################################################### //


	// TODO: Put into external function and replace with the SFLM library

	bool repeat = true;
	int i = 1;
	while (repeat)
		{
		// when no key is hit, we are here, so this is where some of the code needs to be
	    std::cout << '*';
	    std::this_thread::sleep_for(std::chrono::milliseconds(500));

	    /** Read system state */
	    /** In the later version we should get this from Golem */
	    /* TrajPoint should be here somehow defined in such a way where we should be changing the waypoint that is being observed after each input */
		/* This will likely be redundant once we will be actually reading the system state from the Golem environment */

		if (std::stoi(NearWayp) >= 1)

	    {
	    	x = x + xDot;
	    	std::cout << "System value: " << x << std::endl << std::endl;
	    }

		std::cout << "First wayp: " << NearWayp << std::endl;

		// We compute the nearest neighbour to our current state (x) from the map of all waypoints and both containers
		NearWayp = WaypComp.NearestWaypoint(x, CompleteWaypoints, WaypContainer, TrajContainer, UserHistory, NearWayp);

		std::cout << "Wayp after computation: " << NearWayp << std::endl;

	    if (LinuxKbhit()) // Checks for keyhits. if it records one it goes into the loop.
	    {
	    char ch = getch();
	            switch (ch)
	            {
	                case 27:               // press ESC to exit
	                    repeat = false;
	                    break;

	                /** W */
	                case 119:
	                	std::cout << "forward" << std::endl;
	                	uUser << 0.5,
	                			 0;
	                	break;

	                /** E */
	                case 101:
	                	std::cout << "diagonal right" << std::endl;
	                	uUser << 0.5,
	                		     -0.5;
	                	break;

	                /** D */
	                case 100:
	                	std::cout << "right" << std::endl;
	                	uUser <<  0,
	                		     -0.5;
	                	break;

	                /** Q */
	                case 113:
	                	std::cout << "diagonal left" << std::endl;
	                	uUser << 0.5,
	                		     0.5;
	                	break;

	                /** A */
	                case 97:
	                	std::cout << "left" << std::endl;
	                	uUser << 0,
	                		     0.5;
	                	break;
	            }
	        }

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

			if (CurrentCost > PredictionCost)
				{CurrentCost = PredictionCost;
				OptTraj = *TrajIt;} // optimal trajectory
			}
		BestTraj = 999;
		CurrentCost = 99999;

		OptWaypoint = OptTraj + NearWayp;
	    UserHistory.push_back(OptTraj);

	    std::cout << "The optimal trajectory: " << OptWaypoint << std::endl << std::endl;
	    /** Select correct trajectory + correct waypoint */
	    LQR.OptimalControl(x, uUser, FeedbackMatrixMap[OptWaypoint], uStar);
		LQR.ReferenceChange (uBar, xBar, CompleteInputs[OptWaypoint], CompleteWaypoints[OptWaypoint], uStar, x);


	    /* we compute current - optimal
	     * for both system and ipnut
	     */

	    xDot = A*xBar + B*uStar;


	    }

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


/*	OptimalInputMap[TrajPoint] << std::endl;
	xDotMap[TrajPoint] << std::endl;
*/

	// DONE - checked to get keyboard information
	/** Check the ASCII value

	 char c;
	 std::cout << "Enter a character: ";
	 std::cin >> c;
	 std::cout << "ASCII Value of " << c << " is " << int(c);

	  *  W - 119
	  *  E - 101
	  *  D - 100
	  *  Q - 113
	  *  A - 97
	  *  */
return 0;
}






