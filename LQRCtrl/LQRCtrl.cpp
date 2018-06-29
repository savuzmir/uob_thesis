/*
 * LQRCtrl.cpp
 *
 *  Created: 18 Jun 2018
 *  Author: Sebastijan Veselic
 *  Last edit: 28.6
 *  Last comment: 28.6.
 */

// we will first open up the file and read in the trajectories in the form of a map.

#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>
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
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <stropts.h>
#include <thread>
#include <chrono>

#include <stdio.h>
#include <termios.h>


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


/*
* for trajectory in trajectories
* for waypoint in waypoints
	 * 	compute prediction
	 * 	compute cost
	 * 	if currentcost > cost
	 * 		currentcost = cost
	 * 		save prediction
	 * 		save trajectory
	 *
	 *based on the obtained trajectory
	 *look for the K matrix of that id for that waypoint.
	 *
	 *
*/


//------------------------------------------------------------------------------


															// This saves trajectory ID and the trajectory state
void ReadFile(std::ifstream& TrajFile, Containers::WaypointMap& CompleteWaypoints, Containers::InputMap& CompleteInputs)
{

 /** Define local variables */
 std::string WaypointIdentifier;
 Containers::StateVector State;
 Containers::InputVector Input;
 double xCoord, yCoord, Theta, xDot, yDot;

 /** We simply hardcode the trajectory file we want to read from
  * This is the file created by CreateTrajectory.cpp
  */

 TrajFile.open("trajectory.txt");
 int RowCounter = 0;

 for(std::string Line; getline( TrajFile, Line); )
 {
     std::vector<std::string> Sentence;
     boost::split(Sentence, Line, [](char c){return c == ' ';});

     if (RowCounter == 0){}
     else {
     /** This is a slightly stupid way of doing it, e.g.
      * it assumes a specific format of how the input file will look.
      * This should be redone if it will be used later in a real case */
     try {
     xCoord = std::stod(Sentence[0]);
	 yCoord = std::stod(Sentence[1]);
	 Theta = std::stod(Sentence[2]);
	 xDot = std::stod(Sentence[3]);
	 yDot = std::stod(Sentence[4]);
	 	 	 	 	 	  /** TrajID,   WaypointNR */
     WaypointIdentifier = Sentence[6] + Sentence[5];
     State << xCoord,
    		   yCoord,
			   Theta;
     Input << xDot,
    		   yDot;

     CompleteWaypoints.insert({WaypointIdentifier, State});
     CompleteInputs.insert({WaypointIdentifier, Input});
     }
     catch (...)
     {std::cout << "Something went wrong in one of the lines";};

     }
     RowCounter++;
     }

 TrajFile.close();
};

void GetKeys(const Containers::WaypointMap& CompleteWaypoints, std::vector<std::string>& WaypContainer, std::vector<std::string>& TrajContainer) {
  std::string WaypointNr, CurrentTraj, TrajID;
  std::vector<std::string> TmpVector, KeyVector;

  for (auto const& Key: CompleteWaypoints)
  {
    TmpVector.push_back(Key.first);
  }

  /** We use *2 because we want to separate the trajectory ID from the waypoint */
  for (int e = 0; e < TmpVector.size(); e++)
  	  {
	  WaypointNr = "";
	  CurrentTraj = TmpVector[e]; // to be used in the appending of waypoint values
	  TrajID = CurrentTraj[0]; // to be used when added to the trajectory vector;
	  WaypointNr.append(CurrentTraj.begin()+1, CurrentTraj.end());

	  WaypContainer.push_back(TrajID); // This always pushes the first element which is the trajectory ID
	  TrajContainer.push_back(WaypointNr); // This always pushes the second element which is the waypoint number
  	  }

}

std::string NearestWaypoint(const Containers::StateVector& x, Containers::WaypointMap& CompleteWaypoints, std::vector<std::string>& WaypContainers, std::vector<std::string>& TrajContainers) {

	/*
	 * Each time we look at the "1st" and "2nd" waypoints because we will resize the waypoint containers after each computation
	 * The other way would be to have external information on the previously known waypoint so we know where to approximately look at
	*/

	 /** We first need to extract all the keys from the map */
	  std::string Identifier, TrajectoryID;
      sort(begin(WaypContainers), end(WaypContainers));
      sort(begin(TrajContainers), end(TrajContainers));

      std::vector<std::string>::iterator WaypIt, TrajIt;

      WaypIt= std::unique (WaypContainers.begin(), WaypContainers.end());
      TrajIt = std::unique (TrajContainers.begin(), TrajContainers.end());

	  WaypContainers.resize( std::distance(WaypContainers.begin(), WaypIt) );
	  TrajContainers.resize( std::distance(TrajContainers.begin(), TrajIt) );

	double SmallestDistance = 99999, CurrentDistance;

	/*
	 * Be mindful of how to best compute the current best waypoint?
	 * Might make most sense to have external information on the amount of waypoints we've
	 * crossed out so far so it starts with a reasonable guess regarding the next one
	 */
	int Wayp = 0;

	for (int Traj = 0; Traj < WaypContainers.size(); Traj++)
	  {

	  /** Here save the relevant trajectory ID and waypoint */
	  Identifier = WaypContainers[Traj] + TrajContainers[Wayp];

	  /** Compute Euclidean distance */
	  CurrentDistance = sqrt((x[0] - CompleteWaypoints[Identifier][0]) + (x[1] - CompleteWaypoints[Identifier][1]) + (x[2] - CompleteWaypoints[Identifier][2]));

	  if (CurrentDistance < SmallestDistance)
	  {
		SmallestDistance = CurrentDistance;
		/* Save trajectory id from this map */
		TrajectoryID = Identifier;
	  }
	  Identifier = "";
	  }
	return TrajectoryID;
};

// -------------------------------------------------------------------------- //

int main()
{

	/** initialise the classes we need */
	iLQR LQR;
	Util Utility;
	Containers Data;

	/** Initialising relevant variables */

	int P_N, P_M;
	P_N = Data.PARAM_N;
	P_M	= Data.PARAM_M;

	/** Size of our trajectories; i.e. number of waypoints */
	int TrajectorySize = Data.TRAJECTORY_SIZE;

	/** Container with waypoint information filled up in GetKeys */
	std::vector<std::string> WaypContainer;
	/** Container with Traejctory information filled up in GetKeys */
	std::vector<std::string> TrajContainer;
	/** Nearest waypoint computed from NearestNeighbour */
	std::string NearWayp;
	/** Text file with trajectories */
	std::ifstream TrajFile;
	/** Final map with all waypoints */
	Containers::WaypointMap CompleteWaypoints; // this is the map with all the waypoints
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
	Containers::SystemPrediction Prediction;

	/** This will be our Waypoint counter */
	int Waypoint;

	/** We will set GoalState to be a certain  value. This will, however, depend on the trajectory in question
	 * The question is also what exactly the GoalState is? */
	/** GoalState is a hardcoded placeholder for now */
    int TrajCounter = 0;
    // double GoalState = 0.5;

    /** Here, we have our waypoint struct that is used in the K computation */
	Containers::WaypointSelection Waypoints[TrajectorySize];

	/** If CreateTrajectory is set to 1, we set of Waypoints with states and inputs.
	 *  DONE: Works correctly from a separate function
	 *  This will become redundant once I receive trajectory information from Golem
	 */
	int TrajectoryCreator = 1;
	Data.CreateTrajectory(TrajectoryCreator, Waypoints);

	/**
	 * TODO: Replace the "int" in these maps such that we can now use the string that we have in GetKeys
	 * These are maps where we have the 1. feedback matrix, optimal input matrix, and xDot matrix
	 * the latter two are here mostly for plotting purposes for now.
	 * The int value could also be replaced by a char/string such that we would end up with trajectory information
	 *
	 */
	std::map<int, Eigen::MatrixXd> FeedbackMatrixMap;
	std::map<int, Eigen::MatrixXd> OptimalInputMap;
	std::map<int, Eigen::MatrixXd> xDotMap;

	Eigen::VectorXd xNaught = Eigen::VectorXd::Random(Data.PARAM_N, Data.SINGLE);
	Containers::StateVector xBar = Eigen::VectorXd::Zero(Data.PARAM_N, Data.SINGLE);

	/** This is a temporary placeholder, will need to be substituted with the trajectory provided by golem */
	Eigen::VectorXd xBarGoal(P_N);
	Eigen::VectorXd xDot = Eigen::VectorXd::Zero(Data.PARAM_N, Data.SINGLE);

	Eigen::VectorXd uNaught = Eigen::VectorXd::Random(Data.PARAM_M, Data.SINGLE);
	Containers::InputVector uBar = Eigen::VectorXd::Zero(Data.PARAM_M, Data.SINGLE);
	Eigen::VectorXd u = Eigen::VectorXd::Random(Data.PARAM_M, Data.SINGLE);
	Eigen::VectorXd uStar = Eigen::VectorXd::Zero(Data.PARAM_M, Data.SINGLE);

	/** These remain identity matrices and are computed at each time step into their exp transforms */

	Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(Data.PARAM_N, Data.PARAM_N);
	Eigen::MatrixXd R = Eigen::MatrixXd::Identity(Data.PARAM_M, Data.PARAM_M);

	/** These represent the Q and R matrices at time t */
	Eigen::MatrixXd Qt;
	Eigen::MatrixXd Rt;

	Eigen::MatrixXd K = Eigen::MatrixXd::Random(Data.PARAM_M, Data.PARAM_N);

	Eigen::MatrixXd P = Eigen::MatrixXd::Random(Data.PARAM_N, Data.PARAM_N);

	/** What should A and B be exactly?
	 * Now initialised as in Tomasz' thesis, where he mentioned how
	 * he defined them in his thesis
	 */

	Eigen::MatrixXd B(P_N, P_M);
	B <<  1, 0,
		  0, 1,
		  0, 0;

	/** Hard code for now */
	Eigen::MatrixXd A(P_N, P_N);
	A <<   1,   0,  0,
		 -0.9,  1,  0,
		   0, -0.9, 0;


	// We read the .txt file with trajectories and pack them into 2 maps. one has trajectory IDs, the other has inputs
	ReadFile(TrajFile, CompleteWaypoints, CompleteInputs);

	// We obtain the individual finalised keys from both maps
	GetKeys(CompleteWaypoints, WaypContainer, TrajContainer);

	// We compute the nearest neighbour to our current state (x) from the map of all waypoints and both containers
	NearWayp = NearestWaypoint(x, CompleteWaypoints, WaypContainer, TrajContainer);


	/** For trajectory in trajectories; for waypoints in waypoints */
	for (int Traj = 0; Traj < WaypContainer.size(); Traj++) {

		/** In the loop below, Data.TRAJECTORY_SIZE - 1 because we want to go up to the last point before the end of a trajectory */
		for (int Wayp = 0; Wayp < TrajectorySize - 1; Wayp++)
		{

			/** We iteratively solve P and K */

			// Rt and Qt correspond to the current Waypoint
			Rt = LQR.UpdateR(R, Wayp);
			Qt = LQR.UpdateQ(Q, Wayp);

			P = LQR.SolveDARE(A, B, Qt, Rt); /** The Drake implementation gives nearly identical results to my own implementation
												   * Will use this one because it's faster and has additional checks
												   */
			K = LQR.ComputeK (Rt, B, P);

	//				uStar = LQR.OptimalControl (x, Waypoints[TrajPoint].Inputs, K); // the uStar is fed into the reference change and that is fed into the CostFunction
	//				LQR.ReferenceChange (uBar, xBar, Waypoints[TrajPoint].Inputs, Waypoints[TrajPoint].States, uStar, x);
	//				Data.System[TrajPoint].StateBar = xBar;
	//				Data.System[TrajPoint].InputBar = uBar;

			/** Before we add information to the feedback matrix we need to convert it into the correct format to have a good identifier */

			Identifier = WaypContainers[Traj] + TrajContainers[Wayp];
			/** Save K for this trajectory and waypoint */
			FeedbackMatrixMap.insert({Identifier, K});
			}

//		OptimalInputMap.insert({TrajPoint, uStar});
//		xDotMap.insert({TrajPoint, xDot});

/*		std::cout << "xDot is: " << xDot << std::endl << std::endl;
		std::cout << "This is the feedback matrix: " << std::endl;
		std::cout << FeedbackMatrixMap[TrajPoint] << std::endl;
		std::cout << "This is the optimal input matrix: " << std::endl;
		std::cout << OptimalInputMap[TrajPoint] << std::endl;
		std::cout << "xdot matrix: " << std::endl;
		std::cout << xDotMap[TrajPoint] << std::endl;
*/	}

	/** This is the ID of the nearest waypoint computed each time after the user supplies us with new input */
	std::string TrajID;

	// this initialises the whole sequence
	bool repeat = true;
	int i = 1;
	while (repeat)
		{
		// when no key is hit, we are here, so this is where some of the code needs to be
	    std::cout << '*';
	    std::this_thread::sleep_for(std::chrono::milliseconds(500));

	    //	    read system state
//	    if (TrajPoint >= 1) {x = x + xDot;}

		NearWayp = NearestWaypoint(x, CompleteWaypoints, WaypContainer, TrajContainer);
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
		}



	/*
	 * BestTraj = 999;
	   CurrentCost = 99999;

	    /** We need to have a*/
		/* for (int Traj; traj values) {

/*	    Prediction = iLQR::Prediction(A, B, uUser, TrajectoryInput, TrajectoryState);

		Rt = iLQR::UpdateR(R, TrajPoint);
		Qt = iLQR::UpdateQ(Q, TrajPoint);

	    PredictionCost = iLQR::PredictionCostFunc(xHat, uHat, Rt, Qt)

	    if (CurrentCost > PredictionCost)
	    	{CurrentCost = PredictionCost;
	    	TrajCounter = Traj;}
	    }

	    TrajID = NearestWaypoint(x, CompleteWaypoints, WaypContainers, TrajContainers);
	    if (TrajID==NearWayp)
	    {
*/

	    /** Select correct trajectory + correct waypoint */

	/*    uStar = OptimalControl(x, uNaught, FeedbackMatrixMap[Waypoint]);

	    xDot = A*xBar + B*uStar;
	    }
	    else ()


//	    make prediction, select appropriate trajectory
//		check whether the nearest waypoint stays the same
//		compute optimal input
//		send optimal input with user input to update x
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






