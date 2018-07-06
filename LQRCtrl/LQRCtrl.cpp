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

/** Why doesnt the include statement add these .cpp files? */
#include </home/sebastijan/eclipse-workspace/LQR/src/CreateTrajectory.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/UpdateQ.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/UpdateR.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/SolveDARE.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/ComputeK.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/Prediction.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/PredictionCostFunc.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/OptimalControl.cpp>
#include </home/sebastijan/eclipse-workspace/LQR/src/util.cpp>


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


															// This saves trajectory ID and the trajectory state
void ReadFile(std::ifstream& TrajFile, Containers::WaypMap& CompleteWaypoints, Containers::InputMap& CompleteInputs)
{

 /** Define local variables */
 std::string WaypointIdentifier;
 Containers::StateVector State;
 Containers::InputVector Input;
 double xCoord, yCoord, Theta, xDot, yDot;

 /** We simply hardcode the trajectory file we want to read from
  * This is the file created by CreateTrajectory.cpp
  */

 TrajFile.open("Trajectory.txt");
 int RowCounter = 0;

 for(std::string Line; getline(TrajFile, Line); )
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
 std::cout << "Successfully Imported the trajectory information." << std::endl;
};

void GetKeys(const Containers::WaypMap& CompleteWaypoints, Containers::WaypSeq& WaypContainer, Containers::TrajSeq& TrajContainer) {
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

	std::cout << "Trajectory ID and waypoint numbers are saved." << std::endl;
}

// we send the history of the user sequences into this function together with the map of the current best
// options. we return the best option  
std::string HistorySeqCheck(Containers::WaypSeqHist& UserHistory, const Containers::NearNeighMap& PotentialCandidates)
{
	// TODO: Make names nicer. otherwise this works correctly

	std::locale loc;
	std::string::const_iterator CandidateEl;
	std::string CurrentTraj;
	std::string CandidateKey;
	std::string BestSolution;
	std::string TemporarySave;
	std::string tempstring;

//	UserHistory = {"B", "C", "A"};

	// i'll make it like this such that we are able to play with different settings
	// for now, look only at the last element
	/* all potential candidate. if there is more of them, look at the previous user history and take the one that's inline. if there is one, just take it */
	for (auto const& Candidate : PotentialCandidates)
	{	/* if the filtered trajectory is the same as the last one */
		CandidateKey = Candidate.first;
		for (CandidateEl = CandidateKey.begin(); CandidateEl != CandidateKey.end(); CandidateEl++)
			 {	 /* If the character is an alphabetic character (i.e. is not a number) */
				 /* we add it to the string we are currently building. the string we are building is the trajectory representation */

				 if (std::isalpha(*CandidateEl, loc))
				 {
					 CurrentTraj.push_back(*CandidateEl);
				 } /* we are interested just in the case where it is the same as the current one. if the trajectory one is moving to hasnt been recorded in the past
				    * it most likels means the person is deliberately moving to a place later along the trajectory */
				 if ((UserHistory.size()> 1) && (UserHistory.back() == CurrentTraj))
				 /* this will add the current trajectory information into a new container that will be returned */
				 {BestSolution = Candidate.first;
				  break;}
				 else if (UserHistory.empty()) /* this will make a random the best solution if the user history is empty. this can only happen if the person has just started */
				 {BestSolution = Candidate.first;
				  break;}
				 CurrentTraj = "";
			 }
		TemporarySave = Candidate.first;
		}

	/* if that procedure yields and empty BestSolution, we arbitrarily pick the last candidate of the loop
	 * it shouldnt matter which one is picked considering the cost of all of them is very low and comparable
	 * as this procedure wold yield an empty BestSolution only in the case when any of the offered solutions was not present in the last element of the user History */

	if (BestSolution.empty())
	{
		BestSolution = TemporarySave;
	}
	return BestSolution;
	}


std::string NearestWaypoint(const Containers::StateVector& x,
							Containers::WaypMap& CompleteWaypoints,
							Containers::WaypSeq& WaypContainer,
							Containers::TrajSeq& TrajContainer,
							Containers::WaypSeqHist& UserHistory,
							std::string NearWayp)
	{
	// TODO: make names nicer. otherwise this works correctly
	/*
	 * Each time we look at the "1st" and "2nd" waypoints because we will resize the waypoint containers after each computation
	 * The other way would be to have external information on the previously known waypoint so we know where to approximately look at
	*/

	 /** We first need to extract all the keys from the map */
	  std::string Identifier, TrajectoryID, WaypID;
	  Containers::TrajSeq::const_iterator trajit, waypit;
	  Containers::NearNeighMap CurrNeighs;
	  Containers::NearNeighMap PotentialCandidates;
	  Containers::TrajSeq TrajHelperContainer;
	  double LowestCost = 999999;
	  std::string LowestCostTraj;
	  int Wayp;

	  Wayp = std::stod(NearWayp);

      sort(begin(WaypContainer), end(WaypContainer));
      sort(begin(TrajContainer), end(TrajContainer));

      std::vector<std::string>::iterator WaypIt, TrajIt;

      WaypIt= std::unique (WaypContainer.begin(), WaypContainer.end());
      TrajIt = std::unique (TrajContainer.begin(), TrajContainer.end());

	  WaypContainer.resize( std::distance(WaypContainer.begin(), WaypIt) );
	  TrajContainer.resize( std::distance(TrajContainer.begin(), TrajIt) );

	double SmallestDistance = 99999, CurrentDistance;


/*	std::cout << "this is the x: " << x << std::endl << std::endl;
/*	for (auto const& kv : CompleteWaypoints)
	{std::cout << "these are the completewaypoints: " << kv.first << "and " << kv.second << std::endl << std::endl;}
*/
	for(waypit = TrajContainer.begin(); waypit != TrajContainer.end(); waypit++)
	{std::cout << "this is the trajcontainer: " << *waypit << std::endl << std::endl;}
/*
	for(trajit = WaypContainer.begin(); trajit != WaypContainer.end(); trajit++)
	{std::cout << "this is the waypcontainer: " << *trajit << std::endl << std::endl;}

	for(Containers::WaypSeqHist::const_iterator userit = UserHistory.begin(); userit != UserHistory.end(); userit++)
	{std::cout << "this is the user history : " << *userit << std::endl << std::endl;}

*/

	/*
	 * Be mindful of how to best compute the current best waypoint?
	 * Might make most sense to have external information on the amount of waypoints we've
	 * crossed out so far so it starts with a reasonable guess regarding the next one
	 */

	// here we iterate through  the first element of all trajectories //
	for (trajit = WaypContainer.begin(); trajit != WaypContainer.end(); trajit++)

	  {

	  std::cout << TrajContainer[Wayp];
	  /** Here save the relevant trajectory ID and waypoint */
	  /** This always returns the first Wayp? */
	  Identifier =  *trajit + TrajContainer[Wayp];

	  /** Compute Euclidean distance - this should also be changed to make it work for more than 3 states */
	  CurrentDistance = sqrt(abs((x[0] - CompleteWaypoints[Identifier][0])) + abs((x[1] - CompleteWaypoints[Identifier][1])) + abs((x[2] - CompleteWaypoints[Identifier][2])));

	  CurrNeighs.insert({Identifier, CurrentDistance});
	  }

	  double tol = 1e-3; // tolerance value defining the minimum allowed difference between two costs

	  // we now have a map with a single element and its associated cost.
	  // we will now go through the original map of neighbours again and check
	  int helpercounter = 0;
	  for (auto const& currneighkv : CurrNeighs)

	  {
		  if (currneighkv.second < LowestCost)
		  {
		  LowestCostTraj = currneighkv.first;
		  LowestCost = currneighkv.second;
		  }
	  }

	  PotentialCandidates.insert({LowestCostTraj, LowestCost});

	// we can take just the lowestcostvalue
	  for (auto const& comparisonkv : CurrNeighs)
	  {
	  if (comparisonkv.second - LowestCost < tol)
	  {
		  PotentialCandidates.insert({comparisonkv.first, comparisonkv.second}); // this will add additional neighbours that differ from the lowest one by a certain level
	  } // now we have a list of potential candidates
	  }

	  // if the size of potentialcandidates is bigger than 1  we need to select the one that's in line with the previous trajectory
	  if (PotentialCandidates.size() > 1)
	  {
		  // now we check them
		  TrajectoryID = HistorySeqCheck(UserHistory, PotentialCandidates);
	  }
	  // we simply take the lowest one from the inner loop
	  else {TrajectoryID = LowestCostTraj;};

	  // from this trajectory we extract the waypoint itself
	  for (std::string::const_iterator WaypExtractor = TrajectoryID.begin(); WaypExtractor != TrajectoryID.end(); WaypExtractor++)
	  {	 /* If the character is an alphabetic character (i.e. is not a number) */
				 /* we add it to the string we are currently building. the string we are building is the trajectory representation */
		  if (std::isdigit(*WaypExtractor))
		  {
			  WaypID.push_back(*WaypExtractor);
		  }
	  }

	return WaypID;
};

// -------------------------------------------------------------------------- //


int main()
{

	/** initialise the classes we need */
	iLQR LQR;
	Util Utility;
	Containers Data;

	/** these are same data structures (i.e. a vector of strings) but I use 2 different typedefs for clarity purposes */
	Containers::WaypSeq::const_iterator waypit;
	Containers::TrajSeq::const_iterator trajit;

	/** Initialising relevant variables */

	int P_N, P_M;
	P_N = Data.PARAM_N;
	P_M	= Data.PARAM_M;

	/** Size of our trajectories; i.e. number of waypoints */
	int TrajectorySize = Data.TRAJECTORY_SIZE;

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


	Containers::StateVector WaypointArgument;
	Containers::InputVector InputArgument;

	/** State vector representing the current state of our system. This will get read out by the system */
	Containers::StateVector x;

	x << 0.1,
		 0.2,
		 0.3;

	/** User input */
	Containers::InputVector uUser;
	/** Prediction vector based on the user input */
	Containers::SystemPrediction SystemHat;


	double PredictionCost;

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

	Containers::StateVector xNaught = Eigen::VectorXd::Random(Data.PARAM_N, Data.SINGLE);
	Containers::StateVector xBar = Eigen::VectorXd::Zero(Data.PARAM_N, Data.SINGLE);

	/** This is a temporary placeholder, will need to be substituted with the trajectory provided by golem */
	Containers::StateVector xBarGoal(P_N);
	Containers::StateVector xDot = Eigen::VectorXd::Zero(Data.PARAM_N, Data.SINGLE);

	Containers::InputVector uNaught = Eigen::VectorXd::Random(Data.PARAM_M, Data.SINGLE);
	Containers::InputVector uBar = Eigen::VectorXd::Zero(Data.PARAM_M, Data.SINGLE);
	Containers::InputVector u = Eigen::VectorXd::Random(Data.PARAM_M, Data.SINGLE);
	Containers::InputVector uStar = Eigen::VectorXd::Zero(Data.PARAM_M, Data.SINGLE);

	/** These remain identity matrices and are computed at each time step into their exp transforms */

	Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(Data.PARAM_N, Data.PARAM_N);
	Eigen::MatrixXd R = Eigen::MatrixXd::Identity(Data.PARAM_M, Data.PARAM_M);

	/** These represent the Q and R matrices at time t */
	Containers::StateCostMatrix Qt;
	Containers::InputCostMatrix Rt;

	Containers::FeedbackMatrix K = Eigen::MatrixXd::Random(Data.PARAM_M, Data.PARAM_N);

	Containers::PMatrix P = Eigen::MatrixXd::Random(Data.PARAM_N, Data.PARAM_N);

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


	/** Define both */
	double BestTraj = 999;
	double CurrentCost = 99999;

	// We read the .txt file with trajectories and pack them into 2 maps. one has trajectory IDs, the other has inputs
	ReadFile(TrajFile, CompleteWaypoints, CompleteInputs);

	// We obtain the individual finalised keys from both maps
	GetKeys(CompleteWaypoints, WaypContainer, TrajContainer);

	double tempwayp;

	/** For trajectory in trajectories; for waypoints in waypoints */
	for(trajit = WaypContainer.begin(); trajit != WaypContainer.end(); trajit++) {


		/** Recheck whether we want to iterate until the last element */
		for(waypit = TrajContainer.begin(); waypit != TrajContainer.end(); waypit++) {

			/** We iteratively solve P and K */

			// Rt and Qt correspond to the current Waypoint

			/* This is helper that converts the waypoint number to a double that can be used in the Update functions */
			tempwayp = std::stod(*waypit);

			Rt = LQR.UpdateR(R, tempwayp);
			Qt = LQR.UpdateQ(Q, tempwayp);
			P = LQR.SolveDARE(A, B, Qt, Rt);
			K = LQR.ComputeK (Rt, B, P);

	//				uStar = LQR.OptimalControl (x, Waypoints[TrajPoint].Inputs, K); // the uStar is fed into the reference change and that is fed into the CostFunction
	//				LQR.ReferenceChange (uBar, xBar, Waypoints[TrajPoint].Inputs, Waypoints[TrajPoint].States, uStar, x);
	//				Data.System[TrajPoint].StateBar = xBar;
	//				Data.System[TrajPoint].InputBar = uBar;

			/** Before we add information to the feedback matrix we need to convert it into the correct format to have a good identifier */

			Identifier = *trajit + *waypit;
			/** Save K for this trajectory and waypoint */
			FeedbackMatrixMap.insert({Identifier, K});
			std::cout << "Computed feedback matrix for waypoint " << *waypit << std::endl;
		}
	std::cout << "Computed feedback matrix for trajectory: " << *trajit << std::endl;

	}
	std::cout << "Computed all the Feedback matrices." << std::endl;

	// this initialises the whole sequence
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

		if (std::stod(NearWayp) >= 1)

	    {
	    	x = x + xDot;
	    }

		// We compute the nearest neighbour to our current state (x) from the map of all waypoints and both containers
		NearWayp = NearestWaypoint(x, CompleteWaypoints, WaypContainer, TrajContainer, UserHistory, NearWayp);

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
		for(trajit = WaypContainer.begin(); trajit != WaypContainer.end(); trajit++)
		{


			PredIdentifier = *trajit + NearWayp;
			/* convert due to input specifics */

			WaypointArgument = CompleteWaypoints[PredIdentifier];
			InputArgument = CompleteInputs[PredIdentifier];

			SystemHat = LQR.Prediction(A, B, uUser, x, WaypointArgument, InputArgument);

			// we use same trick as above //
			tempwayp = std::stod(NearWayp);
			Rt = LQR.UpdateR(R, tempwayp);
			Qt = LQR.UpdateQ(Q, tempwayp);

			PredictionCost = LQR.PredictionCostFunc(SystemHat.StateHat, SystemHat.InputHat, Rt, Qt);

			if (CurrentCost > PredictionCost)
				{CurrentCost = PredictionCost;
				OptTraj = *trajit;} // optimal trajectory
			}
		BestTraj = 999;
		CurrentCost = 99999;

		OptWaypoint = OptTraj + NearWayp;
	    UserHistory.push_back(OptTraj);

	    /** Select correct trajectory + correct waypoint */
	    uStar = LQR.OptimalControl(x, uUser, FeedbackMatrixMap[OptWaypoint]);

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






