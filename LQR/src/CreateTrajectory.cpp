/*
 * SaveUK.cpp
 *
 *  Created: 9 Jun 2018
 *  Author: Sebastijan Veselic
 *  Last edit: 28.6
 *  Last comment: 28.6.
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


void Containers::CreateTrajectory (const int &TrajectoryCreator, struct Containers::WaypSelect Waypoints[Containers::TRAJECTORY_SIZE])
	{

	/** Arbitrarily create 5 trajectories to test if it works */
	int TotalTrajectories = 5;
	std::string CurrentTrajectory[5] = { "A", "B", "C", "D", "E"};
	std::vector<float> TrajChange = {0.4, 0.3, 0.1, 0.2, 0.5};

	/** We create a trajectory */
	if (TrajectoryCreator == 1)
		{
		std::ofstream ofile;
		ofile.open("Trajectory.txt", std::ios::app);
		ofile << "x y Theta xDot yDot Waypoint Trajectory" << std::endl;

		double mu1 = 0.3;
		double sigma1 =  0.05;

		double mu2 = 0.6;
		double sigma2 = 0.05;

		for (int TrajNum = 0; TrajNum < TotalTrajectories; TrajNum++) {

		/** We change this slightly such that there is variety in the trajectories that are being generated */
		mu1 = mu1 + TrajChange[TrajNum];
		mu2 = mu1 + TrajChange[TrajNum];

        /** This represents the input vector */
		double xDot, yDot;

		std::default_random_engine generator;

		std::normal_distribution<double> distributionx(mu1, sigma1);
		std::normal_distribution<double> distributiony(mu2, sigma2);

		xDot = distributionx(generator);
		yDot = distributiony(generator);

		Waypoints[0].States[0] = xDot; /* we say xdot is the initial state of x and ydot state of y */
		Waypoints[0].States[1] = yDot;
		Waypoints[0].States[2] = (xDot+yDot)/2; /** This is just an arbitrary definition of the initial theta */

		Waypoints[0].Inputs[0] = xDot;
		Waypoints[0].Inputs[1] = yDot;

		/** For the amount of predefined waypoints */
		for (int i = 1; i < Containers::TRAJECTORY_SIZE; i++)
			{
			xDot = distributionx(generator);
			yDot = distributiony(generator);

			Waypoints[i].States[0] += Waypoints[i - 1].States[0] + xDot;
			Waypoints[i].States[1] += Waypoints[i - 1].States[1] + yDot;
			Waypoints[i].States[2] += Waypoints[i - 1].States[2] + (xDot+yDot)/2;

			/** Here we assume that xDot = Ax + Bu where A and B do not transform anything. */
			Waypoints[i].Inputs[0] = xDot;
			Waypoints[i].Inputs[1] = yDot;

			double x = Waypoints[i].States[0]*pow(i, 1.1);
			double y = Waypoints[i].States[1]*pow(i, 0.5);
			double theta = Waypoints[i].States[2]*pow(i, 0.3);

			ofile << x << " " << y << " " << theta << " " << xDot << " " << yDot << " " << i << " " << CurrentTrajectory[TrajNum] << std::endl;

			/** After we have saved them, we clean them so they don't cause issues for the next iteration */
			}

		/** This is very stupid but does what it needs to do
		 * It needs to clean the container after each run is written to the file
		 * This will not be used anymore after I'm done with the initial coding so
		 * I this will be ok for now.
		 */
		for (int j = 1; j < Containers::TRAJECTORY_SIZE; j++)
		{ Waypoints[j].States[0] = 0;
		  Waypoints[j].States[1] = 0;
		  Waypoints[j].States[2] = 0;
		  Waypoints[j].Inputs[0] = 0;
		  Waypoints[j].Inputs[1] = 0;
		}
		}
		ofile.close();
		};
	std::cout << "Created trajectories." << std::endl;
};

