/*
 * ReadFIle.cpp
 *
 *  Created on: 7 Jul 2018
 *      Author: sebastijan
 */


#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>
#include <vector>
#include <string>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <stdio.h>
#include <sstream>
#include <map>
#include <thread>
#include <math.h>
#include<fstream>
#include <iostream>

															// This saves trajectory ID and the trajectory state
void FileParser::ReadFile(std::ifstream& TrajFile, Containers::WaypMap& CompleteWaypoints, Containers::InputMap& CompleteInputs)
{

 /** Define local variables */
 std::string Identifier;
 Containers::StateVector State;
 Containers::InputVector Input;
 double xCoord, yCoord, Theta, xDot, yDot;

 /** We know the trajectory file; it's created by CreateTrajectory.cpp
  */

 TrajFile.open("Trajectory.txt");
 int RowCounter = 0;

 for(std::string Line; getline(TrajFile, Line); )
 {
     std::vector<std::string> Sentence;
     boost::split(Sentence, Line, [](char c){return c == ' ';});

     if (RowCounter == 0){}
     else {
     /** This assumes the file is built the way I have built it in the CreateTrajectory.cpp */
     try {
     xCoord = std::stod(Sentence[0]);
	 yCoord = std::stod(Sentence[1]);
	 Theta = std::stod(Sentence[2]);
	 xDot = std::stod(Sentence[3]);
	 yDot = std::stod(Sentence[4]);
	 	 	 	  /** TrajID,   WaypointNR */
	 Identifier = Sentence[6] + Sentence[5];
     State << xCoord,
    		   yCoord,
			   Theta;
     Input << xDot,
    		   yDot;

     CompleteWaypoints.insert({Identifier, State});
     CompleteInputs.insert({Identifier, Input});
     }
     catch (...)
     {std::cout << "Something went wrong in one of the lines";};

     }
     RowCounter++;
     }

 TrajFile.close();
 std::cout << "Successfully Imported the trajectory information." << std::endl;
};


