/*
 * FileParser.cpp
 *
 *  Created on: 7 Jul 2018
 *      Author: sebastijan
 */

#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <map>
#include <iostream>



void FileParser::GetKeys(const Containers::WaypMap& CompleteWaypoints, Containers::WaypSeq& WaypContainer, Containers::TrajSeq& TrajContainer)
{
  std::string WaypointNr, CurrentTraj, TrajID;
  std::vector<std::string> TmpVector, KeyVector;

  for (auto const& Key: CompleteWaypoints)
  {
    TmpVector.push_back(Key.first);
  }


  for (int e = 0; e < TmpVector.size(); e++)
  	  {
	  WaypointNr = "";
	  CurrentTraj = TmpVector[e]; /* We use it to append the current trajectory value */
	  TrajID = CurrentTraj[0];  /* We use it to append the current to be used when added to the trajectory vector; */

	  WaypointNr.append(CurrentTraj.begin()+1, CurrentTraj.end());

	  WaypContainer.push_back(std::stod(WaypointNr)); /* This always pushes the waypoint number */
	  TrajContainer.push_back(TrajID);     /* and this the trajectory id */
  	  }

	std::cout << "Trajectory ID and waypoint numbers are saved." << std::endl;

}
