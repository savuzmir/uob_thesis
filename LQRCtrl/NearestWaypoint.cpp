/*
 * NearestWaypoint.cpp
 *
 *  Created on: 5 Jul 2018
 *      Author: sebastijan
 */

#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>

#include <stdio.h>
#include <map>
#include <iostream>



/* TODO: Transfer NearestWaypoint here */



std::string WaypointCompute::NearestWaypoint(const Containers::StateVector& x,
							Containers::WaypMap& CompleteWaypoints,
							const Containers::WaypSeq& WaypContainer,
							const Containers::TrajSeq& TrajContainer,
							Containers::WaypSeqHist& UserHistory,
							std::string NearWayp)
	{
	// TODO: make names nicer. otherwise this works correctly
	/*
	 * Each time we look at the "current waypoint" as we have external information on the last known waypoint along the trajectories
	 * Question is whether this will work correctly for all types of trajectories?
	 * An alternative is to check through all the trajectory waypoints where we have not been so far. but this might be quite costly to do on each computation?
	 * An additional alternative is to have a heuristic lookahead (i.e. compute nearest neighbour for all trajectories from the last known waypoint to future 5 waypoints)
	*/

	  std::string Identifier, TrajectoryID, WaypID, LowestCostTraj;
	  Containers::TrajSeq::const_iterator TrajIt;
	  Containers::WaypSeq::const_iterator WaypIt;
	  Containers::NearNeighMap CurrNeighs;
	  Containers::NearNeighMap PotentialCandidates;
	  double LowestCost = 9e6, SmallestDistance = 9e6, CurrentDistance;
	  int Wayp;

	  /* This value defines the minimum difference between two neighbour costs that determines whether 2 waypoint values
	   * are indistinguishably close and should be considered for the next computation */

	  double tol = 1e-3;

	  /** This might be the easiest way to manipulate the NearWayp value in and outside this function */
	  Wayp = std::stoi(NearWayp);

/*	Keep this in for now in case I will need it later.
	std::cout << "this is the x: " << x << std::endl << std::endl;
	for (auto const& kv : CompleteWaypoints)
	{std::cout << "these are the completewaypoints: " << kv.first << "and " << kv.second << std::endl << std::endl;}

	for(waypit = TrajContainer.begin(); waypit != TrajContainer.end(); waypit++)
	{std::cout << "this is the trajcontainer: " << *waypit << std::endl << std::endl;}

	for(trajit = WaypContainer.begin(); trajit != WaypContainer.end(); trajit++)
	{std::cout << "this is the waypcontainer: " << *trajit << std::endl << std::endl;}

	for(Containers::WaypSeqHist::const_iterator userit = UserHistory.begin(); userit != UserHistory.end(); userit++)
	{std::cout << "this is the user history : " << *userit << std::endl << std::endl;}

*/

	/* We use the variable NearWayp to keep track of where we are
	 * Be mindful of how to best compute the current best waypoint?
	 * Might make most sense to have external information on the amount of waypoints we've
	 * crossed out so far so it starts with a reasonable guess regarding the next one
	 */

	// We iterate through trajectories //
	for (TrajIt = TrajContainer.begin(); TrajIt != TrajContainer.end(); TrajIt++)

	  {
	  /** Save the relevant trajectory ID (TrajIt) and waypoint (Wayp)
	   *  This will always look at the current wayp and use that to calculate nearest neighbour
	   *  it might make sense to check for the currrent and next one */

		Identifier =  *TrajIt + std::to_string(WaypContainer[Wayp]);

	  /** Compute Euclidean distance - this should also be changed to make it work for more than 3 states
		* TODO: Make this generalisable to more dimensions */
	  CurrentDistance = sqrt(abs((x[0] - CompleteWaypoints[Identifier][0])) + abs((x[1] - CompleteWaypoints[Identifier][1])) + abs((x[2] - CompleteWaypoints[Identifier][2])));

	  CurrNeighs.insert({Identifier, CurrentDistance});
	  }

	  /* We now have a map with a potential waypoint and the associated cost.
	   * Now we will go through this map and check for the closest
	   */
	  for (auto const& CurrNeigh : CurrNeighs)

	  {
		  if (CurrNeigh.second < LowestCost)
		  {
		  LowestCostTraj = CurrNeigh.first;
		  LowestCost = CurrNeigh.second;
		  }
	  }

	  /* We put the best one in a new map */
	  PotentialCandidates.insert({LowestCostTraj, LowestCost});

	  /* In case there is another one that's very close to the lowest one in cost, we also add that one */
	  for (auto const& Compare : CurrNeighs)
	  {
	  if (abs(Compare.second - LowestCost) < tol)
	  {
		  /* Now we  have a list of potential candidates with comparable cost */
		  PotentialCandidates.insert({Compare.first, Compare.second});
	  }
	  }

	  /* If the size of potential candidates is bigger than 1,  we need to select the one that's in line with the previous trajectory of the user */
	  if (PotentialCandidates.size() > 1)
	  {
		  TrajectoryID = HistorySeqCheck(UserHistory, PotentialCandidates);
	  }
	  /* If there is only one with the lowest value, we take the LowestCostTraj */
	  else {TrajectoryID = LowestCostTraj;};

	  /* This enables us to ignore the trajectory itself and we can extract only the waypoint */
	  for (std::string::const_iterator WaypExtractor = TrajectoryID.begin(); WaypExtractor != TrajectoryID.end(); WaypExtractor++)
	  {	 /* If the character is an alphabetic character (i.e. is not a number)
		  * we add it to the string we are currently building. The string we are building is the waypoint */
		  if (std::isdigit(*WaypExtractor))
		  {
			  WaypID.push_back(*WaypExtractor);
		  }
	  }

	return WaypID;
};
