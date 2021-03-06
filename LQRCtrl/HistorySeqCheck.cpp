/*
 * HistorySeqCheck.cpp
 *
 *  Created on: 6 Jul 2018
 *      Author: sebastijan
 */

#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>
#include <stdio.h>
#include <math.h>
#include <iostream>

/* We send the history of the user sequence into this function together with the map of the current best waypoints of trajectories
 * we return the best option based */
std::string WaypointCompute::HistorySeqCheck(Containers::WaypSeqHist& UserHistory, const Containers::NearNeighMap& PotentialCandidates)
{
	// TODO: Make names nicer. otherwise this works correctly

	std::locale loc;
	std::string::const_iterator CandidateEl;
	std::string CurrentTraj;
	std::string CandidateKey;
	std::string BestSolution;
	std::string TemporarySave;
	std::string tempstring;

	/*  for all potential candidates (i.e. trajectory waypoints + associated cost */
	for (auto const& Candidate : PotentialCandidates)
	{	/* take the trajectory waypoint  */
		CandidateKey = Candidate.first;
		for (CandidateEl = CandidateKey.begin(); CandidateEl != CandidateKey.end(); CandidateEl++)
			 {	 /* parse the string to extract only the trajectory ID but not the waypoint number
				  * we build a string that only represents the trajectory ID */

				 if (std::isalpha(*CandidateEl, loc))
				 {
					 CurrentTraj.push_back(*CandidateEl);
				 } /* we are interested just in the case where it is the same as the current one. if the trajectory one is moving towards hasn't been recorded in the past
				    * it most likels means the person is deliberately moving to a place later along the trajectory */
				 if ((UserHistory.size()> 1) && (UserHistory.back() == CurrentTraj))
				 /* this will add the current trajectory information into a new container that will be returned */
				 {BestSolution = Candidate.first;
				  break;}
				 else if (UserHistory.empty()) /* this will make a random BestSolution if the user history is empty. this can only happen if the person has just started */
				 {BestSolution = Candidate.first;
				  break;}
				 CurrentTraj = "";
			 }
		TemporarySave = Candidate.first;
		}

	/* if that procedure yields an empty BestSolution, we arbitrarily pick the last candidate of the loop
	 * it shouldn't matter which one is picked considering the cost of all of them are very low and comparable
	 * as this procedure would yield an empty BestSolution only in the case when any of the offered solutions were not present in the last element of the user History */

	if (BestSolution.empty())
	{
		BestSolution = TemporarySave;
	}
	return BestSolution;
	}
