/*
 * TrajectoryExtractor.cpp
 *
 *  Created on: 5 Jul 2018
 *      Author: sebastijan
 */




/** This function receives the potential candidates (i.e. trajectories with waypoints for a given time */
/*Containers::TrajSeq TrajectoryExtractor(const Containers::NearNeighMap& PotentialCandidates)
{
	Containers::TrajSeq PotentialTrajs;

	std::locale loc;
	std::string::const_iterator CandidateEl;
	std::string currenttraj;
	std::string CandidateKey;

	/* it iterates through all the keys of the candidate in question */
/*	for (auto const& Candidate : PotentialCandidates)
	{
		CandidateKey = Candidate.first;

		/* We iterate through each character in the key (i.e. A15) */
/*	 for (CandidateEl = CandidateKey .begin(); CandidateEl != CandidateKey.end(); CandidateEl++)
	 {	 /* If the character is an alphabetic character (i.e. is not a number) */
    	 /* we add it to the string we are currently building. the string we are building is the trajectory representation */
/*		 while (std::isalpha(*CandidateEl, loc))
		 {
			 currenttraj += *CandidateEl;
		 }
		 /* this will add the current trajectory information into a new container that will be returned */
/*		 PotentialTrajs.push_back(currenttraj);
		 currenttraj = "";
		 }
	 } /* this now just returns the container of potential trajectories */
/*	return PotentialTrajs;
}
*/
