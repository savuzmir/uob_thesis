/*
 * PrepareContainers.cpp
 *
 *  Created on: 7 Jul 2018
 *      Author: sebastijan
 */


#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>
#include <stdio.h>
#include <string>
#include <iostream>


void WaypointCompute::PrepareContainers(Containers::WaypSeq& WaypContainer, Containers::TrajSeq& TrajContainer)
	{

    sort(begin(WaypContainer), end(WaypContainer));
    sort(begin(TrajContainer), end(TrajContainer));

    Containers::TrajSeq::iterator TrajIt;
    Containers::WaypSeq::iterator WaypIt;

    WaypIt= std::unique (WaypContainer.begin(), WaypContainer.end());
    TrajIt = std::unique (TrajContainer.begin(), TrajContainer.end());

    WaypContainer.resize( std::distance(WaypContainer.begin(), WaypIt) );
	TrajContainer.resize( std::distance(TrajContainer.begin(), TrajIt) );
	}
