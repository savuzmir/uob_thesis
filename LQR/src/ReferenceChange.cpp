/*
 * ReferenceChange.cpp
 *
 *  Created: 5 Jun 2018
 *  Author: Sebastijan Veselic
 *  Last edit: 9.6
 *  Last comment: 9.6.
 */

#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <stdio.h>
#include <math.h>


/* Rereferences the state vector by subtracting x - nNaught. It directly manipulates the vector in question.
 */

void iLQR::ReferenceChange (Containers::InputVector &uBar, Containers::StateVector &xBar,
							const Containers::InputVector &uNaught, const Containers::StateVector &xNaught,
							const Containers::InputVector &u, const Containers::StateVector &x)
{
   Util Utility;

   Utility.SizeCheck(uBar, Utility.InpVec());
   Utility.SizeCheck(xBar, Utility.StVec());
   Utility.SizeCheck(uNaught, Utility.InpVec());
   Utility.SizeCheck(xNaught, Utility.StVec());
   Utility.SizeCheck(u, Utility.InpVec());
   Utility.SizeCheck(x, Utility.StVec());

	xBar = x - xNaught;
	uBar = u - uNaught;
}


