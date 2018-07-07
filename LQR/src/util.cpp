/*
 * util.cpp
 *
 *  Created: 5 Jun 2018
 *  Author: Sebastijan Veselic
 *  Last edit: 25.6
 *  Last comment: 25.6.
 */

#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <string>
#include <stdio.h>
#include <sstream>
#include <typeinfo>

/* TODO: Rethink this function and whether its working the way it should. It would be good to have functionality
 * that would check the size of matrices before we start computing to make sure it runs correctly during execution
 */

std::string  Util::SizeCheck(const Eigen::MatrixXd &MatrixInput, const std::string &MatrixType)

{
	/** Initialise local variables and constants
	 * MatrixInput is the actual matrix. MatrixType is the string information about which matrix we have
	 * MatrixID tells us later which MatrixType was used as an input
	 * Message tells us what went wrong */

	int MatrixID, Message;

	int n = Containers::PARAM_N;
	int m = Containers::PARAM_M;

	/** High value to catch if something goes wrong */
	MatrixID = 999;

	std::stringstream Output;
	std::string Input = MatrixType;

	/** Different sizes:             nxn,                    mxn,                  mxn,             nxn,                mxm,          nxn,          nxm 		*/
	std::string Matrices[] = {"TransitionMatrix", "ControlMatrixTranspose", "FeedbackMatrix", "StateCostMatrix", "InputCostMatrix", "PMatrix", "ControlMatrix"};

	for (int i = 0; i < Containers::MATRIX_NUM; i++){
		if (Input.compare(Matrices[i]) == 0)
				{
			MatrixID = i;
				}
	};

	int row = MatrixInput.rows();
	int col = MatrixInput.cols();

	switch(MatrixID) {
	case 0 :
		if ((row != n) & (col != n))
		{Message = 1;}
	case 1 :
		if ((row  != m) & (col != n))
		{Message  = 1;}
	case 2 :
		if ((row  != m) & (col != n))
		{Message = 1;}
	case 3 :
		if ((row  != n) & (col != n))
		{Message = 1;}
	case 4 :
		if ((row  != n) & (col != n))
		{Message = 1;}
	case 5 :
		if ((row  != n) & (col != n))
		{Message = 1;}
	case 6 :
		if ((row  != n) & (col != m))
		{Message = 1;}
	case 999 :
		Message = 0;}

	if (Message == 1)

	{}// std::cout << "Your " << Input << " doesn't have the correct size. You either used the incorrect matrix as an input or mixed up your matrix parametrisation. Check it.";}

	else if (Message == 2)

	{}// std::cout << "The string you have used does not match anything we have stored.";}

	else if (Message == 0)

	{}//Output << "0";}

	else

	{}//Output << "This should have not happened";};

	std::string Fin;

	Fin = Output.str();

	return Fin;


};
