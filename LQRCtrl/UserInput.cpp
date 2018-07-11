/*
 * UserInput.cpp
 *
 *  Created on: 10 Jul 2018
 *      Author: sebastijan
 */

#include </home/sebastijan/Documents/Golem/packages/Plugin/Data/iLQR/include/Golem/Data/iLQR/iLQR.h>
#include <iostream>
#include <SFML/Graphics.hpp>
#include <string>

// This is a temporary function to test receiving user input

// This helper function scales the input between -1 and 1
double Scale(double curr, double min, double max)
{return 2*(curr - min)/(max-min) - 1;}


Containers::InputVector InputSFML::UserInput(Containers::WIDTH Width, Containers::HEIGHT Height, Containers::InputVector &uUser, Containers::NORM_CONST NormFactor)
	{
	std::string Filter;
	double xDot, yDot, Theta;

	/** Check for key presses. if R is pressed it will only change rotation
	 * other keys don't do anything.
	 */

	if (sf::Keyboard::isKeyPressed(sf::Keyboard::R))
			{
				Filter = "R";
			}

	else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
			{
				Filter = "L";
			}
	else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
			{
				Filter = "R";
			}

	// get the global mouse position
	sf::Vector2i globalPosition = sf::Mouse::getPosition();

	// we first normalise the position change with respect to screen size
	xDot = globalPosition.x/(double)Width;
	yDot = globalPosition.y/(double)Height;

	// we then normalise both such that the centre is 0 and after that we normalise between -1 and 1
	xDot = xDot - NormFactor;
	xDot = Scale(xDot, -0.5, 0.5);

	//  y is additionally multiplied by -1 to obtain positive values when moving the mouse up
	yDot = (yDot - NormFactor)*(-1);
	yDot = Scale(yDot, -0.5, 0.5);


	if (Filter == "R") {

		xDot = 0;
		yDot = 0;
		Theta = globalPosition.x/(double)Width - NormFactor;

		Theta = Scale(Theta, -0.5, 0.5);

		Filter == "None";
	}

	uUser << xDot,
			 yDot;


	std::cout << "User input: " << "(" << xDot << ", " << yDot << ")" << std::endl;


	return uUser;

	}
