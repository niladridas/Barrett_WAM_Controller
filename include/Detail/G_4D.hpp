/*
 * G_4D.hpp
 *
 *  Created on: 31-Jan-2015
 *      Author: nilxwam
 */

#ifndef G_4D_HPP_
#define G_4D_HPP_

#include <eigen2/Eigen/Core>
#include <math.h>

Eigen::Vector4d G_4D(const Eigen::Vector4d x) /* x denoted the four joint positions ,  g denoted the acceleration due to gravity */
{

	double g;
	g = 10;
	Eigen::Vector4d G;

	G[0] = 0.0;

	G[1] = ((((((((((((((0.037 * g * sin((x[1] + x[2]) - x[3])
			- 0.037 * g * sin((x[1] - x[2]) + x[3]))
			+ 0.00906 * g * cos((x[1] - x[2]) - x[3]))
			+ 0.037 * g * sin((x[1] - x[2]) - x[3]))
			- 0.0299 * g * cos(x[1] + x[2])) + 0.0181 * g * cos(x[1] + x[3]))
			- 0.0741 * g * sin(x[1] + x[3])) - 0.0299 * g * cos(x[1] - x[2]))
			- 0.0181 * g * cos(x[1] - x[3])) - 0.0741 * g * sin(x[1] - x[3]))
			+ 0.00108 * g * cos(x[1])) - 1.23 * g * sin(x[1]))
			+ 0.00906 * g * cos((x[1] + x[2]) + x[3]))
			- 0.037 * g * sin((x[1] + x[2]) + x[3]))
			+ 0.00906 * g * cos((x[1] + x[2]) - x[3]))
			+ 0.00906 * g * cos((x[1] - x[2]) + x[3]);

	G[2] = ((((((((0.037 * g * sin((x[1] + x[2]) - x[3])
			+ 0.037 * g * sin((x[1] - x[2]) + x[3]))
			- 0.00906 * g * cos((x[1] - x[2]) - x[3]))
			- 0.037 * g * sin((x[1] - x[2]) - x[3]))
			- 0.0299 * g * cos(x[1] + x[2])) + 0.0299 * g * cos(x[1] - x[2]))
			+ 0.00906 * g * cos((x[1] + x[2]) + x[3]))
			- 0.037 * g * sin((x[1] + x[2]) + x[3]))
			+ 0.00906 * g * cos((x[1] + x[2]) - x[3]))
			- 0.00906 * g * cos((x[1] - x[2]) + x[3]);

	G[3] = ((((((((((0.0181 * g * cos(x[1] + x[3])
			- 0.037 * g * sin((x[1] - x[2]) + x[3]))
			- 0.00906 * g * cos((x[1] - x[2]) - x[3]))
			- 0.037 * g * sin((x[1] - x[2]) - x[3]))
			- 0.037 * g * sin((x[1] + x[2]) - x[3]))
			- 0.0741 * g * sin(x[1] + x[3])) + 0.0181 * g * cos(x[1] - x[3]))
			+ 0.0741 * g * sin(x[1] - x[3]))
			+ 0.00906 * g * cos((x[1] + x[2]) + x[3]))
			- 0.037 * g * sin((x[1] + x[2]) + x[3]))
			- 0.00906 * g * cos((x[1] + x[2]) - x[3]))
			+ 0.00906 * g * cos((x[1] - x[2]) + x[3]);

	return G;
}

#endif /* G_4D_HPP_ */
