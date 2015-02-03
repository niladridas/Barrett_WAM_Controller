/*
 * Dynamics.cpp
 *
 *  Created on: 25-Aug-2012
 *      Author: mobman
 */

//#include <Dynamics.h>
#include <typeinfo>
#include <math.h>
#include <sstream>

#include <M_4D.hpp>
#include <C_4D.hpp>
#include <G_4D.hpp>


Dynamics::Dynamics() :
		massMatrix(Eigen::Matrix4d()), Cvec(Eigen::Vector4d()), Gx(
				Eigen::Vector4d()), Tm(Eigen::Vector4d()) {

	massMatrix = Eigen::Matrix4d::Zero(4, 4);
	Cvec = Eigen::Vector4d::Zero(4);
	Gx = Eigen::Vector4d::Zero(4);
	Tm = Eigen::Vector4d::Zero(4);

}

// Mass matrix definition/////////////////////////////////////////
void Dynamics::makemassMat(const Eigen::Vector4d theta) {
	massMatrix = M_4D(theta);
}

// Definition of C matrix//////////////////////////////////////////////////////////////
void Dynamics::makeCvec(const Eigen::Vector4d theta,
		const Eigen::Vector4d thetaDot) {

	Cvec = C_4D(theta, thetaDot);
}

// Phi Vector definition ///////////////////////////////////////////////////////////
void Dynamics::makeGx(const Eigen::Vector4d theta) {

	Gx = G_4D(theta);

}

void Dynamics::computeModel(const Eigen::Vector4d theta,
		const Eigen::Vector4d thetaDot, const Eigen::Vector4d thetaDotDot) {

	makemassMat(theta);
	makeCvec(theta, thetaDot);
	makeGx(theta);
	Tm = massMatrix * thetaDotDot + Cvec  + Gx;

	return;
}

 /* namespace Sam */

