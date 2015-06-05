/*
 * DMP_first.h
 *
 *  Created on: 28-Mar-2015
 *      Author: nilxwam
 */

#ifndef DMP_FIRST_H_
#define DMP_FIRST_H_

#include <time.h>
#include <fstream>
#include <iostream>

#include <barrett/math/traits.h>
#include <list>
#include <barrett/units.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <eigen3/Eigen/Core>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/controller.h>
#include <barrett/products/product_manager.h>

using namespace std;
using namespace barrett;
using namespace systems;

template<size_t DOF>
class DMP_first: public System {
	/* Torque*/
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	unsigned int Dmps; // number of DMP , individual trajectory we will follow
	unsigned int Bfs; // Number of Radial basis function for non linear function
	float *Y0; // Initial state
	float *Y; // Calculated state
	float *Dy; // Calculated velocity
	float *DDy; // Calculated acceleration
	float *Goal; // Goal state
	float **Weights; // Weights for function
	float *ay; // PD controller constants
	float *by; // PD controller constants
	float *Centers; // Radial basis function centers
	float *Variance; // Radial basis function variance
	float *Psi; // Calculated Radial basis function value
	float force; // Non linear force to modify trajectory
	float CSx; // Canonical system value
	float CSax; // Canonical system time constant
	double dt;
	timespec Time;
	float PrevTime;
	bool IsGoal;
	jp_type ref_jp_tmp; // To move the hand
	jv_type ref_jv_tmp;
	ja_type ref_ja_tmp;

	void MakeMatrix(float ** &temp, unsigned rows, unsigned cols); // ONE  TIME

	void LoadWeights(); // ONE  TIME

	void InitDmpSys(const unsigned int dmps, const unsigned int bfs, float* a,
			float* b, float runtime, float tolerance); // ONE  TIME

	// Set goal and initial value
	void SetDMPConditions(float* y0, float* goal); // ONE  TIME

	// Set Radial Basis Function Centers and Varience
	void GenDMPGaussCenters(float runtime); // ONE  TIME

	// Calculate Radial Basis Function value
	float GenDMPActFunc();
	// Run DMP system one time step
	bool StepDMP(float dt);

	void CheckDMPGaolOffset(); // ONE  TIME

	void CheckDMPGaol(); //

	// Reset DMP
	void ResetDMPState();

	DMP_first(const unsigned int dmps, const unsigned int bfs, float* a,
			float* b, float runtime, float tolerance, float* y0, float* goal);
	virtual ~DMP_first() {
		this->mandatoryCleanUp();
	}
	;

public:
	Output<jp_type> ref_jp; // To move the hand
	Output<jv_type> ref_jv;
	Output<ja_type> ref_ja;

protected:
	typename Output<jp_type>::Value* ref_jp_OutputValue;
	typename Output<jv_type>::Value* ref_jv_OutputValue;
	typename Output<ja_type>::Value* ref_ja_OutputValue;

	virtual void operate() {

		StepDMP(0.002);
		CheckDMPGaol();
		ref_jp_tmp[0] = Y[0];
		ref_jv_tmp[0] = Dy[0];
		ref_ja_tmp[0] = DDy[0];
		ref_jp_tmp[1] = Y[1];
		ref_jv_tmp[1] = Dy[1];
		ref_ja_tmp[1] = DDy[1];
		ref_jp_tmp[2] = Y[2];
		ref_jv_tmp[2] = Dy[2];
		ref_ja_tmp[2] = DDy[2];
		ref_jp_tmp[3] = Y[3];
		ref_jv_tmp[3] = Dy[3];
		ref_ja_tmp[3] = DDy[3];

		this->ref_jp_OutputValue->setData(&ref_jp_tmp);
		this->ref_jv_OutputValue->setData(&ref_jv_tmp);
		this->ref_ja_OutputValue->setData(&ref_ja_tmp);

	}

};

#include <Detail/DMP_first-inl.h>
#endif /* DMP_FIRST_H_ */
