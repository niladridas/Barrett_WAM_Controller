/*
 * DMP_first-inl.hpp
 *
 *  Created on: 28-Mar-2015
 *      Author: nilxwam
 */

#ifndef DMP_FIRST_INL_H_
#define DMP_FIRST_INL_H_

template<size_t DOF>
void DMP_first<DOF>::MakeMatrix(float ** &temp, unsigned rows, unsigned cols) {
	temp = new float*[rows];

	for (unsigned int i = 0; i < rows; ++i)
		temp[i] = new float[cols];
}

template<size_t DOF>
void DMP_first<DOF>::LoadWeights() {

	string line;
	ifstream file("w.csv"); //TODO : specifics
	while (file.good()) {
		for (unsigned int ii = 0; ii < Dmps; ii++) {
			for (unsigned int jj = 0; jj < Bfs; jj++) {
				getline(file, line, ',');
				Weights[ii][jj] = atof(line.c_str());
			}
		}
		break;
	}
	//for (unsigned int i=0; i<Dmps; i++)
	//    for(unsigned int j=0;j<Bfs;j++)
	//    Weights[i][j] = data[i][j];
}

// Set goal and initial value
template<size_t DOF>
void DMP_first<DOF>::SetDMPConditions(float* y0, float* goal) {
	for (unsigned i = 0; i < Dmps; i++) {
		Y0[i] = y0[i];
		Y[i] = y0[i];
		Goal[i] = goal[i];
	}
}

// Set Radial Basis Function Centers and Varience
template<size_t DOF>
void DMP_first<DOF>::GenDMPGaussCenters(float runtime) {
	unsigned int i;
	float *des_c = new float[Bfs];
	des_c[0] = exp(-CSax * runtime);
	des_c[Bfs - 1] = 1.05 - des_c[0];
	float tempdiff = (des_c[Bfs - 1] - des_c[0]) / (Bfs - 1);

	for (i = 1; i < Bfs; i++)
		des_c[i] = des_c[i - 1] + tempdiff;
	for (i = 0; i < Bfs; i++) {
		// x = exp(-c), solving for c
		// Centers are distributed evenly in time axis
		// Variance accordint to Sachaal 2012 paper
		Centers[i] = -std::log(des_c[i]); // Set centeres
		Variance[i] = (pow(Bfs, 1.5)) / (Centers[i] + 0.001); // Set variance
	}
}

// Calculate Radial Basis Function value
template<size_t DOF>
float DMP_first<DOF>::GenDMPActFunc() {
	float sum = 0.0;
	for (unsigned int i = 0; i < Bfs; i++)
	{
		Psi[i] = exp(-Variance[i] * pow((CSx - Centers[i]), 2)); // Calculate Radial Basis Function value
		sum = sum + Psi[i];
	}
	return sum;
}

template<size_t DOF>
void DMP_first<DOF>::InitDmpSys(const unsigned int dmps,const unsigned int bfs, float* a, float* b, float runtime, float tolerance)
		{
			dt = 0.002;
			PrevTime = 0.0;
			IsGoal = false;
			Time.tv_sec = 0;
			Time.tv_nsec = 0;
			Dmps = dmps; // Set number of DMPs
    			Bfs = bfs;
			Y0 = new float [Dmps]; // Set initial value
			Y = new float [Dmps]; // Set initial value
			Dy = new float [Dmps]; // Set initial value
			DDy = new float [Dmps]; // Set initial value
			Goal = new float [Dmps]; // Set goal value
			Centers = new float [Bfs]; // Allocate memory for Radial Basis Function centers
			Variance = new float [Bfs]; // Allocate memory for Radial Basis Function variance
			Psi = new float [Bfs];  // Allocate memory for Radial Basis Function output
			MakeMatrix(Weights,Dmps,Bfs); // Allocate memory for weights
			ay = new float [Dmps]; // Allocate memory for PD controller parameters
			by = new float [Dmps]; // Allocate memory for PD controller parameters

    			CSax = -std::log(tolerance)/runtime; // Time constant such that trajectory goes to 95% or (1-tolerance) in runtime
    			//CSax = -log(0.05)/; // Sachaal 2012 paper values to verify code
			CSx = 1.0; // Canonicalsystem initialize
			force = 0.0;

			for (unsigned int i=0; i<Dmps; i++)
			{
				ay[i] = a[i]; // Set value
				by[i] = b[i]; // Set value
				Y[i] = 0.0; // Empty for safety
				Dy[i] = 0.0; // Empty for safety
				DDy[i] = 0.0; // Empty for safety

				for(unsigned int j=0;j<Bfs;j++)
					Weights[i][j] = 0.0; // Empty for safety
			}

			GenDMPGaussCenters(runtime); // Set Radial Basis Function Centers and Varience
		}


// Run DMP system one time step
template<size_t DOF>
bool DMP_first<DOF>::StepDMP(float dt) {
	CSx = CSx - CSax * CSx * dt; // Run Canonical system one time step
	float sum = GenDMPActFunc(); // Calculate Radial Basis Function value
	//float sum = 0.0;
	float w_phi = 0.0;
	for (unsigned int i = 0; i < Dmps; i++) {
		w_phi = 0.0;
		// force = sumation(weight*RBF value)/sumation(RBF value)
		for (unsigned int j = 0; j < Bfs; j++) {
			w_phi = w_phi + Weights[i][j] * Psi[j];
			//sum = sum + Psi[j];
		}

		force = (w_phi * CSx * (Goal[i] - Y0[i])) / sum;
		// acceleration = ay*( by*(error) - deriavtive_error) + force
		// stable dynamics as force is zero when canonical system goes to zero
		DDy[i] = (ay[i] * (by[i] * (Goal[i] - Y[i]) - Dy[i]) + force);
		Dy[i] = Dy[i] + DDy[i] * dt;
		Y[i] = Y[i] + Dy[i] * dt;
		//Y[i] = Y0[i] + 0.5*DDy[i]*DDy[i]*dt*dt;
	}
	if (CSx == 0.05)
		return true;
	else
		return false;
}

template<size_t DOF>
void DMP_first<DOF>::CheckDMPGaolOffset() //
{
	// Check to see if initial position and goal are the same
	// If they are, offset slightly so that the forcing term is not 0
	for (unsigned int i = 0; i < Dmps; i++)
		if (Y0[i] == Goal[i])
			Goal[i] = Goal[i] + 0.0001;
}

template<size_t DOF>
void DMP_first<DOF>::CheckDMPGaol() //
{
	// Check to see if initial position and goal are the same
	// If they are, offset slightly so that the forcing term is not 0
	IsGoal = true;
	for (unsigned int i = 0; i < Dmps; i++) {
		if (abs(Y[i] - Goal[i]) < 0.01) {
			//printf(" 1 ");
			IsGoal &= true;
		} else {
			//printf(" 0 ");
			IsGoal &= false;
		}

	}
	if(IsGoal)
		printf(" = %d \n", IsGoal);
}

// Reset DMP
template<size_t DOF>
void DMP_first<DOF>::ResetDMPState() {
	CSx = 1.0;
	CSax = 0.0;
	for (unsigned int i = 0; i < Dmps; i++) {
		Y[i] = Y0[i];
		Dy[i] = 0;
		DDy[i] = 0;
	}
}

template<size_t DOF>
DMP_first<DOF>::DMP_first(const unsigned int dmps, const unsigned int bfs,
		float* a, float* b, float runtime, float tolerance, float* y0,
		float* goal) :
		ref_jp(this, &ref_jp_OutputValue), ref_jv(this, &ref_jv_OutputValue), ref_ja(
				this, &ref_ja_OutputValue) {
	InitDmpSys(dmps, bfs, a, b, runtime, tolerance);
	SetDMPConditions(y0, goal); // Initial conditions
	CheckDMPGaolOffset();
	LoadWeights();
	ref_jp_tmp(0.0);
	ref_jp_tmp[0] = y0[0];
	ref_jp_tmp[1] = y0[1];
	ref_jp_tmp[2] = y0[2];
	ref_jp_tmp[3] = y0[3];
	ref_jv_tmp(0.0);
	ref_ja_tmp(0.0);
}

#endif /* DMP_FIRST_INL_HPP_ */
