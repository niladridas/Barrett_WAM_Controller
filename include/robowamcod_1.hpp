//================================
/*
	The DMP  requires the following inputs
	Current State [X, Y, X, Vx, Vy, Vz]
	Output of this system is
	Next State [X, Y, Z]
*/
//=================================

#ifndef ROBOT_TT_H_
#define ROBOT_TT_H_

#include <barrett/units.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/controller.h>

//---------------------------------------------------
#include <iostream> // cin, cout, getline
#include <cmath> // ceil
#include <time.h> // Measuring time
#include <vector>
//--------------------------------------------------

using namespace barrett;
using namespace systems;

template<size_t DOF>
class DMPCONTROL: public System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	public:
		Input<cp_type> CurrentStatePos;
		Input<cv_type> CurrentStateVel;
		Output<cp_type> NextState;

	protected: 
		typename Output<jt_type>::Value* NextStateOutputValue;

//----------------------------------------------------------------------------------------------------------------------------------
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
		float *Centers;  // Radial basis function centers
		float *Variance; // Radial basis function variance
		float *Psi; // Calculated Radial basis function value
		float force; // Non linear force to modify trajectory
		float CSx; // Canonical system value
		float CSax; // Canonical system time constant
		cp_type  cp_out;

		void MakeMatrix(float ** &temp,unsigned rows, unsigned cols)
		{
			temp = new float*[rows]; 

			for(unsigned int i = 0; i < rows; ++i)
    				temp[i] = new float[cols];
		}

		void InitDmpSys(const unsigned int dmps,const unsigned int bfs, float* a, float* b, float runtime, float tolerance)
		{
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

    			CSax = -log(tolerance)/runtime; // Time constant such that trajectory goes to 95% or (1-tolerance) in runtime
    			CSax = 0.1; // Sachaal 2012 paper values to verify code
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

		// Set goal and initial value
		void SetDMPConditions(float* y0, float* goal)
		{
			for(unsigned i=0; i<Dmps; i++)
			{
				Y0[i] = y0[i];
				Goal[i] = goal[i];
			}
		}

		// Set Radial Basis Function Centers and Varience
		void GenDMPGaussCenters(float runtime)
		{
			unsigned int i;
			float *des_c = new float [Bfs];
			des_c[0] = exp(-CSax*runtime);
			des_c[Bfs-1] = 1.05 - des_c[0];
			float tempdiff = (des_c[Bfs-1] - des_c[0]) / (Bfs-1);
			
			for(i = 1; i < Bfs; i++)
				des_c[i] = des_c[i-1] + tempdiff;
			for (i = 0; i < Bfs; i++)
			{
				// x = exp(-c), solving for c
				// Centers are distributed evenly in time axis
				// Variance accordint to Sachaal 2012 paper
				Centers[i] = -log(des_c[i]); // Set centeres
				Variance[i] = (pow(Bfs,1.5))/Centers[i]; // Set variance
			}
		}

		// Calculate Radial Basis Function value
		void GenDMPActFunc()
		{
			for(unsigned int i =0;i<Bfs;i++)
				Psi[i] = exp(-Variance[i]*pow((CSx - Centers[i]),2)); // Calculate Radial Basis Function value
		}

		// Run DMP system one time step 
		void StepDMP(float dt)
		{
			CSx = CSx - CSax*CSx*dt; // Run Canonical system one time step
			GenDMPActFunc(); // Calculate Radial Basis Function value
			float sum = 0.0;
			float w_phi = 0.0;
			for(unsigned int i = 0;i<Dmps;i++)
			{
				// force = sumation(weight*RBF value)/sumation(RBF value)
				for(unsigned int j=0;j<Bfs;j++)
				{
					w_phi = w_phi + Weights[i][j]*Psi[j];
					sum = sum + Psi[j];
				}
				
				force = (w_phi*CSx*(Goal[i]-Y0[i]))/sum;
				// acceleration = ay*( by*(error) - deriavtive_error) + force
				// stable dynamics as force is zero when canonical system goes to zero
				DDy[i] = (ay[i]*(by[i]*(Goal[i] - Y[i]) - Dy[i]) + force);
				Dy[i] = Dy[i] + DDy[i]*dt;
				Y[i] = Y[i] + Dy[i]*dt;
			}  
		}

		void CheckDMPGaolOffset() //
		{
		 	// Check to see if initial position and goal are the same
		 	// If they are, offset slightly so that the forcing term is not 0
			for (unsigned int i = 0; i<Dmps; i++)
				if (Y0[i] == Goal[i])
					Goal[i] = Goal[i] + 0.0001;
		}
			
		// Reset DMP
		void ResetDMPState()
		{
			CSx = 1.0;
			CSax = 0.0;
			for(unsigned int i=0; i<Dmps; i++)
			{
				Y[i] = Y0[i];
				Dy[i] = 0;
				DDy[i] = 0;
			} 
		}
	
	DMPCONTROL(const std::string& sysName = " DMPCONTROL ") :
	System(sysName), referencejpInput(this), referencejvInput(this), referencejaInput(this), feedbackjpInput(this), feedbackjvInput(this), M(this), C(this), NextState(this, & NextStateOutputValue)
	{

	}

	virtual ~ DMPCONTROL () 
	{
		this->mandatoryCleanUp();
	}

	virtual void operate() 
	{
		/*Taking reference values from the input terminal of this system*/
		
		CurrentStatePos = this->referencecpInput.getValue();
		CurrentStateVel = this->referencecvInput.getValue();
	
		Y[0] = CurrentStatePos[0];
		Y[1] = CurrentStatePos[1];
		Y[2] = CurrentStatePos[2];
	
		Dy[0]  = CurrentStateVel[0];
		Dy[1] = CurrentStateVel[1];
		Dy[2] = CurrentStateVel[2]; 
	
		StepDMP(0.0025);
	
		cp_out[0] = Y[0];
		cp_out[1] = Y[1];
		cp_out[1] = Y[2];
	
		controlOutputValue->setData(&cp_out);
	}

	private:
		DISALLOW_COPY_AND_ASSIGN(DMPCONTROL);
};
#endif
