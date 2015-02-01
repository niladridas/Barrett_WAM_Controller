#ifndef INPUTMATRIX_H_
#define INPUTMATRIX_H_

#include<barrett/systems.h>
#include <barrett/systems/abstract/system.h>
#include <iostream>
#include <tsk.h>


 namespace barrett{
 namespace systems{

template<size_t DOF, int NSTATES, int NR>
class inputMatrix : public System
{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
public:
	typedef typename Eigen::Matrix<double, NSTATES, NSTATES/2> inputMatrix_type;
	typedef typename math::Vector<NSTATES, double>::type gx_type;
//
public:
		Input<jp_type> jPositions;
		Input<jv_type> jVelocities;
		Input<jt_type> jTorques;
		Output<inputMatrix_type> gx;

		Output<gx_type> gxrow;

protected:
		typename Output<inputMatrix_type >::Value* gxVal;
		typename Output<gx_type>::Value* gxrowVal;

private:
		Sam::tsk<NSTATES, NSTATES/2, NR> fuz;//("./FuzzParams/cons_5nrX_20sat_182e_10kd.txt", "./FuzzParams/gaussParams_5nrX_20sat_182e_10kd.txt");
		Eigen::Matrix<double, NSTATES/2, 1> u;
		Eigen::Matrix<double, NSTATES, 1> x;
		jp_type jp;
		jv_type jv;
		jt_type jt;
//		Eigen::Matrix<double, 2*NJOINTS, NJOINTS> gx_mat;
		inputMatrix_type gx_mat;
		gx_type  gx_matrow;

public:

		explicit inputMatrix(const std::string& sysName = "inputMatrix");
		 virtual ~inputMatrix(){this->mandatoryCleanUp();};
private:
		virtual void operate();
private:
	DISALLOW_COPY_AND_ASSIGN(inputMatrix);
};
}
}
#include<Detail/inputMatrix-inl.h>
#endif
