/*
 * GMMTrajectory.h
 *
 *  Created on: 18-Dec-2014
 *      Author: mobman
 */

#ifndef GMMTRAJECTORY_H_
#define GMMTRAJECTORY_H_
//#include <eigen3/Eigen/Core>
#include <barrett/units.h>
#include <barrett/systems.h>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/system.h>
//#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <GMM.h>
using namespace barrett;


namespace isl{

class GMMTrajectory: public systems::System {

	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;



public:
	explicit GMMTrajectory(systems::ExecutionManager* em,double Rho_0, double K,const std::string& sysName="GMMTrajectory");
	virtual ~GMMTrajectory();
	typedef Eigen::VectorXd vecXD;
	typedef math::Matrix<3,1, double> vec3D;

public:
	Input<cp_type> cp_input;
	Output<cp_type> cp_output;
	Output<cv_type> cv_output;


#ifdef DEBUG_GMM_
	Output<cv_type> gmm_out;
	Output<double> diff_norm_out;
	Output<vec3D> f_out, u_out;


protected:
	Output<cv_type>::Value* gmm_out_val;
	Output<double>::Value* diff_norm_out_val;
	Output<vec3D>::Value *f_outVal, *u_outVal;
private:
	cv_type gmm_cv_tmp;
	double difnrm;
	vec3D f, u;
#endif


protected:
	Output<cp_type>::Value* cp_out_val;
    Output<cv_type>::Value* cv_out_val;


private:
	GMM gmm;
	math::FirstOrderFilter<cv_type> cvFilter;
	cp_type cp_tmp;
	cv_type cv_tmp;
	vecXD cp_ref, cp_real;
	double Ts, gain, nrm;
	static const double MAX_VEL = 0.07;

protected:
	virtual void operate();
public:
	void start(const cp_type& curr_cp);
	void stop();
	void setLowpass(double omega);




private:
	DISALLOW_COPY_AND_ASSIGN(GMMTrajectory);
};
}
#include<Detail/GMMTrajectory-inl.h>
#endif /* GMMTRAJECTORY_H_ */
