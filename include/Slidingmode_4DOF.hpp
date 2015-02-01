/*
 * Slidingmode_4DOF.hpp
 *
 *  Created on: 31-Jan-2015
 *      Author: nilxwam
 */

#ifndef SLIDINGMODE_4DOF_HPP_
#define SLIDINGMODE_4DOF_HPP_

#include <barrett/math/traits.h>
#include <list>
#include <barrett/units.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <Dynamics.h>

namespace barrett {
namespace systems {

template<size_t DOF>
class Slidingmode_Controller: public System , Nilu::Dynamics {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// IO
public:
	Input<jp_type> referencejpInput;
public:
	Input<jv_type> referencejvInput;
public:
	Input<ja_type> referencejaInput;

public:
	Input<jp_type> feedbackjpInput;
public:
	Input<jv_type> feedbackjvInput;

public:
	Output<jt_type> controlOutput;

protected:
	typename Output<jt_type>::Value* controlOutputValue;

public:
	explicit Slidingmode_Controller(const Eigen::Matrix4d lamda,
			const double coeff, const double delta, const double gravity,const std::string& sysName =
					"Slidingmode_Controller") :
			System(sysName), referencejpInput(this), referencejvInput(this), referencejaInput(this), feedbackjpInput(
					this), feedbackjvInput(this), Lamda(lamda), Coeff(coeff), Delta(delta), Gravity(gravity),

			controlOutput(this, &controlOutputValue) {
	}
	virtual ~Controller() {
		mandatoryCleanUp();
	}

protected:
	double Coeff, Delta, Gravity;
	jt_type jt_out;
	jp_type jp_sys, jp_ref, ep;
	jv_type jv_sys, jv_ref, ev;
	ja_type ja_ref;
	Eigen::Matrix4d K, Lamda, M;
	Eigen::Vector4d S, tmp_p, tmp_v, tmp_ev, tmp_ep, C, G, tmp_control, tmp_aref, tmp_pref;


	virtual void operate() {
		jp_ref = this->referencejpInput.getValue();
		jv_ref = this->referencejvInput.getValue();
		ja_ref = this->referencejaInput.getValue();

		jp_sys = this->feedbackjpInput.getValue();
		jv_sys = this->feedbackjvInput.getValue();
		ep = jp_sys - jp_ref;
		ev = jv_sys - jv_ref;
		K = Coeff * Lamda;

		tmp_p << jp_sys[0], jp_sys[1], jp_sys[2], jp_sys[3];
		tmp_v << jv_sys[0], jv_sys[1], jv_sys[2], jv_sys[3];
		tmp_aref << ja_ref[0], ja_ref[1], ja_ref[2], ja_ref[3];
		tmp_pref << jp_ref[0], jp_ref[1], jp_ref[2], jp_ref[3];
		tmp_ev << ev[0], ev[1], ev[2], ev[3];
		tmp_ep << ep[0], ep[1], ep[2], ep[3];



		S = tmp_ev + Lamda * tmp_ep;

		C = Dynamics::makeCvec(tmp_p,
				tmp_v);
		G = Gravity*Dynamics::makeGx(tmp_p);
		M = Dynamics::makemassMat(tmp_p);


		tmp_control[0] = S[0]/(fabs(S[0])+ Delta);
		tmp_control[1] = S[1]/(fabs(S[1])+ Delta);
		tmp_control[2] = S[2]/(fabs(S[2])+ Delta);
		tmp_control[3] = S[3]/(fabs(S[3])+ Delta);

		jt_out = C + G + M*(tmp_aref - Lamda*(tmp_v - tmp_pref) - K*tmp_control);


		this->controlOutput->setData(&jt_out);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(Slidingmode_Controller);
};

}
}

#endif /* SLIDINGMODE_4DOF_HPP_ */
