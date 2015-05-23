/*
 * SMC_higher_order.hpp
 *
 *  Created on: 22-May-2015
 *      Author: nilxwam
 */

#ifndef SMC_HIGHER_ORDER_HPP_
#define SMC_HIGHER_ORDER_HPP_

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
#include <samlibs.h>

#include <math.h>

using namespace barrett;
using namespace systems;

template<size_t DOF>
class SMC_higher_order: public System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	/*The sliding mode controller requires the following inputs
	 * Reference Jp, Jv, Ja
	 * Actual System Jp, Jv for feedback
	 * M matrix
	 * C matrix
	 * G matrix (optional for now)
	 * */

	/*Output of this system is
	 * Torque*/
public:
//	Input<bool> Status;
	Input<jp_type> referencejpInput;
	Input<jv_type> referencejvInput;
	Input<ja_type> referencejaInput;
public:
	Input<jp_type> feedbackjpInput;
	Input<jv_type> feedbackjvInput;
public:
	Input<Eigen::Matrix4d> M;
	Input<Eigen::Vector4d> C;
public:
	Output<jt_type> controlOutput;

protected:
	typename Output<jt_type>::Value* controlOutputValue;

public:
	SMC_higher_order(/*systems::ExecutionManager* em*/bool status,
			const Eigen::Matrix4d coeff, const Eigen::Vector4d delta,
			const Eigen::Matrix4d A, const Eigen::Matrix4d B, const float P,
			const float Q,
			const std::string& sysName = "Slidingmode_Controller") :
			System(sysName), referencejpInput(this), referencejvInput(this), referencejaInput(
					this), feedbackjpInput(this), feedbackjvInput(this), M(
					this), C(this),

			controlOutput(this, &controlOutputValue), STATUS(status), Coeff(
					coeff), Delta(delta), A(A), B(B), P(P), Q(Q) {
		inv_B = B.inverse();
	}
	virtual ~SMC_higher_order() {
		this->mandatoryCleanUp();
	}

public:

	float sign(float x) {

		float A;
		if (x > 0) {
			A = 1;
		} else if (x < 0) {
			A = -1;
		} else if (x == 0) {
			A = 0;
		}
		return A;
	}

protected:
	Eigen::Matrix4d A;
	Eigen::Matrix4d B;
	Eigen::Matrix4d inv_B;
	float P, Q;
	bool STATUS;
	Eigen::Matrix4d Coeff;
	Eigen::Vector4d Delta;
	Eigen::Matrix4d M_inside;
	Eigen::Vector4d C_inside;
	jt_type jt_out;
	jp_type jp_sys, jp_ref, ep;
	jv_type jv_sys, jv_ref, ev;
	ja_type ja_ref;
	Eigen::Vector4d S, tmp_p, tmp_v, tmp_ev, tmp_ep, tmp_control, tmp_aref,
			tmp_pref, tmp_vref, jt_out_tmp;

	Eigen::Vector4d E_PQ;
	Eigen::Vector4d ED_PQ;
	Eigen::Vector4d E2_PQ;
	Eigen::Matrix4d D1;
	Eigen::Matrix4d D2;
	int i;
	Eigen::Vector4d ep_vector;

	virtual void operate() {
		/*Taking reference values from the input terminal of this system*/
		jp_ref = this->referencejpInput.getValue();
		jv_ref = this->referencejvInput.getValue();
		ja_ref = this->referencejaInput.getValue();
		/*Taking feedback values from the input terminal of this system*/
		jp_sys = this->feedbackjpInput.getValue();
		jv_sys = this->feedbackjvInput.getValue();
		/*Taking M and C values from the input terminal of this system*/
		M_inside = this->M.getValue();
		C_inside = this->C.getValue();

		ep = jp_sys - jp_ref;
		ev = jv_sys - jv_ref;

		D1 = Eigen::Matrix4d::Zero();
		D2 = Eigen::Matrix4d::Zero();

		for (i = 0; i < 4; i++) {
			E_PQ[i] = sign(ep(i)) * pow(fabs(ep(i)), P / Q);
			ED_PQ[i] = sign(ev(i)) * pow(fabs(ev(i)), P / Q);
			E2_PQ[i] = sign(ev(i)) * pow(fabs(ev(i)), 2.0 - (P / Q));
			D1(i, i) = sign(ev(i)) * pow(fabs(ev(i)), (P / Q) - 1.0);
			D2(i, i) = sign(ep(i)) * pow(fabs(ep(i)), (P / Q) - 1.0);
			ep_vector(i) = ep(i);
		}

		tmp_p << jp_sys[0], jp_sys[1], jp_sys[2], jp_sys[3];
		tmp_v << jv_sys[0], jv_sys[1], jv_sys[2], jv_sys[3];
		tmp_aref << ja_ref[0], ja_ref[1], ja_ref[2], ja_ref[3];
		tmp_vref << jv_ref[0], jv_ref[1], jv_ref[2], jv_ref[3];
		tmp_pref << jp_ref[0], jp_ref[1], jp_ref[2], jp_ref[3];
		tmp_ev << ev[0], ev[1], ev[2], ev[3];
		tmp_ep << ep[0], ep[1], ep[2], ep[3];


		S = B * ED_PQ + A * E_PQ + ep_vector;

		tmp_control[0] = S[0] / (fabs(S[0]) + Delta[0]);
		tmp_control[1] = S[1] / (fabs(S[1]) + Delta[1]);
		tmp_control[2] = S[2] / (fabs(S[2]) + Delta[2]);
		tmp_control[3] = S[3] / (fabs(S[3]) + Delta[3]);

		if (STATUS == true) {
			jt_out_tmp = C_inside
					+ M_inside
							* (tmp_aref
									- inv_B
											* (Eigen::Matrix4d::Identity()
													* (Q / P) + A * D2) * E2_PQ)
					- Coeff * tmp_control;

		} else
			jt_out_tmp = Eigen::Vector4d::Zero();

		jt_out[0] = jt_out_tmp[0];
		jt_out[1] = jt_out_tmp[1];
		jt_out[2] = jt_out_tmp[2];
		jt_out[3] = jt_out_tmp[3];

		controlOutputValue->setData(&jt_out);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(SMC_higher_order);
};

#endif /* SMC_HIGHER_ORDER_HPP_ */
