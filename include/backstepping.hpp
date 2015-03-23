/*
 * backstepping.hpp
 *
 *  Created on: 01-Mar-2015
 *      Author: nilxwam
 */

#ifndef BACKSTEPPING_HPP_
#define BACKSTEPPING_HPP_

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
class Backstepping_Controller: public System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Input<jp_type> referencejpInput;
	Input<jv_type> referencejvInput;
	Input<ja_type> referencejaInput;
public:
	Input<jp_type> feedbackjpInput; //----------
	Input<jv_type> feedbackjvInput; ///--------------
public:
	Input<Eigen::Matrix4d> M; //-----------------
	Input<Eigen::Vector4d> C; //-----------------
public:
	Output<jt_type> controlOutput;

protected:
	typename Output<jt_type>::Value* controlOutputValue;

public:
	Backstepping_Controller(Eigen::Matrix4d K1, Eigen::Matrix4d K2,
			const std::string& sysName = "Slidingmode_Controller") :
			System(sysName), referencejpInput(this), referencejvInput(this), referencejaInput(
					this), feedbackjpInput(this), feedbackjvInput(this), M(
					this), C(this),

			controlOutput(this, &controlOutputValue),K1(K1),K2(K2) {

//		    }
	}
	virtual ~Backstepping_Controller() {
		this->mandatoryCleanUp();
	}

protected:
	Eigen::Matrix4d M_inside;
	Eigen::Vector4d C_inside;
	Eigen::Matrix4d Identity;

	jt_type jt_out;
	jp_type jp_sys, jp_ref;
	jv_type jv_sys, jv_ref;
	ja_type ja_ref;
	Eigen::Matrix4d K1;
	Eigen::Matrix4d K2;

	Eigen::Vector4d S, tmp_p, tmp_v, tmp_control, tmp_aref, tmp_pref, tmp_vref,
			jt_out_tmp;

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

		tmp_p << jp_sys[0], jp_sys[1], jp_sys[2], jp_sys[3];
		tmp_v << jv_sys[0], jv_sys[1], jv_sys[2], jv_sys[3];
		tmp_aref << ja_ref[0], ja_ref[1], ja_ref[2], ja_ref[3];
		tmp_vref << jv_ref[0], jv_ref[1], jv_ref[2], jv_ref[3];
		tmp_pref << jp_ref[0], jp_ref[1], jp_ref[2], jp_ref[3];

		jt_out_tmp = Eigen::Vector4d::Zero();

		jt_out_tmp = C_inside
				+ M_inside
						* (-(Identity.setIdentity(4, 4) + K1 * K2)
								* (tmp_p - tmp_pref) - (K1 + K2) * tmp_v
								+ (K1 + K2) * tmp_vref + tmp_aref);

		jt_out[0] = jt_out_tmp[0];
		jt_out[1] = jt_out_tmp[1];
		jt_out[2] = jt_out_tmp[2];
		jt_out[3] = jt_out_tmp[3];

		controlOutputValue->setData(&jt_out);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(Backstepping_Controller);
};

#endif /* BACKSTEPPING_HPP_ */
