/*
 * Optimal_Sliding_Mode.hpp
 *
 *  Created on: 25-Feb-2015
 *      Author: nilxwam
 */

#ifndef OPTIMAL_SLIDING_MODE_HPP_
#define OPTIMAL_SLIDING_MODE_HPP_

#include <barrett/math/traits.h>
#include <list>
#include <barrett/units.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
//
#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/controller.h>
#include <samlibs.h>

#include <math.h>

using namespace barrett;
using namespace systems;
using namespace Eigen;

template<size_t DOF>
class Optimal_Sliding_Mode: public System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Input<jp_type> referencejpInput; // qd
	Input<jv_type> referencejvInput; // qdd
	Input<ja_type> referencejaInput; // qddd
	Input<jp_type> feedbackjpInput;
	Input<jv_type> feedbackjvInput;
	Input<Eigen::Matrix4d> M;
	Input<Eigen::Vector4d> C;
	Input<Eigen::Matrix4d> Md;
	Input<Eigen::Vector4d> Cd;
public:
	Output<jt_type> optslidecontrolOutput;

protected:
	typename Output<jt_type>::Value* optslidecontrolOutputValue;

public:
	Optimal_Sliding_Mode(const Eigen::Vector3d W1, const Eigen::Vector3d W2,
			const Eigen::Vector3d W3, const Eigen::Vector3d W4,
			const Eigen::Vector4d phi, const Eigen::Vector4d c,
			const Eigen::Vector4d K, const double b, const double del,
			const Eigen::Vector3f Gtilde, const std::string& sysName =
					"Optimal_Sliding_Mode") :
			System(sysName), referencejpInput(this), referencejvInput(this), referencejaInput(
					this), feedbackjpInput(this), feedbackjvInput(this), M(
					this), C(this), Md(this), Cd(this),

			optslidecontrolOutput(this, &optslidecontrolOutputValue), W1(W1), W2(
					W2), W3(W3), W4(W4), phi(phi), c(c), K(K), b(b), Gtilde(
					Gtilde), del(del) {

	}
	virtual ~Optimal_Sliding_Mode() {
		this->mandatoryCleanUp();
	}
// Position states and Velocity states
public:
	Eigen::Vector3d W1;
	Eigen::Vector3d W2;
	Eigen::Vector3d W3;
	Eigen::Vector3d W4;
	Eigen::Vector4d phi;
	jp_type tmp_theta_pos;
	jv_type tmp_theta_vel;
	Eigen::Matrix4d M_tmp;
	Eigen::Vector4d C_tmp;
	Eigen::Matrix4d Md_tmp;
	Eigen::Vector4d Cd_tmp;
	Eigen::Vector4d ThetaInput;
	Eigen::Vector4d ThetadotInput;
	Eigen::Vector4d qd;
	Eigen::Vector4d qdd;
	Eigen::Vector4d qddd;
	Eigen::Vector4d e;
	Eigen::Vector4d ed;
	Eigen::Vector4d c;
	Eigen::Vector4d K;
	double b;
	Eigen::Vector4d eta;
	Eigen::VectorXd Xtilde;
	Eigen::Vector4d F;
	Eigen::Vector4d Fd;
	Eigen::Vector4d rho;
	Eigen::Vector4d fbar;
	int i;
	Eigen::Vector3f Gtilde;
	Eigen::Vector3d Ftilde1;
	Eigen::Vector3d Ftilde2;
	Eigen::Vector3d Ftilde3;
	Eigen::Vector3d Ftilde4;
	Eigen::RowVector3d s1;
	Eigen::RowVector3d s2;
	Eigen::RowVector3d s3;
	Eigen::RowVector3d s4;
	Eigen::Vector3d DVDX1;
	Eigen::Vector3d DVDX2;
	Eigen::Vector3d DVDX3;
	Eigen::Vector3d DVDX4;
	Eigen::Vector3d DVDW1;
	Eigen::Vector3d DVDW2;
	Eigen::Vector3d DVDW3;
	Eigen::Vector3d DVDW4;
	double r1, r2, r3, r4;
	Eigen::Vector3d W1dot;
	Eigen::Vector3d W2dot;
	Eigen::Vector3d W3dot;
	Eigen::Vector3d W4dot;
	Eigen::Vector4d Ueq;
	Eigen::Vector4d Ud;
	Eigen::Vector4d Tau;
	double ueq1, ueq2, ueq3, ueq4;
	double del;
	Eigen::Vector4d phidot;
	Eigen::Vector4d L;
	Eigen::Vector4d Lbar;
	jt_type torque_tmp;
	Eigen::MatrixXd M_tmpinverse;
	Eigen::MatrixXd Md_tmpinverse;

	virtual void operate() {
		tmp_theta_pos = this->feedbackjpInput.getValue();
		ThetaInput << tmp_theta_pos[0], tmp_theta_pos[1], tmp_theta_pos[2], tmp_theta_pos[3];
		M_tmp = this->M.getValue();
		Md_tmp = this->Md.getValue();
		tmp_theta_vel = this->feedbackjvInput.getValue();
		ThetadotInput << tmp_theta_vel[0], tmp_theta_vel[1], tmp_theta_vel[2], tmp_theta_vel[3];
		C_tmp = this->C.getValue();
		Cd_tmp = this->Cd.getValue();

		// Reference position, velocity and accelerations
		qd[0] = this->referencejpInput.getValue()[0];
		qd[1] = this->referencejpInput.getValue()[1];
		qd[2] = this->referencejpInput.getValue()[2];
		qd[3] = this->referencejpInput.getValue()[3];

		qdd[0] = this->referencejvInput.getValue()[0];
		qdd[1] = this->referencejvInput.getValue()[1];
		qdd[2] = this->referencejvInput.getValue()[2];
		qdd[3] = this->referencejvInput.getValue()[3];

		qddd[0] = this->referencejaInput.getValue()[0];
		qddd[1] = this->referencejaInput.getValue()[1];
		qddd[2] = this->referencejaInput.getValue()[2];
		qddd[3] = this->referencejaInput.getValue()[3];
//
		//Position error
		e = ThetaInput - qd;
		//Velocity error
		ed = ThetadotInput - qdd;
		//Eta
		eta = K / b;
		//The error matrix
		Xtilde.resize(8, 1);
		Xtilde << e, ed;

		Md_tmpinverse = Md_tmp.inverse();
		M_tmpinverse = M_tmp.inverse();


		F = -M_tmpinverse * (C_tmp);
		Fd = -Md_tmpinverse * (Cd_tmp);

		rho = F - Fd;

		// The fbar vector
		for (i = 0; i < 4; i = i + 1) {
			fbar[i] = rho[i] + eta[i] * c[i] * Xtilde[i]
					+ (c[i] + eta[i]) * Xtilde[4 + i] + eta[i] * phi[i];
		}

//

		Ftilde1 << Xtilde[4], -eta[0] * c[0] * Xtilde[0]
				- (c[0] + eta[0]) * Xtilde[4] - eta[0] * phi[0], 0;
		Ftilde2 << Xtilde[5], -eta[1] * c[1] * Xtilde[1]
				- (c[1] + eta[1]) * Xtilde[5] - eta[1] * phi[1], 0;
		Ftilde3 << Xtilde[6], -eta[2] * c[2] * Xtilde[2]
				- (c[2] + eta[2]) * Xtilde[6] - eta[2] * phi[2], 0;
		Ftilde4 << Xtilde[7], -eta[3] * c[3] * Xtilde[3]
				- (c[3] + eta[3]) * Xtilde[7] - eta[3] * phi[3], 0;

		//The L and Lbar vectors

		for (i = 0; i < 4; i = i + 1) {
			L[i] = 0.5
					* (Xtilde[i] * Xtilde[i] + Xtilde[4 + i] * Xtilde[4 + i]);
			Lbar[i] = L[i] + fbar[i] * fbar[i];

		}

		// The gradient matrices
		DVDX1
				<< (W1[0] * Xtilde[0] + W1[1] * Xtilde[4] + W1[2] * phi[0])
						* W1[0], (W1[0] * Xtilde[0] + W1[1] * Xtilde[4]
				+ W1[2] * phi[0]) * W1[1], (W1[0] * Xtilde[0]
				+ W1[1] * Xtilde[4] + W1[2] * phi[0]) * W1[2];
		DVDX2
				<< (W2[0] * Xtilde[1] + W2[1] * Xtilde[5] + W2[2] * phi[1])
						* W2[0], (W2[0] * Xtilde[1] + W2[1] * Xtilde[5]
				+ W2[2] * phi[1]) * W2[1], (W2[0] * Xtilde[1]
				+ W2[1] * Xtilde[5] + W2[2] * phi[1]) * W2[2];
		DVDX3
				<< (W3[0] * Xtilde[2] + W3[1] * Xtilde[6] + W3[2] * phi[2])
						* W3[0], (W3[0] * Xtilde[2] + W3[1] * Xtilde[6]
				+ W3[2] * phi[2]) * W3[1], (W3[0] * Xtilde[2]
				+ W3[1] * Xtilde[6] + W3[2] * phi[2]) * W3[2];
		DVDX4
				<< (W4[0] * Xtilde[3] + W4[1] * Xtilde[7] + W4[2] * phi[3])
						* W4[0], (W4[0] * Xtilde[3] + W4[1] * Xtilde[7]
				+ W4[2] * phi[3]) * W4[1], (W4[0] * Xtilde[3]
				+ W4[1] * Xtilde[7] + W4[2] * phi[3]) * W4[2];

		DVDW1
				<< (W1[0] * Xtilde[0] + W1[1] * Xtilde[4] + W1[2] * phi[0])
						* Xtilde[0] + W1[0], (W1[0] * Xtilde[0]
				+ W1[1] * Xtilde[4] + W1[2] * phi[0]) * Xtilde[4] + W1[1], (W1[0]
				* Xtilde[0] + W1[1] * Xtilde[4] + W1[2] * phi[0]) * phi[0]
				+ W1[2];
		DVDW2
				<< (W2[0] * Xtilde[1] + W2[1] * Xtilde[5] + W2[2] * phi[1])
						* Xtilde[1] + W2[0], (W2[0] * Xtilde[1]
				+ W2[1] * Xtilde[5] + W2[2] * phi[1]) * Xtilde[5] + W2[1], (W2[0]
				* Xtilde[1] + W2[1] * Xtilde[5] + W2[2] * phi[1]) * phi[1]
				+ W2[2];
		DVDW3
				<< (W3[0] * Xtilde[2] + W3[1] * Xtilde[6] + W3[2] * phi[2])
						* Xtilde[2] + W3[0], (W3[0] * Xtilde[2]
				+ W3[1] * Xtilde[6] + W3[2] * phi[2]) * Xtilde[6] + W3[1], (W3[0]
				* Xtilde[2] + W3[1] * Xtilde[6] + W3[2] * phi[2]) * phi[2]
				+ W3[2];
		DVDW4
				<< (W4[0] * Xtilde[3] + W4[1] * Xtilde[7] + W4[2] * phi[3])
						* Xtilde[3] + W4[0], (W4[0] * Xtilde[3]
				+ W4[1] * Xtilde[7] + W4[2] * phi[3]) * Xtilde[7] + W4[1], (W4[0]
				* Xtilde[3] + W4[1] * Xtilde[7] + W4[2] * phi[3]) * phi[3]
				+ W4[2];
////
		s1 = DVDW1.transpose();
		s2 = DVDW2.transpose();
		s3 = DVDW3.transpose();
		s4 = DVDW4.transpose();

		Eigen::Matrix3d tmp_Gtilde;
		tmp_Gtilde << Gtilde[0] * Gtilde[0], Gtilde[0] * Gtilde[1], Gtilde[0]
				* Gtilde[2], Gtilde[1] * Gtilde[0], Gtilde[1] * Gtilde[1], Gtilde[1]
				* Gtilde[2], Gtilde[2] * Gtilde[0], Gtilde[2] * Gtilde[1], Gtilde[2]
				* Gtilde[2];

//
		r1 = -Lbar[0] + 0.25 * DVDX1.dot(tmp_Gtilde * DVDX1)
				- DVDX1.dot(Ftilde1);
		r2 = -Lbar[1] + 0.25 * DVDX2.dot(tmp_Gtilde * DVDX2)
				- DVDX2.dot(Ftilde2);
		r3 = -Lbar[2] + 0.25 * DVDX3.dot(tmp_Gtilde * DVDX3)
				- DVDX3.dot(Ftilde3);
		r4 = -Lbar[3] + 0.25 * DVDX4.dot(tmp_Gtilde * DVDX4)
				- DVDX4.dot(Ftilde4);

		W1dot = s1.transpose() / (s1.dot(s1)) * r1;
		W2dot = s2.transpose() / (s2.dot(s2)) * r2;
		W3dot = s3.transpose() / (s3.dot(s3)) * r3;
		W4dot = s4.transpose() / (s4.dot(s4)) * r4;
////
		ueq1 = 0.5
				* (W1[0] * (W1[2] - W1[1]) * Xtilde[0]
						+ W1[1] * (W1[2] - W1[1]) * Xtilde[4]
						+ W1[2] * (W1[2] - W1[1]) * phi[0]);
		ueq2 = 0.5
				* (W2[0] * (W2[2] - W2[1]) * Xtilde[1]
						+ W2[1] * (W2[2] - W2[1]) * Xtilde[5]
						+ W2[2] * (W2[2] - W2[1]) * phi[1]);
		ueq3 = 0.5
				* (W3[0] * (W3[2] - W3[1]) * Xtilde[2]
						+ W3[1] * (W3[2] - W3[1]) * Xtilde[6]
						+ W3[2] * (W3[2] - W3[1]) * phi[2]);
		ueq4 = 0.5
				* (W4[0] * (W4[2] - W4[1]) * Xtilde[3]
						+ W4[1] * (W4[2] - W4[1]) * Xtilde[7]
						+ W4[2] * (W4[2] - W4[1]) * phi[3]);
//
// Final torque computations
		Ueq << ueq1, ueq2, ueq3, ueq4;

		Ud = qddd + Md_tmpinverse * Cd_tmp;

		Tau = M_tmp * (Ud + Ueq); // Final torque to system.
//

		phidot[0] = -fbar[0] - ueq1;
		phidot[1] = -fbar[1] - ueq2;
		phidot[2] = -fbar[2] - ueq3;
		phidot[3] = -fbar[3] - ueq4;

		phi = phi + phidot * del;
		W1 = W1 + W1dot * del;
		W2 = W2 + W2dot * del;
		W3 = W3 + W3dot * del;
		W4 = W4 + W4dot * del;

		torque_tmp[0] = Tau[0];
		torque_tmp[1] = Tau[1];
		torque_tmp[2] = Tau[2];
		torque_tmp[3] = Tau[3];

//		torque_tmp[0] = 0;
//		torque_tmp[1] = 0;
//		torque_tmp[2] = 0;
//		torque_tmp[3] = 0;

		optslidecontrolOutputValue->setData(&torque_tmp);



	}
private:
	DISALLOW_COPY_AND_ASSIGN(Optimal_Sliding_Mode);
}
;
#endif /* OPTIMAL_SLIDING_MODE_HPP_ */
