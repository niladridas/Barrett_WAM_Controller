#ifndef DYNAMICS_H_
#define DYNAMICS_H_

#include <eigen2/Eigen/Core>
#include <eigen2/Eigen/Array>
#include <eigen2/Eigen/LU>

#include <math.h>


class Dynamics {
public:
	Dynamics();
	virtual ~Dynamics() {
	}

	void makemassMat(const Eigen::Vector4d theta);
	void makeCvec(const Eigen::Vector4d theta, const Eigen::Vector4d thetaDot);
	void makeGx(const Eigen::Vector4d theta);

// Four link model

	void computeModel(const Eigen::Vector4d theta,
			const Eigen::Vector4d thetaDot, const Eigen::Vector4d thetaDotDot);

	const Eigen::Matrix4d getmassMat() const {
		return massMatrix;
	}

	const Eigen::Vector4d getCvec() const {
		return Cvec;
	}

	const Eigen::Vector4d getGx() const {
		return Gx;
	}

public:
	Eigen::Matrix4d massMatrix;
	Eigen::Vector4d Cvec;
	Eigen::Vector4d Gx;
	Eigen::Vector4d Tm;


};
/* namespace Sam */
#include<Detail/Dynamics-inl.h>
#endif /* DYNAMICS_H_ */
