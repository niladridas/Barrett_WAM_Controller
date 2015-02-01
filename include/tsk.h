/*
 * tsk.h
 *
 *  Created on: 22-Apr-2013
 *      Author: mobman
 */

/* In this class, the membership function is considered as gaussian which is exp(-.5*((x-c)/sigma)^2)
 *
 *
 */


#ifndef TSK_H_
#define TSK_H_
#include <eigen3/Eigen/Core>
#include <constants.h>
namespace Sam {
template<int xDim, int uDim, int nR>
class tsk {
public:
	tsk(const char* consModel, const char* consCost, const char* fuzParam, const std::string tskName = "Dynamic Model");
	virtual ~tsk();


private:

	double** centers;
	double** sigmas;
	double** mu;  //membership value
	Eigen::Matrix<double, nR, 1> W, R;      // normalised Rule membership value
	Eigen::Matrix<double, xDim, xDim>* A;
	Eigen::Matrix<double, xDim, uDim>* B;
	Eigen::Matrix<double, xDim, xDim>* Pmat;
	//For manipulator Dynamics
	Eigen::Matrix<double, xDim/2, xDim/2> mLocal, a;
	Eigen::Matrix<double, xDim/2, xDim/2> cLocal, b;
	size_t CSdim;
	// For tsk Cost
	Eigen::Matrix<double, 1, xDim*(xDim+1)/2> xOrth, dXorth;
	Eigen::Matrix<double, nR*(xDim*(xDim+1)/2), 1> Pvec;
	Eigen::Matrix<double, xDim, nR*(xDim*(xDim+1)/2)> delWx;

	bool ruleBaseFlag;

public:
	// For tsk Cost
	Eigen::Matrix<double, xDim,1> delVdelX;
	Eigen::Matrix<double, xDim, xDim> model_A, model_P;
	Eigen::Matrix<double, xDim, uDim> model_B;

	//For manipulator Dynamics
	Eigen::Matrix<double, xDim/2, xDim/2> M;
	Eigen::Matrix<double, xDim/2, xDim/2> C;



public:
	const Eigen::Matrix<double, xDim, xDim>* getA() const {return A;}
	const Eigen::Matrix<double, xDim, xDim> getModelA() const {return model_A;}
	const Eigen::Matrix<double, xDim, uDim>* getB() const {return B;}
	const Eigen::Matrix<double, xDim, uDim> getModelB() const {return model_B;}
	const Eigen::Matrix<double, xDim/2, xDim/2>& getMassmat() const {return M;}
	const Eigen::Matrix<double, xDim/2, xDim/2>& getCmat() const {return C;}
	const Eigen::Matrix<double, nR, 1>& getRule_W(){return W;}
//	const Eigen::Matrix<double, nR, xDim>& getCenters() const {return centers;}
//	const Eigen::Matrix<double, nR, xDim>& getSigmas() const {return sigmas;}
	void computeFuzModel(const Eigen::Matrix<double, xDim, 1>& x, const Eigen::Matrix<double, uDim, 1>& u);
	void makeDelJDelx( const Eigen::Matrix<double, xDim, 1>& e, const Eigen::Matrix<double, xDim, 1>& x);
	void makeDelJDelx_Inte_Lyp( const Eigen::Matrix<double, xDim, 1>& e, const Eigen::Matrix<double, xDim, 1>& x);
	Eigen::Matrix<double, nR, 1>& getR(){return R;}
	void calcXdot(const Eigen::Matrix<double, xDim, 1>& x, const Eigen::Matrix<double, uDim, 1>& u, Eigen::Matrix<double, xDim, 1 >& xdot);
	double output(const Eigen::Matrix<double, xDim, 1>& x);
private:
	void calculateMu(const Eigen::Matrix<double, xDim, 1>& x, const Eigen::Matrix<double, uDim, 1>& u); // membership value (mu)
	void calculateMu(const Eigen::Matrix<double, xDim, 1>& x);
	void calculateW();   // normalized membership value (sigma)



};



} /* namespace sam */
#include<Detail/tsk-inl.h>
#endif /* TSK_H_ */
