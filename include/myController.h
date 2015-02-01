#ifndef MYCONTROLLER_H_
#define MYCONTROLLER_H_


#include <eigen3/Eigen/Core>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/controller.h>
#include <samlibs.h>
#include <tsk.h>
#include <Dynamics.h>

namespace barrett {
namespace systems {



template<int NJOINTS, int NR, typename InputType1, typename InputType2,
		 typename OutputType = typename InputType1::actuator_type,
		 typename MathTraits = math::Traits<InputType1> >
class myController : public Controller<InputType1, OutputType> {



public:
	typedef typename MathTraits::unitless_type unitless_type_jp;
	typedef math::Matrix<NJOINTS,NJOINTS,double> Mat2D_type;
	typedef  typename units::JointTorques<NJOINTS>::type criticTorque_type;
	typedef math::Matrix<2*NJOINTS,NJOINTS, double> B_type;
	typedef typename math::Vector<NJOINTS*NR, double>::type vec_type;


public:

	   System::Input<InputType2> wamJVIn;
	   System::Input<Mat2D_type> mMat2Dinp;

	   System::Output<criticTorque_type> criticconOp;
//	   System::Output<gx_type> gxrow;

	   System::Output<vec_type> dataVecOutput;
	   //To make available critic torque value


protected:
	   typename System::Output<criticTorque_type>::Value* criticconOpValue;
//	   typename System::Output<gx_type>::Value* gxrowVal;

	   typename System::Output<vec_type>::Value* dataVecOutVal; // For additional usage


private:
	   criticTorque_type ctorque;
	   Mat2D_type mMat2D;
	   B_type Bmat_2J;
	   Eigen::Matrix<double,NJOINTS*2, 1> x,e;
	   Eigen::Matrix<double,NJOINTS, NR> u0_mat;
	   Eigen::Matrix<double,NJOINTS, 1>  u0,eVel, ePos, kp_comp, kd_comp;

	   Eigen::Matrix<double, NJOINTS,NJOINTS > Rinv;
	   Sam::tsk<NJOINTS*2, NJOINTS, NR> fuz;
	   vec_type data;


private:

	size_t rCount;
	double omega1, omega2;

public:
	void setKpKd_comp(double val1, double val2);




public:

	explicit myController(const std::string& wFname="./FuzzParams/weight.txt",const std::string& consFname="./FuzzParams/cons.txt",
	    const std::string& gaussFname="./FuzzParams/gaussParams.txt",const std::string& rMatFname="./FuzzParams/Rmat.txt", const std::string& sysName = "myController");
	explicit myController(const libconfig::Setting& setting, const std::string& wFname="./FuzzParams/weight.txt",const std::string& consFname="./FuzzParams/cons.txt",
        const std::string& gaussFname="./FuzzParams/gaussParams.txt",const std::string& rMatFname="./FuzzParams/Rmat.txt", const std::string& sysName = "myController" );

	virtual ~myController(){
		this->mandatoryCleanUp();
	}

  void setFromConfig(const libconfig::Setting& setting);

  ////////////////////////////////////////////////////////////////////////////////////////
  //  test for myController
  //================================================================================
  void setKp(const unitless_type_jp& proportionalGains);
  void setKi(const unitless_type_jp& integralGains);
  void setKd(const unitless_type_jp& derivitiveGains);
  void setIntegratorState(const unitless_type_jp& integratorState);
  void setIntegratorLimit(const unitless_type_jp& intSaturations);
  void resetIntegrator();
  unitless_type_jp& getKp() {  return kp;  }
  const unitless_type_jp& getKp() const {  return kp;  }
  unitless_type_jp& getKi() {  return ki;  }
  const unitless_type_jp& getKi() const {  return ki;  }
  unitless_type_jp& getKd() {  return kd;  }
  const unitless_type_jp& getKd() const {  return kd;  }
  const unitless_type_jp& getIntegratorState() const {  return intError;  }
  unitless_type_jp& getIntegratorLimit() {  return intErrorLimit;  }
  const unitless_type_jp& getIntegratorLimit() const {  return intErrorLimit;  }


  Eigen::Matrix<double, NJOINTS, NJOINTS>& getRinv(){
    return Rinv;
  }



  //////////////////////////////////////////////////////////////////////////////////////
  
  void setControlSignalLimit(const unitless_type_jp& csSaturations);
  OutputType& getControlSignalLimit() {  return controlSignalLimit;  }
  const OutputType& getControlSignalLimit() const {  return controlSignalLimit;  }
  
  void resetRcount(){rCount=0;}



protected:
  void setSamplePeriod(double timeStep);

  virtual void operate();
  virtual void onExecutionManagerChanged() {
    Controller<InputType1, OutputType>::onExecutionManagerChanged();  // First, call super
    getSamplePeriodFromEM();
  }
  void setRinv(const char* RmatFile);
  void setU0(const char* U0File);
  double T_s;
  InputType1 error_jp, error_1_jp, currentJp, prevJp;
  InputType2 error_jv, error_1_jv, currentJv;
  ///////////////////////////////////////////////////////////////////////////////
  //Test for myController
  //=========================================================================
  unitless_type_jp intError, intErrorLimit;
  unitless_type_jp kp, ki, kd, pE, vE, ref;
  ///////////////////////////////////////////////////////////////////////////////
  
  OutputType controlSignal, controlSignalLimit;
//	unitless_type_jp controlSignal, controlSignalLimit;

  void getSamplePeriodFromEM();

private:
  DISALLOW_COPY_AND_ASSIGN(myController);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(MathTraits::RequiresAlignment);






};

}
}
#include <Detail/myController-inl.h>
#endif
