#include <cassert>

#include <libconfig.h++>

#include <barrett/math/utils.h>
#include <barrett/thread/abstract/mutex.h>


namespace barrett {
namespace systems {
template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::myController(const std::string& wFname,const std::string& consFname,
    const std::string& gaussFname,const std::string& rMatFname,const std::string& sysName) :
	Controller<InputType1, OutputType>(sysName),
	wamJVIn(this), mMat2Dinp(this),  criticconOp(this, &criticconOpValue),dataVecOutput(this,&dataVecOutVal),/* pEout(this, &pEoutVal), vEout(this, &vEoutVal),refout(this, &refoutVal),
	Rout1(this, &RoutVal1), Rout2(this, &RoutVal2),*/ ctorque(0.0),mMat2D(Mat2D_type()), Bmat_2J(B_type()),x(Eigen::Matrix<double,NJOINTS*2, 1>()),e(Eigen::Matrix<double,NJOINTS*2, 1>()),
	u0_mat(Eigen::Matrix<double, NJOINTS, NR>()),u0(Eigen::Matrix<double, NJOINTS, 1>()),eVel(Eigen::Matrix<double, NJOINTS, 1>()), ePos(Eigen::Matrix<double,
	    NJOINTS, 1>()),	kp_comp(Eigen::Matrix<double, NJOINTS, 1>()), kd_comp(Eigen::Matrix<double, NJOINTS, 1>()),
	    Rinv(Eigen::Matrix<double, NJOINTS,NJOINTS >()),fuz(consFname.c_str(),wFname.c_str(),gaussFname.c_str()), data(0.0),

	rCount(0),omega1(0.0),omega2(0.0),
	T_s(0.0),  error_1_jp(0.0),currentJp(0.0), prevJp(0.0), currentJv(0.0),intError(0.0),
	intErrorLimit(0.0),	kp(0.0), ki(0.0), kd(0.0),pE(0.0), vE(0.0),controlSignal(0.0), controlSignalLimit(0.0)
	{
		getSamplePeriodFromEM();
		u0_mat.fill(0.0);
		x.fill(0.0);
		setRinv(rMatFname.c_str());
		setU0("./FuzzParams/u0.txt");
		Bmat_2J.fill(0.0);
		std::cout<<"***************************\n Using following files to create TSK object:\n "<<consFname<<"\n"<<gaussFname<<"\n"<<wFname<<"\n"<<rMatFname<<"\n";

	}



template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::myController(const libconfig::Setting& setting,const std::string& wFname,const std::string& consFname,
    const std::string& gaussFname, const std::string& rMatFname, const std::string& sysName) :
	Controller<InputType1, OutputType>(sysName),
	wamJVIn(this), mMat2Dinp(this),  criticconOp(this, &criticconOpValue),dataVecOutput(this,&dataVecOutVal),/*pEout(this, &pEoutVal), vEout(this, &vEoutVal), refout(this, &refoutVal),
	    Rout1(this, &RoutVal1), Rout2(this, &RoutVal2),*/ ctorque(0.0), mMat2D(Mat2D_type()), Bmat_2J(B_type()), x(Eigen::Matrix<double,NJOINTS*2, 1>()),e(Eigen::Matrix<double,NJOINTS*2, 1>()),
	    u0_mat(Eigen::Matrix<double, NJOINTS, NR>()), u0(Eigen::Matrix<double, NJOINTS, 1>()),eVel(Eigen::Matrix<double, NJOINTS, 1>()),ePos(Eigen::Matrix<double,
	        NJOINTS, 1>()), kp_comp(Eigen::Matrix<double, NJOINTS, 1>()), kd_comp(Eigen::Matrix<double, NJOINTS, 1>()),
	        Rinv(Eigen::Matrix<double, NJOINTS,NJOINTS >()),fuz("./FuzzParams/cons.txt","./FuzzParams/weight.txt", "./FuzzParams/gaussParams.txt"), data(0.0),

	    rCount(0),omega1(0.0),omega2(0.0),
	    T_s(0.0),  error_1_jp(0.0),currentJp(0.0), prevJp(0.0), currentJv(0.0),intError(0.0),
	    intErrorLimit(0.0), kp(0.0), ki(0.0), kd(0.0),pE(0.0), vE(0.0), controlSignal(0.0), controlSignalLimit(0.0)
{
	getSamplePeriodFromEM();
	setFromConfig(setting);
	setRinv(rMatFname.c_str());
	setU0("./FuzzParams/u0.txt");

	x.fill(0.0);
	Bmat_2J.fill(0.0);
	std::cout<<"***************************\n Using following files to create TSK object:\n "<<consFname<<"\n"<<gaussFname<<"\n"<<wFname<<"\n"<<rMatFname<<"\n";

}

template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
void myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::setRinv(const char* RmatFile){

	std::vector<std::vector<double> > r;
	size_t tr, tc;


	r=Sam::readFile<double>(RmatFile);
	tr=r.size();
	tc=r[0].size();
	if(!(tr==NJOINTS || tc== NJOINTS)){

		std::cout<<"File "<<RmatFile<<" is not proper for R matrix"<<std::endl;
		exit(EXIT_FAILURE);
	}

	for (size_t i=0; i<NJOINTS; i++){
		for (size_t j=0; j<NJOINTS; j++){
			Rinv(i,j)=r[i][j];
		}
	}

	Rinv=Rinv.inverse();

}

template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
void myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::setU0(const char* U0File){


    size_t tr, tc;


    std::vector<std::vector<double> > r=Sam::readFile<double>(U0File);
    tr=r.size();
    tc=r[0].size();
    if(!(tr==NJOINTS && tc== NR)){

        std::cout<<"File "<<U0File<<" is not proper for R matrix"<<std::endl;
        exit(EXIT_FAILURE);
    }

    for (size_t i=0; i<NJOINTS; i++){
        for (size_t j=0; j<NR; j++){
            u0_mat(i,j)=r[i][j];
        }
    }



}



template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
void myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::setFromConfig(const libconfig::Setting& setting)
{


  ///////////////////////////////////////////////////////////////
  // Test for myController
  //===========================================================
  if (setting.exists("kp")) {
		setKp(unitless_type_jp(setting["kp"]));
	}
	if (setting.exists("ki")) {
		setKi(unitless_type_jp(setting["ki"]));
	}
	if (setting.exists("kd")) {
		setKd(unitless_type_jp(setting["kd"]));
	}
	if (setting.exists("integrator_limit")) {
		setIntegratorLimit(unitless_type_jp(setting["integrator_limit"]));
	}
	
	//////////////////////////////////////////////////////////////////

	
	if (setting.exists("control_signal_limit")) {
		setControlSignalLimit(unitless_type_jp(setting["control_signal_limit"]));
	}
}

template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
void myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::setSamplePeriod(double timeStep)
{
	T_s = timeStep;
}


  
  ///////////////////////////////////////////////////////////////////////////////
  //Test for myController
  //============================================================================
  template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
void myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::setKp(const unitless_type_jp& proportionalGains)
{
	std::cout << "Our previous proportional control gain: kp = " << kp << std::endl;
	kp = proportionalGains;
	std::cout << "Our new proportional control gain: kp = " << kp << std::endl;
}

template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
void myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::setKi(const unitless_type_jp& integralGains)
{
	ki = integralGains;
}

template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
void myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::setKd(const unitless_type_jp& derivitiveGains)
{
	kd = derivitiveGains;
}

template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
void myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::setIntegratorState(
		const unitless_type_jp& integratorState)
{
	// intError is written and read in operate(), so it needs to be locked.
	BARRETT_SCOPED_LOCK(this->getEmMutex());
	intError = integratorState;
}

template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
void myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::setIntegratorLimit(
		const unitless_type_jp& intSaturations)
{
	intErrorLimit = intSaturations;
}


 template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
inline void myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::resetIntegrator()
{
	setIntegratorState(unitless_type_jp(0.0));
}
  /////////////////////////////////////////////////////////////////////////////////////////



  

 template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
void myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::setControlSignalLimit(
		const unitless_type_jp& csSaturations)
{
	controlSignalLimit = csSaturations;
}

 template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
 void myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::setKpKd_comp(double val1, double val2){
   omega1=val1;omega2= val2;
 }



  
template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
void myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::operate()

{

  typedef MathTraits MT;
  currentJp = this->feedbackInput.getValue();
  pE = MT::sub(this->referenceInput.getValue(), currentJp);
  currentJv = wamJVIn.getValue();


  vE=MT::div(MT::sub(pE, error_1_jp), T_s);

  mMat2D = mMat2Dinp.getValue();
  Bmat_2J(1,0)=mMat2D(0,0);
  Bmat_2J(1,1)=mMat2D(0,1);
  Bmat_2J(3,0)=mMat2D(1,0);
  Bmat_2J(3,1)=mMat2D(1,1);
  x<<currentJp[1],currentJv[1],currentJp[3],currentJv[3];
  ctorque<<controlSignal[1],controlSignal[3];
  e<<-pE[1],-vE[1],-pE[3],-vE[3];
  fuz.computeFuzModel(x,ctorque);
//  model_B=fuz.model_B;
  fuz.makeDelJDelx_Inte_Lyp(e,x);
  u0=u0_mat*fuz.getRule_W();

  data(0)=u0(0);
  data(1)=u0(1);
  ctorque=-0.5*Rinv* Bmat_2J.transpose()*fuz.delVdelX;

//  ctorque(0)=ctorque(0) - Sam::signOf<double>(x(0))*2.7;//u0(0); //3.7
//  ctorque(1)=ctorque(1) - Sam::signOf<double>(x(2))*0.9;//u0(1); //0.75
  //=================================================================

  intError = MT::add(intError, MT::mult(ki, MT::mult(T_s, error_1_jp)));
  if (intErrorLimit != MT::zero()) {
    intError = math::saturate(intError, intErrorLimit);
  }


    controlSignal = MT::add(MT::mult(kp, pE),
                            MT::add(intError,
                                MT::mult(kd, vE)));



    prevJp=currentJp;
    error_1_jp = pE;

//===============================================================================

    kd_comp<<2*omega1, omega2;
    kp_comp<<omega1*omega1,15*omega2*omega2;
      ePos << pE[1], pE[3];
  //  ePos << -currentJp[1], -currentJp[3];
      eVel << vE[1], vE[3];
  //  eVel= -qDot;
  //  qDesDD<<refacc[1], refacc[3];

      ///// Computed torque
//      ctorque   = mMat2Dinp.getValue()*(kd_comp.cwise()*eVel + kp_comp.cwise()*ePos);


//      ctorque<<u(0),u(1);

//
//      if(rCount>100){
//      controlSignal[1]=ctorque(0);
//      controlSignal[3]=ctorque(1);
//      }


	if (controlSignalLimit != MT::zero()) {
			controlSignal = math::saturate(controlSignal, controlSignalLimit);
		}
	this->controlOutputValue->setData(&controlSignal);
	criticconOpValue->setData(&ctorque);
	dataVecOutVal->setData(&data);

	rCount++;

  /////////////////////////////////////////////////////////////////////////


}


  
  template<int NJOINTS, int NR, typename InputType1, typename InputType2, typename OutputType, typename MathTraits>
  void myController<NJOINTS, NR, InputType1, InputType2, OutputType, MathTraits>::getSamplePeriodFromEM()
{
	if (this->hasExecutionManager()) {
		assert(this->getExecutionManager()->getPeriod() > 0.0);
		setSamplePeriod(this->getExecutionManager()->getPeriod());
	} else {
		setSamplePeriod(0.0);
	}
}
}
}
