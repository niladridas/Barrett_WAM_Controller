
#include <iostream>
namespace barrett{
namespace systems{


template<size_t DOF, int NSTATES, int NR>
inputMatrix<DOF, NSTATES, NR>::inputMatrix(const std::string& sysName):
System(sysName), jPositions(this),jVelocities(this), jTorques(this), gx(this, &gxVal), gxrow(this, &gxrowVal),
fuz("./FuzzParams/cons_5nrX_20sat_182e_10kd.txt", "./FuzzParams/gaussParams_5nrX_20sat_182e_10kd.txt"),
u(Eigen::Matrix<double, NSTATES/2, 1>()),x(Eigen::Matrix<double, NSTATES, 1>()), jp(0.0), jv(0.0), jt(0.0),
gx_mat(inputMatrix_type()), gx_matrow(0.0)
{

}



template<size_t DOF, int NSTATES, int NR>
void inputMatrix<DOF, NSTATES, NR>::operate(){
	jp=jPositions.getValue();
	jv=jVelocities.getValue();
	jt=jTorques.getValue();
	x<<jp[1],jv[1],jp[3],jv[3];
	u<<jt[1],jt[3];
	fuz.computeFuzModel(x,u);
	gx_mat=fuz.model_B;
	gxVal->setData(&gx_mat);
	gx_matrow(0,0)=gx_mat(1,0);
	gx_matrow(1,0)=gx_mat(1,1);
	gx_matrow(2,0)=gx_mat(3,0);
	gx_matrow(3,0)=gx_mat(3,1);
	gxrowVal->setData(&gx_matrow);
}
}
}
