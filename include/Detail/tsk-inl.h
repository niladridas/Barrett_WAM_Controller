/*
 * tsk.cpp
 *
 *  Created on: 22-Apr-2013
 *      Author: samrat
 */

//#include <constants.h>

namespace Sam {

template <int xDim, int uDim, int nR>
tsk <xDim, uDim, nR>::tsk(const char* consModel, const char* consCost, const char* fuzParam, const std::string tskName):
W(Eigen::Matrix<double, nR, 1>()), R(Eigen::Matrix<double, nR, 1>()), mLocal(Eigen::Matrix<double, xDim/2, xDim/2>()), a(Eigen::Matrix<double, xDim/2, xDim/2>()),
cLocal(Eigen::Matrix<double, xDim/2, xDim/2>()), b(Eigen::Matrix<double, xDim/2, xDim/2>()),
CSdim(0), xOrth(Eigen::Matrix<double, 1, xDim*(xDim+1)/2>()), dXorth(Eigen::Matrix<double, 1, xDim*(xDim+1)/2>()),
Pvec(Eigen::Matrix<double, nR*(xDim*(xDim+1)/2), 1>()), delWx(Eigen::Matrix<double, xDim, nR*(xDim*(xDim+1)/2)>()), ruleBaseFlag(false),
delVdelX(Eigen::Matrix<double, xDim,1>()), model_A(Eigen::Matrix<double, xDim, xDim>()), model_P(Eigen::Matrix<double, xDim, xDim>()), model_B(Eigen::Matrix<double, xDim, uDim>()),
M(Eigen::Matrix<double, xDim/2, xDim/2>()), C(Eigen::Matrix<double, xDim/2, xDim/2>())
{

	//Allocate memory to the pointer
	A= new Eigen::Matrix<double, xDim, xDim>[nR];
	B= new Eigen::Matrix<double, xDim, uDim>[nR];
	Pmat = new Eigen::Matrix<double, xDim, xDim>[nR];
	std::string fnameConsModel(consModel);
	std::string fnameConsCost(consCost);
	std::string fnameFuz(fuzParam);
	std::vector<std::vector<double> > consDataModel, consDataCost, fuzData; //store data from file
	size_t trConsModel, trConsCost, tcConsModel, tcConsCost, trFuz,tcFuz;
	//Read the files

	fuzData=Sam::readFile<double>(fnameFuz);
	trFuz = fuzData.size();
	tcFuz = fuzData[0].size();

			std::cout<<trFuz<<"  "<<tcFuz<<std::endl;

		//Check Fuzzy params ile
		//===========================
		if(!(trFuz==nR && (tcFuz==2*xDim || tcFuz==2*(xDim+uDim))) ){

			std::cout<<"File "<<fuzParam<<" is not proper for Fyzzy params"<<std::endl;
			std::cout<<(trFuz!=nR)<< (tcFuz!=2*xDim) << (tcFuz!=2*(xDim+uDim))<< (trFuz!=nR || tcFuz!=2*xDim || tcFuz!=2*(xDim+uDim))<<std::endl;
			exit(EXIT_FAILURE);
		}

		std::ifstream fcons, ffp;

		double tmp;
		size_t elmt1=0;
		size_t elmt2=0;

		//Check if total number of elements are ok
		open2read(ffp, fuzParam);
		while(!ffp.eof()){
			ffp>>tmp;
			elmt2++;
		}
		if(!((elmt2-1)==2*(tcFuz/2)*nR))
		{
			std::cout<<"File "<<fuzParam<<" is not proper for Fyzzy params"<<std::endl;
			exit(EXIT_FAILURE);
		}

		ffp.close();

		// Resize Centers, sigmas and mu according to Parameter FIle
		CSdim=(tcFuz)/2;


		centers = new double* [nR];
		sigmas = new double* [nR];
		mu = new double* [nR];

		for(int r=0; r<nR; r++){
			centers[r] = new double [CSdim];
			sigmas[r]=new double [CSdim];
			mu[r]=new double [CSdim];
		}


		// collect centers and sigmas
		for(size_t r=0; r<nR; r++){
				for(size_t i1=0; i1<CSdim; i1++){

					centers[r][i1]=fuzData[r][i1];
				}
				for(size_t i2=0; i2<CSdim; i2++){

					sigmas[r][i2]=fuzData[r][CSdim+i2];

				}
		}


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Build Dynamic Model of the system


		std::cout<<"---->  TSK object created for "<<tskName<<" of the system  <----"<<std::endl;
		consDataModel=Sam::readFile<double>(fnameConsModel);
		trConsModel  = consDataModel.size();
		tcConsModel = consDataModel[0].size();
		model_A.fill(0);
		model_B.fill(0);


		//Check the files
		if(trConsModel!=nR*xDim || tcConsModel!=xDim+uDim){
			std::cout<<"File "<<consModel<<" is not proper for consequent params"<<std::endl;
			exit(EXIT_FAILURE);
		}

		// Check total no of elements

		open2read(fcons,consModel);

		while(!fcons.eof())
		{
			fcons>>tmp;
			elmt1++;
		}

		if(!((elmt1-1)==(xDim+uDim)*xDim*nR))
		{

			std::cout<<"File "<<consModel<<" is not proper for consequent params"<<std::endl;
			exit(EXIT_FAILURE);
		}


		fcons.close();

		//File checking is complete
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Collect A and B
		size_t rcount=0;
		for(size_t r=0; r<trConsModel; r++){
			for(size_t i=0; i<xDim; i++){
				A[rcount]((r%xDim),i)=consDataModel[r][i];
			}

			for(size_t j=0; j<uDim; j++){
				B[rcount]((r%xDim),j)=consDataModel[r][xDim+j];
			}
			if(((r+1)%xDim)==0){
				rcount=rcount+1;
			}

		}


	//	std::cout<<centers<<std::endl<<"==============================================="<<std::endl<<sigmas<<std::endl<<"==============================================="<<std::endl;



		consDataCost=Sam::readFile<double>(fnameConsCost);
		trConsCost = consDataCost.size();
		tcConsCost=consDataCost[0].size();
		std::cout<<"File Size: "<<trConsCost<<" , "<<tcConsCost<<std::endl;

		open2read(fcons,consCost);
		elmt1 = 0;
		while(!fcons.eof())
		{
			fcons>>tmp;
			elmt1++;
		}



		if(!(((elmt1-1)==(xDim+1)*xDim*nR/2) || (elmt1-1)==nR*xDim*xDim))
		{

			std::cout<<"File "<<consCost<<" is not proper for consequent params"<<std::endl;
			exit(EXIT_FAILURE);
		}


		fcons.close();





			std::cout<<"::::Conventional Lyapunov Function::::"<<std::endl;
			for(size_t i=0; i<trConsCost; i++){
				Pvec(i,0)=consDataCost[i][0];
			}

			size_t pvcnt =0;
			for(size_t r=0; r<nR; r++){
				for(size_t i=0; i<xDim; i++){
					for(size_t j=i; j<xDim; j++){
						Pmat[r](i,j)=Pvec(pvcnt,0);
						Pmat[r](j,i)=Pmat[r](i,j);
						pvcnt++;
					}
				}

//			std::cout<<Pmat[r]<<std::endl;
			}







}





template<int xDim, int uDim, int nR>
tsk<xDim, uDim, nR>::~tsk() {

	for(int r=0; r<nR; r++){
		delete [] centers[r];
		delete [] sigmas[r];
		delete [] mu[r];
	}
	delete [] centers;
	delete [] sigmas;
	delete [] mu;
	delete [] A;
	delete [] B;
	delete [] Pmat;

	// TODO Auto-generated destructor stub
}
//==========================================================================================================
template <int xDim, int uDim, int nR>
void tsk <xDim, uDim, nR>::calculateMu(const Eigen::Matrix<double, xDim, 1>& x, const Eigen::Matrix<double, uDim, 1>& u){
	for(size_t r=0; r<nR; r++){
		for(size_t i=0; i<xDim; i++){
			mu[r][i]=exp(-0.5*  pow((x(i,0)-centers[r][i]),2)  /   (sigmas[r][i]*sigmas[r][i])  );
		}
		if(CSdim==xDim+uDim){
			for(size_t i=0; i<uDim; i++){
						mu[r][xDim+i]=exp(-0.5*  pow((u(i,0)-centers[r][xDim+i]),2)  /   (sigmas[r][xDim+i]*sigmas[r][xDim+i])  );
			}
		}
	}
//std::cout<<"mu: "<<mu<<std::endl;
}

//================================================================================================================
template <int xDim, int uDim, int nR>
void tsk <xDim, uDim, nR>::calculateMu(const Eigen::Matrix<double, xDim, 1>& x){
	for(size_t r=0; r<nR; r++){
		for(size_t i=0; i<xDim; i++){
			mu[r][i]=exp(-0.5*  pow((x(i,0)-centers[r][i]),2)  /   (sigmas[r][i]*sigmas[r][i])  );
		}

	}
//std::cout<<"mu: "<<mu<<std::endl;
}

//=======================================================================================================
template <int xDim, int uDim, int nR>
void tsk <xDim, uDim, nR>::calculateW(){
	R.fill(1.0);
	for(size_t r=0; r<nR; r++){
			for(size_t i=0; i<CSdim; i++){
				R(r,0)=R(r,0)*mu[r][i];
			}
		}
//	std::cout<<"W: "<<W<<std::endl;
	W=R/R.sum();
//	std::cout<<"W: "<<W<<std::endl;
//	std::cout<<"W: "<<W<<std::endl<<"==========================="<<std::endl;
}
//==============================================================================================
template <int xDim, int uDim, int nR>
void tsk <xDim, uDim, nR>::computeFuzModel(const Eigen::Matrix<double, xDim, 1>& x, const Eigen::Matrix<double, uDim, 1>& u){
	model_A.fill(0);
	model_B.fill(0);
	M.fill(0.0);
	C.fill(0.0);
	calculateMu(x,u);
	calculateW();

//	std::cout<<"W: "<<W<<std::endl;

	for(int r=0; r<nR; r++){

		model_A=model_A+ W(r,0)*A[r];
		model_B=model_B+ W(r,0)*B[r];
	}


#ifdef LOCAL_MASSMAT

	for(int r=0; r<nR; r++){

		a(0,0) = A[r](1,1);
		a(0,1) = A[r](1,3);
		a(1,0) = A[r](3,1);
		a(1,1) = A[r](3,3);

		b(0,0)=B[r](1,0);
		b(0,1)=B[r](1,1);
		b(1,0)=B[r](3,0);
		b(1,1)=B[r](3,1);

		mLocal = b.inverse();
		cLocal = mLocal*a;

		model_A.fill(0);
//std::cout<<mLocal<<std::endl;
		M=M + W(r,0)*mLocal;
		C=C + W(r,0)*cLocal;
	}
//std::cout<<M<<std::endl;
#endif




	ruleBaseFlag = true;
}

//==================================================================================================
template <int xDim, int uDim, int nR>
void tsk <xDim, uDim, nR>::calcXdot(const Eigen::Matrix<double, xDim, 1>& x, const Eigen::Matrix<double, uDim, 1>& u, Eigen::Matrix<double, xDim, 1 >& xdot){

	xdot= model_A*x + model_B*u;

}



//===================================================================================================
template <int xDim, int uDim, int nR>
void tsk <xDim, uDim, nR>::makeDelJDelx(const Eigen::Matrix<double, xDim, 1>& e, const Eigen::Matrix<double, xDim, 1>& x){
//	delWx.resize(xDim,810);
	model_P.fill(0);

	if(ruleBaseFlag == false){
		calculateMu(x);
		calculateW();

	}

	size_t i=0;
	//Xorth============

	for(size_t j=0; j<xDim; j++){
		for(size_t k=j; k<xDim;k++){
			if(j==k){
				xOrth(0,i)=e(j)*e(k);
				i++;
			}
			else{
				xOrth(0,i)=2*e(j)*e(k);
				i++;
			}
		}
	}
	/// delWx calculation

	for(size_t i=0; i<xDim; i++){
		dXorth.fill(0.0);

		double tmp2(0.0);
		for(size_t r=0; r<nR; r++){
			//		  tmp2=tmp2+ m_normMemVal[r]*(x[i]-m_mC[r][i])/(m_vden[i]*m_vden[i]);
			tmp2=tmp2+ W(r)*(x(i)-centers[r][i])/(sigmas[r][i]*sigmas[r][i]);
		}
		size_t cnt=0;
		for(size_t m=0; m<xDim; m++){
			for(size_t n=m; n<xDim; n++){
				if (i==m)
					dXorth(0,cnt)= 2*e(n);
				if (i==n)
					dXorth(0,cnt)= 2*e(m);
				cnt++;
			}
		}
		for(size_t j=0; j<nR; j++){
			double tmp1(0.0);

			// delXorth/delx_i

			tmp1= (x[i]-centers[j][i])/(sigmas[j][i]*sigmas[j][i]);
	 	    delWx.template block<1,(xDim*(xDim+1)/2)>(i,j*(xDim*(xDim+1)/2))=0.5*W(j)*( dXorth - tmp1*xOrth + xOrth*tmp2);
		}
	}
	delVdelX = delWx*Pvec;


	for(size_t r=0; r<nR; r++){
		model_P=model_P + W(r)*Pmat[r];
	}


	ruleBaseFlag = false;

}


template <int xDim, int uDim, int nR>
void tsk <xDim, uDim, nR>::makeDelJDelx_Inte_Lyp(const Eigen::Matrix<double, xDim, 1>& e, const Eigen::Matrix<double, xDim, 1>& x){
//	delWx.resize(xDim,810);
	model_P.fill(0);

	if(ruleBaseFlag == false){
		calculateMu(x);
		calculateW();

	}


	for(size_t r=0; r<nR; r++){
			//		  tmp2=tmp2+ m_normMemVal[r]*(x[i]-m_mC[r][i])/(m_vden[i]*m_vden[i]);
			model_P=model_P + W(r)*Pmat[r];
		}

	delVdelX =2* model_P*e;

	ruleBaseFlag = false;

}

template <int xDim, int uDim, int nR>
double tsk <xDim, uDim, nR>::output(const Eigen::Matrix<double, xDim, 1>& x){
    model_P.fill(0);

    if(ruleBaseFlag == false){
      calculateMu(x);
      calculateW();

    }


    for(size_t r=0; r<nR; r++){
      //        tmp2=tmp2+ m_normMemVal[r]*(x[i]-m_mC[r][i])/(m_vden[i]*m_vden[i]);
      model_P=model_P + W(r)*Pmat[r];
    }

    ruleBaseFlag = false;


    return (x.transpose()*model_P*x)(0,0);
}



} /* namespace sam */
