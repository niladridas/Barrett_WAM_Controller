/*
 * test_optimal_sliding.cpp
 *
 *  Created on: 26-Feb-2015
 *      Author: nilxwam
 */
#include <eigen3/Eigen/Dense>
//#include<eigen3/Eigen/LU>
#include<M_4D.hpp>
#include<C_4D.hpp>
//#include<eigen3/Eigen/Core>
#include<fstream>
#include<iostream>

using namespace Eigen;
using namespace std;

int main() {
	Vector4d theta_init;
	theta_init << 0,0,0,+3.14;
	Vector4d theta_dot_init;
	theta_dot_init << 0,0,0,0;

	Vector4d theta_des;
	theta_des << 0,0,0,+3.14;
	Vector4d theta_dot_des;
	theta_dot_des << 0,0.6,0,0;
	Vector4d theta_dot_dot_des;
	theta_dot_dot_des << 0,0,0,0;


	Matrix4d M_init;
	Matrix4d M_des;

	Vector4d C_init;
	Vector4d C_des;

	Matrix4d M_init_inverse;
	Matrix4d M_des_inverse;

	Vector4d e;
	Vector4d ed;

	Vector4d eta;

	VectorXd Xtilde;

	Eigen::Vector4d F;
	Eigen::Vector4d Fd;

	Eigen::Vector4d rho;

	Eigen::Vector3d W1;
	Eigen::Vector3d W2;
	Eigen::Vector3d W3;
	Eigen::Vector3d W4;

	Eigen::Vector4d phi;

	Eigen::Vector4d c;

	Eigen::Vector4d K;

	int i;

	Eigen::Vector4d fbar;

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

	Eigen::Vector4d Ueq;
	Eigen::Vector4d Ud;
	Eigen::Vector4d Tau;
	double ueq1, ueq2, ueq3, ueq4;

	Eigen::Vector4d L;
	Eigen::Vector4d Lbar;

	M_init = M_4D(theta_init);
	std::cout << "The value of M_init is :\n "<< M_init <<std::endl;

	M_des = M_4D(theta_des);
	std::cout << "The value of M_des is :\n "<< M_des <<std::endl;

	C_init = C_4D(theta_init, theta_dot_init);
	std::cout << "The value of C_init is : "<< C_init <<std::endl;

	C_des = C_4D(theta_des, theta_dot_des);
	std::cout << "The value of C_des is : "<< C_des <<std::endl;

	M_init_inverse = M_init.inverse();
	std::cout << "The value of M_init_inverse is :\n "<< M_init_inverse <<std::endl;

	M_des_inverse = M_des.inverse();
	std::cout << "The value of M_des_inverse is : \n"<< M_des_inverse <<std::endl;

	//Position error
	e = theta_init - theta_des;
	//Velocity error
	ed = theta_dot_init - theta_dot_des;

	ifstream myReadFile1;
	myReadFile1.open("data/W_optimal_sliding.txt");
	double W[12];
	for (int i = 0; i < 12; i++) {
		myReadFile1 >> W[i];
	}
	myReadFile1.close();

	W1 << W[0], W[1], W[2];
	W2 << W[3], W[4], W[5];
	W3 << W[6], W[7], W[8];
	W4 << W[9], W[10], W[11];

	ifstream myReadFile2;
	myReadFile2.open("data/Phi_optimal_sliding.txt");
	double Phi[4];
	for (int i = 0; i < 4; i++) {
		myReadFile2 >> Phi[i];
	}
	myReadFile2.close();
	//
	phi << Phi[0], Phi[1], Phi[2], Phi[3];

	ifstream myReadFile3;
	myReadFile3.open("data/c_optimal_sliding.txt");
	double C[4];
	for (int i = 0; i < 4; i++) {
		myReadFile3 >> C[i];
	}
	myReadFile3.close();

	//
	c << C[0], C[1], C[2], C[3];

	ifstream myReadFile4;
	myReadFile4.open("data/K_optimal_sliding.txt");
	double K_optimal[4];
	for (int i = 0; i < 4; i++) {
		myReadFile4 >> K_optimal[i];
	}
	myReadFile4.close();
	K << K_optimal[0], K_optimal[1], K_optimal[2], K_optimal[3];

	ifstream myReadFile5;
	myReadFile5.open("data/b_del.txt");
	double b_del[2];
	for (int i = 0; i < 2; i++) {
		myReadFile5 >> b_del[i];
	}
	myReadFile5.close();
	double b;
	b = b_del[0];


	eta = K / b;
	//The error matrix
	Xtilde.resize(8, 1);
	Xtilde << e, ed;

	F = -M_init_inverse * (C_init);
	Fd = -M_des_inverse * (C_des);

	std::cout << "The value of F is : "<< F <<std::endl;
	std::cout << "The value of Fd is : "<< Fd <<std::endl;

	rho = F - Fd;


	std::cout << "The value of W1 is : "<< W1 <<std::endl;
	std::cout << "The value of W2 is : "<< W2 <<std::endl;
	std::cout << "The value of W3 is : "<< W3 <<std::endl;
	std::cout << "The value of W4 is : "<< W4 <<std::endl;
	std::cout << "The value of Xtilde is : "<< Xtilde <<std::endl;
	std::cout << "The value of phi is : "<< phi <<std::endl;


	////
	ueq1 = (1 / 2)
			* (W1[0] * (W1[2] - W1[1]) * Xtilde[0]
					+ W1[1] * (W1[2] - W1[1]) * Xtilde[4]
					+ W1[2] * (W1[2] - W1[1]) * phi[0]);
	std::cout << "The value of ueq1 is : "<< ueq1 <<std::endl;

	ueq2 = (1/2)
			* (W2[0] * (W2[2] - W2[1]) * Xtilde[1]
					+ W2[1] * (W2[2] - W2[1]) * Xtilde[5]
					+ W2[2] * (W2[2] - W2[1]) * phi[1]);
	std::cout << "The value of ueq2 is : "<< ueq2 <<std::endl;

	ueq3 = (1 / 2)
			* (W3[0] * (W3[2] - W3[1]) * Xtilde[2]
					+ W3[1] * (W3[2] - W3[1]) * Xtilde[6]
					+ W3[2] * (W3[2] - W3[1]) * phi[2]);
	std::cout << "The value of ueq3 is : "<< ueq3 <<std::endl;

	ueq4 = (1 / 2)
			* (W4[0] * (W4[2] - W4[1]) * Xtilde[3]
					+ W4[1] * (W4[2] - W4[1]) * Xtilde[7]
					+ W4[2] * (W4[2] - W4[1]) * phi[3]);
	std::cout << "The value of ueq4 is : "<< ueq4 <<std::endl;

	//
	// Final torque computations
	Ueq << ueq1, ueq2, ueq3, ueq4;
	std::cout << "The value of Ueq is : "<< Ueq <<std::endl;


	Ud = theta_dot_dot_des + M_des_inverse * C_des;

	Tau = M_init * (Ud + Ueq); // Final torque to system.
	std::cout << "The value of Tau is : "<< Tau <<std::endl;


	return 0;
}

