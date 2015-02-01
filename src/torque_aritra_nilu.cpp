#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include </usr/include/eigen3/Eigen/Core>
#include </usr/include/eigen3/Eigen/Dense>
#include <ctime>
#include <cstdio>
#include "C_Vector.hpp"
#include "Mass_matrix.hpp"
#include <sys/time.h>

int main(){//int argc, char **argv) {


	clock_t start;
	double diff;
	start = clock();
	Eigen::VectorXd Torque(7);
	Eigen::VectorXd x(14);
	x << 1,1,1,1,1,1,1,1,1,1,1,1,1,1;
	x = 0.1*x;
//	std::cout << x << std::endl;

	Torque = Mass_matrix(x)*x.block(0,0,7,1) + C_Vector(x);
	diff = (double) (std::clock() - start) / (double) CLOCKS_PER_SEC * 1000;
	cout << "printf: " << diff << '\n';
	cout << "Torque: " << x << '\n';
	return 0;
}
