/*
 * testmain.cpp
 *
 *  Created on: 06-Dec-2014
 *      Author: mobman
 */



//#include <testclass.h>
#include<samlibs.h>
#include <GMM.h>
int main(){

//isl::GMM G(0.01, 0.01);
//Eigen::MatrixXd input_output_array;
//Eigen::MatrixXd input_array;
//size_t p=1000;
//Sam::initEigenMat<double>(input_output_array,Sam::readFile<double>("data/input_output.txt"));
//input_array.resize(p, 3);
//input_array = input_output_array.block(0, 0, p, 3);
//std::ofstream ff("out.txt");
//for (size_t i = 0; i < p; i++) {
//        Eigen::VectorXd input(3);
//        input(0) = input_array(i, 0);
//        input(1) = input_array(i, 1);
//        input(2) = input_array(i, 2);
//        G.calculate_model_output(input);
//        ff<<G.output.transpose()<<"\n";
//
//}
//ff.close();

// Running simulation with Ts = 8 ms and assuming that the robot is able to move
isl::GMM G(0.01, 0.01);
Eigen::VectorXd tmp_xi(3);
tmp_xi << 0.0976,0.1701,0.1457;
G.calculate_model_output(tmp_xi);
Eigen::VectorXd xi_star(3);
xi_star << 0.579302, 0.005575, -0.264192;
std::ofstream ff("out.txt");
int count = 0;
while ((tmp_xi -xi_star).norm() >= 0.10 && count <= 10000000)
{
    G.calculate_model_output(tmp_xi);
    tmp_xi = tmp_xi + 0.008*G.output;
    ff << tmp_xi.transpose() << " " << G.foutput.transpose() <<  " " << G.uoutput.transpose() << "\n";
    count++;
}
ff.close();
  return 0;
}
