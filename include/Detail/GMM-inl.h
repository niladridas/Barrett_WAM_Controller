/*
 * GMM.cpp
 *
 *  Created on: 13-Dec-2014
 *      Author: mobman
 */

//#include <nilu/GMM.h>

namespace isl {

  GMM::GMM(double Rho_0, double K): priors (Eigen::VectorXd()), mean(Eigen::MatrixXd()), sigma(Eigen::MatrixXd()),xistar_trn(Eigen::VectorXd()), P_0_data(Eigen::MatrixXd()),P_total_data(Eigen::MatrixXd()),
      Mu(Eigen::VectorXd()),f_hat(Eigen::VectorXd()),grad_val(Eigen::VectorXd()),output(Eigen::VectorXd()),
#ifdef DEBUG_GMM_
      uOutput(Eigen::Matrix<double, 3,1>()), fOutput(Eigen::Matrix<double, 3,1>()),
#endif
      num_inp(0), num_op(0),num_priors(0),      num_lyp_asym_copont(0),
      RHO_0(Rho_0), K_(K){

    pi = boost::math::constants::pi<double>();

    Sam::initEigenVec<double>(priors,Sam::readFile<double>("data/priors.txt"));
    Sam::initEigenMat<double>(mean, Sam::readFile<double>("data/mean.txt"));
    Sam::initEigenMat<double>(sigma, Sam::readFile<double>("data/sigma.txt"));
    Sam::initEigenMat<double>(P_0_data, Sam::readFile<double>("data/P0.txt"));
    Sam::initEigenMat<double>(P_total_data, Sam::readFile<double>("data/P_total.txt"));
    Sam::initEigenVec<double>(Mu,Sam::readFile<double>("data/Mu_total.txt"));
    num_priors = priors.size();
    std::map<std::string, std::string> allData_inp;
    if(Sam::readConfigFile("data/sysconfig.txt", allData_inp)){
      if(!Sam::findValueFromMap<size_t>(allData_inp, "num_input", num_inp)){
        exit(EXIT_FAILURE);
      }
      if(!Sam::findValueFromMap<size_t>(allData_inp, "num_output", num_op)){
        exit(EXIT_FAILURE);
      }
      if(!Sam::findValueFromMap<size_t>(allData_inp, "num_lyp_asym_copont", num_lyp_asym_copont)){
        exit(EXIT_FAILURE);
      }
      std::vector<double> xi;
      if(!Sam::findValueFromMap<double>(allData_inp, "xi_star_training", xi)){
        exit(EXIT_FAILURE);
      }
      Sam::initEigenVec<double>(xistar_trn,xi);
    } else exit(EXIT_FAILURE);

    f_hat.resize(num_op);
    output.resize(num_op);
    grad_val.resize(num_inp);
  }

  GMM::~GMM() {
    // TODO Auto-generated destructor stub
  }


  double GMM::gausspdf(const Eigen::VectorXd& input, const Eigen::VectorXd& Mean,
          const Eigen::MatrixXd& Sigma) // input arguments are the query point, mean and the sigma
          {
      double det_tmp_tmp = pow(double(fabs(Sigma.determinant())), 0.5);
      double coeff = pow((2 * pi), (input.rows())/ 2)* det_tmp_tmp;

  //  long double


      double exponential_part = (-0.5 * (((input - Mean).transpose()) * (Sigma.inverse())) * (input - Mean)) (0,0);

      double result = (1 / coeff) * exp(exponential_part);

  //    std::cout << result << std::endl;

      return result;
  }



  void GMM::f_estimate(const Eigen::VectorXd& input)
  {
  // First calculate dimension of priors
  int num_priors = priors.rows(); // priors comes as a rows
  int num_inp = input.rows();


  Eigen::MatrixXd input_mean_matrix(num_inp, num_priors); //   input  position(xi) mean vector for 6 gaussians
  Eigen::MatrixXd out_mean_matrix(num_inp, num_priors); //
  Eigen::MatrixXd sigma_input(num_inp*num_priors, num_inp); //  extracted  sigma for input
  Eigen::MatrixXd sigma_output_input(num_inp*num_priors, num_inp); //   extracted sigma sigmadot for input
  Eigen::MatrixXd Inverse_sigma_input(num_inp*num_priors, num_inp); // inverse of the sigma matrix taking 6, 2x2 matrix
  Eigen::VectorXd p(num_priors); //probability of input for 6 components
  Eigen::VectorXd h(num_priors); // h value in the linear regression equation
  Eigen::MatrixXd A_matrix(num_inp*num_priors, num_inp); // 'A' matrix in the linear regression equation
  Eigen::MatrixXd B_matrix(num_inp, num_priors); // 'B' matrix in the linear regression equation
  Eigen::VectorXd output(num_inp); // output 'xi dot' of regression equation

  int i =0;
  int k= 0;
  //  float f = 0; // denominator value of h(k)


  input_mean_matrix = mean.block(0, 0, num_inp, num_priors);
  out_mean_matrix   = mean.block(num_inp, 0, num_inp, num_priors);

  //

  for (i = 0; i < num_priors; i++) {
      sigma_input.block(num_inp * i, 0, num_inp, num_inp) = sigma.block(2*num_inp* i, 0, num_inp, num_inp);
  }


  //
  for (i = 0; i < num_priors; i++) {
      sigma_output_input.block(num_inp * i, 0, num_inp, num_inp) = sigma.block(2*num_inp* i + num_inp, 0, num_inp, num_inp);
  }

  //

  for (i = 0; i < num_inp*num_priors - num_inp ; i = i + num_inp) //inverse
          {
      Inverse_sigma_input.block(i, 0, num_inp, num_inp) = sigma_input.block(i, 0, num_inp, num_inp).inverse();
  }

  //


  long double prob_input = 0;

  //float prob_input = input_probability(input,mean1,sigma1);
  for (k = 0; k < num_priors; k++) {
       double prob_input_tmp = gausspdf(input, input_mean_matrix.col(k),
              sigma_input.block(num_inp * k, 0, num_inp, num_inp));
       double tmp_Pk = priors(k) * prob_input_tmp;
      p(k) = tmp_Pk;
      prob_input = prob_input + tmp_Pk;

  }



  for (k = 0; k < num_priors; k++) {
       double h_k = p(k) / prob_input;
      h(k) = h_k;
  }

  for (k = 0; k < num_inp*num_priors - num_inp; k = k + num_inp) {
      A_matrix.block(k, 0, num_inp, num_inp) = sigma_output_input.block(k, 0, num_inp, num_inp) * Inverse_sigma_input.block(k, 0, num_inp, num_inp);
  }

  for (k = 0; k < num_priors; k++) {
      B_matrix.col(k) = out_mean_matrix.col(k) - A_matrix.block(num_inp * k, 0, num_inp, num_inp) * input_mean_matrix.col(k);
  }


  f_hat.fill(0.0);

  for (k = 0; k < num_priors; k++) {
      f_hat = f_hat + h(k) * (A_matrix.block(num_inp * k, 0, num_inp, num_inp) * input + B_matrix.col(k));
  }

  }


 void GMM::calculate_gradient(const Eigen::VectorXd& input) {



      int input_size = input.rows();




      Eigen::VectorXd grad_part_2(input_size);

  //  cout << "hi haha " <<endl;
  //  cout << "hi haha " << num_lyp_asym_copont  << endl;


      grad_part_2 = Eigen::VectorXd::Zero(input_size);



      if (num_lyp_asym_copont == 0) {

          grad_val = -(P_0_data + P_0_data.transpose()) * (xistar_trn - input);

      }



      else {

  //      Eigen::VectorXi beta(lyapunovconfigdata(0));

          for (size_t i = 0; i < num_lyp_asym_copont; i++) {
              float tmp_argument = (-(xistar_trn.transpose() - input.transpose())
                      * P_total_data.block(i * input_size, 0, input_size,
                              input_size)
                      * (input-xistar_trn
                              - Mu.block(i * input_size, 0, input_size, 0))) (0,0);
              if (tmp_argument >= 0) {
                  grad_part_2 =
                          grad_part_2
                                  + 2 * tmp_argument
                                          * ((P_total_data.block(i * input_size,
                                                  0, input_size, input_size)
                                                  - P_total_data.block(
                                                          i * input_size, 0,
                                                          input_size, input_size).transpose())
                                                  * (xistar_trn - input)
                                                  - P_total_data.block(
                                                          i * input_size, 0,
                                                          input_size, input_size)
                                                          * Mu.block(
                                                                  i * input_size,
                                                                  0, input_size,
                                                                  0));
              }

          }

          grad_val = -(P_0_data + P_0_data.transpose()) * (xistar_trn - input)
                  + grad_part_2;
      }


  }

  Eigen::VectorXd GMM::control_input(const Eigen::VectorXd& input) {
    int input_size = input.rows();
    Eigen::VectorXd control_input(input_size);
    float tmp_value1 = ((grad_val.transpose() * f_hat) (0,0)+ (RHO_0 * (1 - exp(-K_ * input.norm()))));
    if (tmp_value1 <= 0 && input != xistar_trn) {
      control_input.Zero(input_size);
    }
    else if(input == xistar_trn)
    {
      control_input = -f_hat;
    }
    else
    {
      control_input = -grad_val*tmp_value1/(grad_val.transpose()*grad_val) (0,0);
    }
    return control_input;
  }


  void GMM::calculate_model_output(const Eigen::VectorXd& input){
    f_estimate(input);
    calculate_gradient(input);

    output = f_hat ;//+ control_input(input);
#ifdef DEBUG_GMM_
    for(size_t i=0; i<3; ++i){
      fOutput(i,0) = f_hat(i);
    }
    uOutput = control_input(input);
#endif
//    std::cout<<"f_hat: \n"<<f_hat<<"\n u: \n"<<control_input(input)<<"\n";
  }

} /* namespace isl */
