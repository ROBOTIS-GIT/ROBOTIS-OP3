
#include <ros/ros.h>

#include "robotis_math/robotis_math.h"
#include "scilab_optimization/scilab_optimization.h"

#include "op3_wholebody_module_msgs/PreviewRequest.h"
#include "op3_wholebody_module_msgs/PreviewResponse.h"
#include "op3_wholebody_module_msgs/GetPreviewMatrix.h"

double P_row_, P_col_;
std::vector<double_t> P_;
double K_row_, K_col_;
std::vector<double_t> K_;

bool calcPreviewParam(double control_cycle, double lipm_height)
{
  double preview_time_ = 1.6;
  int preview_size_;

  Eigen::MatrixXd A_, b_, c_;

  double t = control_cycle;

  preview_size_ = round(preview_time_/t);

  A_.resize(3,3); b_.resize(3,1); c_.resize(1,3);
  A_ << 1,  t,  t*t/2.0,
        0,  1,  t,
        0,  0,  1;
  b_(0,0) = t*t*t/6.0;
  b_(1,0) =   t*t/2.0;
  b_(2,0) =     t;

  c_(0,0) = 1; c_(0,1) = 0; c_(0,2) = -lipm_height/9.81;

  Eigen::MatrixXd tempA = Eigen::MatrixXd::Zero(4,4);
  Eigen::MatrixXd tempb = Eigen::MatrixXd::Zero(4,1);
  Eigen::MatrixXd tempc = Eigen::MatrixXd::Zero(1,4);

  tempA.coeffRef(0,0) = 1;
  tempA.block<1,3>(0,1) = c_*A_;
  tempA.block<3,3>(1,1) = A_;

  tempb.coeffRef(0,0) = (c_*b_).coeff(0,0);
  tempb.block<3,1>(1,0) = b_;

  tempc.coeffRef(0,0) = 1;

  double Q_e = 1, R = 1e-6;//1.0e-6;
  double Q_x = 0;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4,4);

  Q.coeffRef(0,0) = Q_e;
  Q.coeffRef(1,1) = Q_e;
  Q.coeffRef(2,2) = Q_e;
  Q.coeffRef(3,3) = Q_x;

  double matrix_A[] = {tempA.coeff(0,0), tempA.coeff(1,0), tempA.coeff(2,0), tempA.coeff(3,0),
                       tempA.coeff(0,1), tempA.coeff(1,1), tempA.coeff(2,1), tempA.coeff(3,1),
                       tempA.coeff(0,2), tempA.coeff(1,2), tempA.coeff(2,2), tempA.coeff(3,2),
                       tempA.coeff(0,3), tempA.coeff(1,3), tempA.coeff(2,3), tempA.coeff(3,3)};
  int row_A = 4, col_A = 4;

  double matrix_B[] = {tempb.coeff(0,0), tempb.coeff(1,0), tempb.coeff(2,0), tempb.coeff(3,0)};
  int row_B = 4, col_B = 1;

  double matrix_Q[] = {Q.coeff(0,0), Q.coeff(1,0), Q.coeff(2,0), Q.coeff(3,0),
                       Q.coeff(0,1), Q.coeff(1,1), Q.coeff(2,1), Q.coeff(3,1),
                       Q.coeff(0,2), Q.coeff(1,2), Q.coeff(2,2), Q.coeff(3,2),
                       Q.coeff(0,3), Q.coeff(1,3), Q.coeff(2,3), Q.coeff(3,3)};
  int row_Q = 4, col_Q = 4;

  double matrix_R[] = {R};
  int row_R = 1, col_R = 1;

  double *matrix_K = (double*)malloc(100*sizeof(double));
  int row_K, col_K;

  double *matrix_P = (double*)malloc(100*sizeof(double));
  int row_P, col_P;

  double *matrix_E_real = (double*)malloc(100*sizeof(double));
  double *matrix_E_imag = (double*)malloc(100*sizeof(double));
  int row_E, col_E;

  robotis_framework::ScilabOptimization::initialize();
  robotis_framework::ScilabOptimization::solveRiccatiEquation(matrix_K, &row_K, &col_K,
                                                              matrix_P, &row_P, &col_P,
                                                              matrix_E_real, matrix_E_imag, &row_E, &col_E,
                                                              matrix_A, row_A, col_A,
                                                              matrix_B, row_B, col_B,
                                                              matrix_Q, row_Q, col_Q,
                                                              matrix_R, row_R, col_R);

  int K_size = row_K*col_K;
  for (int i=0; i<K_size; i++)
    K_.push_back(matrix_K[i]);

  K_row_ = row_K;
  K_col_ = col_K;

  int P_size = row_P*col_P;
  for (int i=0; i<P_size; i++)
    P_.push_back(matrix_P[i]);

  P_row_ = row_P;
  P_col_ = col_P;

  free(matrix_K);
  free(matrix_P);
  free(matrix_E_real);
  free(matrix_E_imag);

  robotis_framework::ScilabOptimization::terminate();

  return true;
}

bool getPreviewMatrixCallback(op3_wholebody_module_msgs::GetPreviewMatrix::Request& req,
                              op3_wholebody_module_msgs::GetPreviewMatrix::Response& res)
{
  double control_cycle = req.req.control_cycle;
  double lipm_height = req.req.lipm_height;

  bool cal_success = calcPreviewParam(control_cycle, lipm_height);

  if (cal_success == true)
  {
    res.res.K_row = K_row_;
    res.res.K_col = K_col_;

    int K_size = K_row_*K_col_;
    for (int i=0; i<K_size; i++)
      res.res.K.push_back(K_[i]);

    res.res.P_row = P_row_;
    res.res.P_col = P_col_;

    int P_size = P_row_*P_col_;
    for (int i=0; i<P_size; i++)
      res.res.P.push_back(P_[i]);

    return true;
  }
  else
    return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "op3_optimization");
  ros::NodeHandle nh("~");

  ros::ServiceServer get_preview_matrix_server = nh.advertiseService("/robotis/get_preview_matrix", getPreviewMatrixCallback);

  ros::spin();

  return 0;
}
