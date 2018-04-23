#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
    // Vector for rmse.
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // Check for divide by zero error.
    if (estimations.size() == 0) {
        cout << "CalculateRSME () - Error - Zero length data" << endl;
        return rmse;
    }
    if (estimations.size() != ground_truth.size()) {
        cout << "CalculateRSME () - Error - Data lengths not equal" << endl;
        return rmse;
    }

    // Calculate squared residuals.
    for (int i=0; i < estimations.size(); i++) {

      VectorXd residual = estimations[i] - ground_truth[i];
      residual = residual.array()*residual.array();
      rmse += residual;

    }

    // Calculate the sample mean.
    rmse = rmse/estimations.size();

    // Calculate the squared root.
    rmse = rmse.array().sqrt();

    // Return the result.
    return rmse;

}