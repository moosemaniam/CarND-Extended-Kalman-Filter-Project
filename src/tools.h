#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
  public:
    /**
     * Constructor.
     */
    Tools();

    /**
     * Destructor.
     */
    virtual ~Tools();

    /**
     * A helper method to calculate RMSE.
     */
    VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

    /**
     * A helper method to calculate Jacobians.
     */
    MatrixXd CalculateJacobian(const VectorXd& x_state);

    /*
     * Helper method to convert from cartesian to polar co-ordinates
     * */
    VectorXd cartesian_to_polar(const MeasurementPackage &measurement_pack);

    /*
     * Helper to normalize to pi
     * */
    float normalize_to_pi(float angle);
};

#endif /* TOOLS_H_ */
