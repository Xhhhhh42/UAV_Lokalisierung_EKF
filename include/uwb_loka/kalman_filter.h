#ifndef UWB_LOKA_KALMAN_FILTER_H
#define UWB_LOKA_KALMAN_FILTER_H

#include "estimator.h"
// #include "predictor.h"
#include "updator.h"
#include "imu_predictor.h"

namespace uwb_loka 
{

class KF : public StateEstimator, public IMU_Predictor, public Updator 
{
public:
    KF() {}

    KF( double acc_n, double gyr_n, double acc_w, double gyr_w ) 
        : IMU_Predictor( state_ptr_, acc_n, gyr_n, acc_w, gyr_w ) {}
    
    virtual ~KF() {}

    virtual void predict( ImuData_ConstPtr last_imu, ImuData_ConstPtr curr_imu ) = 0;

    // virtual void update() = 0;

private:
    Eigen::MatrixXd measurement_cov_;
    Eigen::MatrixXd measurement_noise_cov_;
};

using KF_Ptr = std::unique_ptr<KF>;

}  // namespace uwb_loka

#endif // UWB_LOKA_KALMAN_FILTER_H