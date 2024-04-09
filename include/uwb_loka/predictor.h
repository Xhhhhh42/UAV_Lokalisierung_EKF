#ifndef UWB_LOKA_PREDICTOR_H
#define UWB_LOKA_PREDICTOR_H

#include <sensor_msgs/Imu.h>

#include "imu.h"

namespace uwb_loka 
{

class Predictor 
{
public:
    Predictor() {}

    Predictor( const Predictor & ) = delete;

    Predictor( State_Ptr &state_ptr, double acc_n, double gyro_n, double acc_w, double gyro_w );

    virtual ~Predictor() {}

    bool init( double ts_meas );

    void imu_callback( const sensor_msgs::ImuConstPtr &imu_msg );

    virtual void predict( ImuData_ConstPtr last_imu, ImuData_ConstPtr curr_imu ) = 0;

public:
    bool inited_ = false;

    IMU imu_model_;
    ImuData_ConstPtr last_imu_ptr_;

private:
    State_Ptr state_p_;
};

using Predictor_Ptr = std::unique_ptr<Predictor>;

}  // namespace uwb_loka

#include "predictor_impl.hpp"

#endif // UWB_LOKA_PREDICTOR_H