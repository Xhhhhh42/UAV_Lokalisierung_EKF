#ifndef UWB_LOKA_ESTIMATOR_H
#define UWB_LOKA_ESTIMATOR_H

#include "state.h"

namespace uwb_loka 
{

class StateEstimator 
{

public:
    StateEstimator() { state_ptr_ = std::make_shared<State>(); }

public:
    State_Ptr state_ptr_;

};

}  // namespace uwb_loka

#endif // UWB_LOKA_ESTIMATOR_H
