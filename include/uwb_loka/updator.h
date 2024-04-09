#ifndef UWB_LOKA_UPDATOR_H
#define UWB_LOKA_UPDATOR_H

#include "observer.h"

namespace uwb_loka 
{

class Updator 
{

public:
    Updator() {}

    Updator(const Updator &) = delete;

public:
    ObserverPtr observer_ptr_;

};

}  // namespace uwb_loka

#endif // UWB_LOKA_UPDATOR_H
