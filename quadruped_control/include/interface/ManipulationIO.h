#ifndef ManipulationIO_H
#define ManipulationIO_H

#include <messages/LowlevelState.h>

class ManipulationIO
{
    public:
        ManipulationIO(){}
        ~ManipulationIO(){}
        virtual void run(LowlevelState *state) = 0;
};

#endif  // ManipulationIO_H