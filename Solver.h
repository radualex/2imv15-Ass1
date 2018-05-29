#ifndef PARTICLETOY_SOLVER_H
#define PARTICLETOY_SOLVER_H

#include <vector>
#include "System.h"

using namespace std;

class Solver {
public:
    virtual void simulateStep(System* sys, float h) = 0;
};

#endif //PARTICLETOY_SOLVER_H