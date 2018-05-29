#pragma once

#include "Particle.h"
#include <vector>

#include <Eigen/Dense>
#include <map>

using namespace Eigen;
using namespace std;

class Force {
protected:
    bool active = true;

public:
    std::vector<Particle*> particles;
    virtual void setTarget(std::vector<Particle*> particles) = 0;
    virtual void apply() = 0;
    virtual void draw() = 0;

    virtual map<int, map<int, float>> jx() = 0;
    virtual MatrixXf jv() = 0;
    void setActive(bool state);
    bool toggle();

};