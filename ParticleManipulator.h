#pragma once

#include "include/Eigen/Dense"

#include "Particle.h"
#include "Force.h"
#include "Constraint.h"

using namespace Eigen;
using namespace std;

class ParticleManipulator {
private:

    void computeForces();
    void clearForces();
    void apply_constraints(float ks, float kd);
    void calculateDerivative();
    void apply_forces();
    
public:
    ParticleManipulator();

    std::vector<Particle*> pVector;
    std::vector<Force*> fVector;
    std::vector<Constraint*> cVector;
    bool walls = false;

    void addParticle(Particle* p);
    void addForce(Force* f);
    int getPositionOfParticle(Particle *p);
    void addConstraint(Constraint* c);
    void free_data();
    void clear_data();
    void derivative();
    VectorXf fixCollisions(VectorXf newState);
    void drawWalls();

    // ODE interface
    unsigned long getDim();
    VectorXf derivEval();
    VectorXf computeDerivative();
    VectorXf getState();
    void setState(VectorXf newState);
};
