#ifndef PARTICLETOY_SYSTEM_H
#define PARTICLETOY_SYSTEM_H

#include "include/Eigen/Dense"

#include "Particle.h"
#include "Force.h"
#include "Constraint.h"

using namespace Eigen;
using namespace std;

class Solver;
class System {
private:

    void computeForces();
    void clearForces();
    void apply_constraints(float ks, float kd);
    void calculateDerivative();
    void apply_forces();

    float time;
public:
    System();
    //System(Solver* solver);

    std::vector<Particle*> pVector;
    std::vector<Force*> fVector;
    std::vector<Constraint*> cVector;

    void addParticle(Particle* p);
    void addForce(Force* f);
    void addConstraint(Constraint* c);
    void free_data();
    void clear_data();



    // ODE interface
    float getTime();
    unsigned long getDim();
    VectorXf derivEval();
    VectorXf computeDerivative();
    VectorXf getState();
    void setState(VectorXf src);
    void setState(VectorXf newState, float time);
    int getPositionOfParticle(Particle *p);


    bool wallExists;


    //float dt;
    //SystemBuilder::AvailableSystems type;
    //Solver* solver;

    VectorXf checkWallCollision(VectorXf oldState, VectorXf newState);


    //void step(bool adaptive);
    //Particle* indexParticle(int x, int y, int xdim, int ydim);
    //Vec3f getNormalForParticleAtIndex(int x, int y, int xdim, int ydim);
};


#endif //PARTICLETOY_SYSTEM_H
