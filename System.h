#ifndef PARTICLETOY_SYSTEM_H
#define PARTICLETOY_SYSTEM_H

#include "Eigen/Dense"

#include "Particle.h"
#include "Force.h"
#include "Constraint.h"

using namespace Eigen;
using namespace std;

class Solver;
class System {
private:
    void drawParticles(bool drawVelocity, bool drawForce);
    void drawForces();
    void drawConstraints();

    void computeForces();
    void clearForces();

    float time;
public:
    System(Solver* solver);

    std::vector<Particle*> particles;
    std::vector<Force*> forces;
    std::vector<Constraint*> constraints;

    bool wallExists;
    bool springsCanBreak = false;
    float dt;
    //SystemBuilder::AvailableSystems type;
    Solver* solver;

    void addParticle(Particle* p);
    void addForce(Force* f);
    void addConstraint(Constraint* c);
    VectorXf checkWallCollision(VectorXf oldState, VectorXf newState);

    // ODE interface
    VectorXf derivEval();
    VectorXf computeDerivative();
    VectorXf getState();
    float getTime();
    void setState(VectorXf src);
    void setState(VectorXf newState, float time);
    unsigned long getDim();

    void step(bool adaptive);
    void free();
    void reset();
    void draw(bool drawVelocity, bool drawForces, bool drawConstraints);
    Particle* indexParticle(int x, int y, int xdim, int ydim);
    //Vec3f getNormalForParticleAtIndex(int x, int y, int xdim, int ydim);
};


#endif //PARTICLETOY_SYSTEM_H
