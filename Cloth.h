#include <gfx/vec2.h>
#include "Particle.h"
#include "Constraint.h"
#include "CircularWireConstraint.h"
#include "RodConstraint.h"
#include "Force.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include <math.h>


class Cloth
{
public:
    Cloth(int x, int y, std::vector<Particle*> &pVector,std::vector<Force*> &fVector, std::vector<Constraint*> &cVector);
    void reset();
    void draw();
    //std::vector<Force*> fVector;
    //std::vector<Constraint*> cVector;
    //std::vector<Particle*> particles;

private:
    int dimX, dimY;
    const float particleMass;
    const float damping;
    const float spring;
    //
    float deltaX;
    float deltaY;
    void init(std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector);
    void applyForces(std::vector<Particle*> &pVector, std::vector<Force*> &fVector);
    void applyConstraints(std::vector<Particle*> &pVector, std::vector<Constraint*> &cVector);
};