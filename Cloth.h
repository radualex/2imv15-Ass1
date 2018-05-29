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
    Cloth(int x, int y);
    void init();
    void reset();
    void draw();
    std::vector<Force *> fVector;
    std::vector<Constraint *> cVector;

private:
    std::vector<Particle*> particles;
    int dimX, dimY;
    const float particleMass;
    const float damping;
    const float spring;
    //
    float deltaX;
    float deltaY;
    void applyForces();
    void applyConstraints();
};