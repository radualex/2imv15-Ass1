#include "GravityForce.h"
#include <GL/glut.h>

GravityForce::GravityForce(std::vector<Particle*> p, Vec2f direction) :
 direction(direction) {
   this->particles = p;
 }

void GravityForce::draw()
{
}

void GravityForce::apply()
{
    for(Particle* p : particles)
    {
        p->m_Force += p->mass * direction;
    }
}
