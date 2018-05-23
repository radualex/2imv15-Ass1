#include "SpringForce.h"
#include <GL/glut.h>

SpringForce::SpringForce(std::vector<Particle*> p, double dist, double ks, double kd) :
 m_dist(dist), m_ks(ks), m_kd(kd) {
   this->particles = p;
 }

void SpringForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( particles[0]->m_Position[0], particles[0]->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( particles[1]->m_Position[0], particles[1]->m_Position[1] );
  glEnd();
}

void SpringForce::apply()
{

  Vec2f l = particles[0]->m_Position - particles[1]->m_Position;
  Vec2f i = particles[0]->m_Velocity - particles[1]->m_Velocity;

  Vec2f result = -((m_ks * (norm(l) - m_dist) + m_kd * ((l*i) / norm(l))) * (l / norm(l)));

  particles[0]->m_Force += result;
  particles[1]->m_Force -= result;
}
