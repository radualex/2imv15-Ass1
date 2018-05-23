#include "SpringForce.h"
#include <GL/glut.h>

SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd) :
  m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

void SpringForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
}

void SpringForce::apply()
{
  Vec2f l = m_p1->m_Position - m_p2->m_Position;
  Vec2f i = m_p1->m_Velocity - m_p2->m_Velocity;

  Vec2f result = -((m_ks * (norm(l) - m_dist) + m_kd * ((l*i) / norm(l))) * (l / norm(l)));

  m_p1->m_Force += result;
  m_p2->m_Force -= result;
}
