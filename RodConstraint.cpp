#include "RodConstraint.h"
#include <GL/glut.h>

RodConstraint::RodConstraint(Particle *p1, Particle *p2, double dist) : Constraint({p1, p2}), m_p1(p1), m_p2(p2), m_dist(dist) {}

void RodConstraint::draw()
{
    glBegin(GL_LINES);
    glColor3f(0.8, 0.7, 0.6);
    glVertex2f(m_p1->m_Position[0], m_p1->m_Position[1]);
    glColor3f(0.8, 0.7, 0.6);
    glVertex2f(m_p2->m_Position[0], m_p2->m_Position[1]);
    glEnd();
}

float RodConstraint::constraint()
{
    Vec2f pVector = m_p1->m_Position - m_p2->m_Position;
    return pVector * pVector - pow(m_dist, 2);
}

float RodConstraint::constraintDerivative()
{
    Vec2f pVectorDerivative = 2 * (m_p1->m_Position - m_p2->m_Position);
    Vec2f vVectorDerivative = 2 * (m_p1->m_Velocity - m_p2->m_Velocity);
    return pVectorDerivative * vVectorDerivative;
}

std::vector<Vec2f> RodConstraint::J()
{
    Vec2f derivativeP1 = (m_p1->m_Position - m_p2->m_Position) * 2;
    Vec2f derivativeP2 = (m_p2->m_Position - m_p2->m_Position) * 2;

    return std::vector<Vec2f>{derivativeP1, derivativeP2};
}

std::vector<Vec2f> RodConstraint::JDerivative()
{
    Vec2f derivativeP1 = (m_p1->m_Velocity - m_p2->m_Velocity) * 2;
    Vec2f derivativeP2 = (m_p2->m_Velocity - m_p2->m_Velocity) * 2;

    return std::vector<Vec2f>{derivativeP1, derivativeP2};
}