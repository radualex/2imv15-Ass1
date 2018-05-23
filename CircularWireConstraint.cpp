#include "CircularWireConstraint.h"
#include <GL/glut.h>

#define PI 3.1415926535897932384626433832795

static void draw_circle(const Vec2f & vect, float radius)
{
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0,1.0,0.0); 
	for (int i=0; i<360; i=i+18)
	{
		float degInRad = i*PI/180;
		glVertex2f(vect[0]+cos(degInRad)*radius,vect[1]+sin(degInRad)*radius);
	}
	glEnd();
}

CircularWireConstraint::CircularWireConstraint(Particle *p, const Vec2f & center, const double radius) :
	Constraint({}), m_p(p), m_center(center), m_radius(radius) {}

void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);
}

float CircularWireConstraint::constraint()
{
    Vec2f pVector = m_p->m_Position - m_center;
    return pVector * pVector - pow(m_radius, 2);
}

float CircularWireConstraint::constraintDerivative()
{
    Vec2f pVectorDerivative = 2 * (m_p->m_Position - m_center);
    Vec2f vVectorDerivative = 2 * m_p->m_Velocity;
    return pVectorDerivative * vVectorDerivative;
}

std::vector<Vec2f> CircularWireConstraint::J()
{
    Vec2f derivativeP = (m_p->m_Position - m_center) * 2;
    return std::vector<Vec2f>{derivativeP};
}

std::vector<Vec2f> CircularWireConstraint::JDerivative()
{
    Vec2f derivativeP = m_p->m_Velocity * 2;
    return std::vector<Vec2f>{derivativeP};
}
