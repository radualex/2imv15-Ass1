#include "Particle.h"
#include <GL/glut.h>

Particle::Particle(const Vec2f &ConstructPos, float mass) : m_ConstructPos(ConstructPos), m_Position(Vec2f(0.0, 0.0)), m_Velocity(Vec2f(0.0, 0.0)), m_Force(Vec2f(0.0, 0.0)), mass(mass)
{
}

Particle::~Particle(void)
{
}

void Particle::reset()
{
	m_Position = m_ConstructPos;
	m_Velocity = Vec2f(0.0, 0.0);
	m_Force = Vec2f(0.0, 0.0);
}

void Particle::updateVelocity(double dt)
{
	Vec2f scaled = m_Force * (dt/mass);
	m_Velocity += scaled;
}

void Particle::updatePosition(double dt) 
{
	Vec2f scaled = m_Velocity * dt;
	m_Position += scaled;
}
void Particle::clearForce()
{
	m_Force = Vec2f(0.0, 0.0);
}
void Particle::draw()
{
	const double h = 0.03;
	glColor3f(1.f, 1.f, 1.f);
	glBegin(GL_QUADS);
	glVertex2f(m_Position[0] - h / 2.0, m_Position[1] - h / 2.0);
	glVertex2f(m_Position[0] + h / 2.0, m_Position[1] - h / 2.0);
	glVertex2f(m_Position[0] + h / 2.0, m_Position[1] + h / 2.0);
	glVertex2f(m_Position[0] - h / 2.0, m_Position[1] + h / 2.0);
	glEnd();

	// glColor3f(1.0f, 0.0f, 0.0f);
	// glBegin(GL_LINES);
	// glVertex2f(m_Position[0], m_Position[1]);
	// glVertex2f(m_Position[0] + m_Force[0] * 0.2f, m_Position[1] + m_Force[1] * 0.2f);
	// glEnd();
}
