#pragma once

#include <gfx/vec2.h>


class Particle
{
public:

	Particle(const Vec2f & ConstructPos, float mass);
	virtual ~Particle(void);

	void reset();
	void draw();
	void clearForce();

	Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;
	Vec2f m_Force;
	float mass;

};
