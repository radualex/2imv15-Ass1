#pragma once

#include "Particle.h"
#include "Force.h"
#include "SpringForce.h"
#include <GL/glut.h>

class AngularSpringForce : public Force {

public:
    AngularSpringForce(std::vector<Particle*> p, int p1, int p2, int p3, double dist, double ks, double kd);
    //AngularSpringForce(std::vector<Particle*> particles, float dist, float ks, float kd);

    void draw() override;
    void apply() override;

private:
    double const m_dist;     // rest length
    double const m_ks, m_kd; // spring strength constants
    int const m_p1, m_p2, m_p3;
};
