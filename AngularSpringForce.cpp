#include "SpringForce.h"
#include "AngularSpringForce.h"

using namespace std;

AngularSpringForce::AngularSpringForce(std::vector<Particle*> p, int p1, int p2, int p3, double dist, double ks, double kd) :
 m_p1(p1), m_p2(p2), m_p3(p3), m_dist(dist), m_ks(ks), m_kd(kd) {
   this->particles = p;
 }

void AngularSpringForce::apply() {
    Vec2f l1 = particles[m_p1]->m_Position - particles[m_p2]->m_Position;
    Vec2f l2 = particles[m_p2]->m_Position - particles[m_p3]->m_Position;
    double cosAngle = (l1 * l2) / (norm(l1) * norm(l2));
    
    if (cosAngle > 1.0) cosAngle = 1.0;
    if (cosAngle < -1.0) cosAngle = -1.0;
    double angle = acos(cosAngle);
    Vec2f l = particles[m_p1]->m_Position - particles[m_p3]->m_Position;
    Vec2f ld = particles[m_p1]->m_Velocity - particles[m_p3]->m_Velocity;

    // Compute spring force
    double b = norm(l1);
    double c = norm(l2);
    Vec2f result = -(m_ks * (norm(l) - sqrt(b * b + c * c - 2 * b * c * cos(m_dist))) + m_kd * ((l * ld) / norm(l))) * (l / norm(l));

    particles[m_p1]->m_Force += result;
    particles[m_p3]->m_Force -= result;
}

void AngularSpringForce::draw() {
    glBegin(GL_LINES);
    glColor3f(1.0f, 1.0f, 0.0f);
    glVertex2f(particles[m_p1]->m_Position[0], particles[m_p1]->m_Position[1]);
    glVertex2f(particles[m_p2]->m_Position[0], particles[m_p2]->m_Position[1]);
    glVertex2f(particles[m_p3]->m_Position[0], particles[m_p3]->m_Position[1]);
    glEnd();
}