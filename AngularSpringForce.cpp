#include "SpringForce.h"
#include "AngularSpringForce.h"

using namespace std;

AngularSpringForce::AngularSpringForce(Particle *p1, Particle *p2, Particle *p3, float dist, float ks, float kd) :
        AngularSpringForce({p1, p2, p3}, dist, ks, kd) {}

AngularSpringForce::AngularSpringForce(std::vector<Particle *> particles, float dist, float ks, float kd) : dist(dist),ks(ks), kd(kd) {
    setTarget(particles);
}

void AngularSpringForce::setTarget(std::vector<Particle *> particles) {
    assert(particles.size() == 3);
    this->particles = particles;
}

void AngularSpringForce::apply() {
    Vec2f l1 = particles[0]->m_Position - particles[1]->m_Position;
    Vec2f l2 = particles[1]->m_Position - particles[2]->m_Position;
    double cosAngle = (l1 * l2) / (norm(l1) * norm(l2));
    
    if (cosAngle > 1.0) cosAngle = 1.0;
    if (cosAngle < -1.0) cosAngle = -1.0;
    double angle = acos(cosAngle);
    Vec3f l = particles[0]->m_Position - particles[2]->m_Position;
    Vec3f ld = particles[0]->m_Velocity - particles[2]->m_Velocity;

    // Compute spring force
    double b = norm(l1);
    double c = norm(l2);
    Vec3f result = -(ks * (norm(l) - sqrt(b * b + c * c - 2 * b * c * cos(dist))) + kd * ((l * ld) / norm(l))) * (l / norm(l));

    particles[0]->m_Force += result;
    particles[2]->m_Force -= result;
}

void AngularSpringForce::draw() {
//    glBegin(GL_LINES);
//    glColor3f(1.0f, 1.0f, 0.0f);
//    glVertex3f(particles[0]->position[0], particles[0]->position[1], particles[0]->position[2]);
//    glVertex3f(particles[1]->position[0], particles[1]->position[1], particles[1]->position[2]);
//    glVertex3f(particles[0]->position[0], particles[0]->position[1], particles[0]->position[2]);
//    glVertex3f(particles[2]->position[0], particles[2]->position[1], particles[2]->position[2]);
//    glEnd();
}

map<int, map<int, float>> AngularSpringForce::jx() {
    map<int, map<int, float>> values = map<int, map<int, float>>();
    return values;
}

MatrixXf AngularSpringForce::jv() {
    return MatrixXf();
}
