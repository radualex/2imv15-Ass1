#include "../Particle.h"
#include "Force.h"
#include "SpringForce.h"
#include <map>

class AngularSpringForce : public Force {

public:
    AngularSpringForce(Particle *p1, Particle * p2, Particle * p3, float dist, float ks, float kd);
    AngularSpringForce(std::vector<Particle*> particles, float dist, float ks, float kd);

    void draw() override;
    void setTarget(std::vector<Particle*> particles) override;
    void apply() override;
    map<int, map<int, float>> jx() override;
    MatrixXf jv() override;

private:
    float const dist;     // rest length
    float const ks, kd; // spring strength constants
};
