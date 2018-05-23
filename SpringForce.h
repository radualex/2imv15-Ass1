#pragma once

#include "Particle.h"

class SpringForce: public Force {
 public:
  SpringForce(std::vector<Particle*> particless, double dist, double ks, double kd);

  void draw() override;
  void apply() override;

 private:
  double const m_dist;     // rest length
  double const m_ks, m_kd; // spring strength constants
};
