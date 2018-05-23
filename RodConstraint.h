#pragma once

#include "Particle.h"
#include "Constraint.h"

class RodConstraint: public Constraint {
 public:
  RodConstraint(Particle *p1, Particle * p2, double dist);

  void draw() override;
  float constraint() override;
  float constraintDerivative() override;
  std::vector<Vec2f> J() override;
  std::vector<Vec2f> JDerivative() override;

 private:

  Particle * const m_p1;
  Particle * const m_p2;
  double const m_dist;
};
