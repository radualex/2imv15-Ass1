#pragma once

#include "Constraint.h"
#include "Particle.h"

class CircularWireConstraint : public Constraint {
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const double radius);

  void draw() override;
  float constraint() override;
  float constraintDerivative() override;
  std::vector<Vec2f> J() override;
  std::vector<Vec2f> JDerivative() override;

 private:

  Particle * const m_p;
  Vec2f const m_center;
  double const m_radius;
};
