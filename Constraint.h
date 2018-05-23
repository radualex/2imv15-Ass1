#pragma once

#include "Particle.h"
#include <vector>
#include <math.h>

class Constraint {
 public:
  Constraint(std::vector<Particle*> particles): particles(particles) {};

  virtual void draw();
  virtual float constraint();
  virtual float constraintDerivative();
  virtual std::vector<Vec2f> J();
  virtual std::vector<Vec2f> JDerivative();

  std::vector<Particle*> particles;
};
