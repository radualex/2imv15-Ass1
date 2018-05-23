#pragma once

#include "Particle.h"
#include "Force.h"

class GravityForce: public Force {
 public:
  GravityForce(std::vector<Particle*> p, Vec2f direction);

  void draw() override;
  void apply() override;

  Vec2f direction;
};
