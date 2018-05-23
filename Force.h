#pragma once

#include "Particle.h"
#include <vector>

class Force {
 public:
 
  virtual void draw();
  virtual void apply();
  std::vector<Particle*> particles;
};
