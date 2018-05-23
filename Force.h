#pragma once

#include "Particle.h"
#include <vector>

class Force {
 public:
 
  void draw();
  void apply();
  std::vector<Particle*> particles;
//   Particle * const m_p1;   
//   Particle * const m_p2;   

};
