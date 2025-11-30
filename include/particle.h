#ifndef PARTICLE_H
#define PARTICLE_H

#include "config.h"

struct Particle {
  // public members for direct access
  float x_pos;
  float y_pos;
  float vx;
  float vy;

  // constructor
  Particle() : x_pos(0.0f), y_pos(0.0f), vx(0.0f), vy(0.0f) {}

  // inline methods avoiding jump instructions.

  // apply gravity
  inline void addGravity(float gravity_x, float gravity_y) {
    vx = vx + DELTA_T * gravity_x;
    vy = vy + DELTA_T * gravity_y;
  }

  // update position based on velocity
  inline void updatePosition() {
    x_pos = x_pos + DELTA_T * vx;
    y_pos = y_pos + DELTA_T * vy;
  }
};

#endif