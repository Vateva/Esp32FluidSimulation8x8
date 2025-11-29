// config.h
#ifndef PARTICLE_H
#define PARTICLE_H
#include "config.h"

class Particle {
 private:
  float x_pos;  // position x
  float y_pos;  // position y
  float vx;     // velocity x
  float vy;     // velocity y

 public:
  Particle();  // constructor
  void addGravity();
  void updatePosition();
  void pushParticleOutOfObstacle();

  // position getters
  float getX() const;
  float getY() const;

  // velocity getters
  float getVx() const;
  float getVy() const;
  
  // velocity setters
  void setVx(float new_vx);
  void setVy(float new_vy);
};

#endif