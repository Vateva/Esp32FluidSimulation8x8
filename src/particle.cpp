#include "particle.h"

// constructor
Particle::Particle() {
  x_pos = 0;
  y_pos = 0;
  vx = 0;
  vy = 0;
};

void Particle::addGravity() {
  vy = vy + DELTA_T * GRAVITY;
}

void Particle::updatePosition() {
  x_pos = x_pos + DELTA_T * vx;
  y_pos = y_pos + DELTA_T * vy;
}

void Particle::pushParticleOutOfObstacle() {}

//position getters
float Particle::getX() const {
  return x_pos;
}
float Particle::getY() const {
  return y_pos;
}

//velocity getters 
float Particle::getVx() const {
    return vx;
}
float Particle::getVy() const {
    return vy;
}

//velocity setters 
void Particle::setVx(float new_vx) {
    vx = new_vx;
}
void Particle::setVy(float new_vy) {
    vy = new_vy;
}