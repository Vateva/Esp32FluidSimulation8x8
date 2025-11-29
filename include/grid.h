#ifndef GRID_H
#define GRID_H

#include "config.h"
#include "particle.h"
#include "utils.h"

// grid cell types
enum class grid_cell_t { CELL_LIQUID, CELL_AIR, CELL_WALL };

class Grid {
 private:
  int size_x, size_y;  // size x * y
  float cell_size;  // physical size of one grid cell

  float* grid_vx;//array of vx in cell boundaries
  float* grid_vy;//array of vy in cell boundaries
  float* grid_previous_vx;// array of previous vx
  float* grid_previous_vy;// array of previous vy
  grid_cell_t* cell_type;//array of types of cell of grid
 

 public:
  Grid();
  void transferVelocityfromGridToParticle(Particle* particle);
};

#endif