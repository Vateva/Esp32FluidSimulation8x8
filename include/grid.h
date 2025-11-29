#ifndef GRID_H
#define GRID_H

#include "config.h"
#include "particle.h"
#include "utils.h"

// grid cell types
enum class grid_cell_t { CELL_LIQUID, CELL_AIR, CELL_WALL };

// velocity component selector for interpolation
enum class VelocityComponent { VX, VY };

// struct to hold bilinear interpolation indices and weights
// used for transferring velocities between particles and grid
struct InterpolationData {
  int index_00;   // bottom left corner index in 1D array
  int index_10;   // bottom right corner index in 1D array
  int index_11;   // top right corner index in 1D array
  int index_01;   // top left corner index in 1D array
  float weight1;  // weight for bottom left
  float weight2;  // weight for bottom right
  float weight3;  // weight for top right
  float weight4;  // weight for top left
};

class Grid {
 private:
  int size_x, size_y;  // size x * y
  float cell_size;     // physical size of one grid cell

  float* grid_vx;           // array of vx in cell boundaries
  float* grid_vy;           // array of vy in cell boundaries
  float* grid_previous_vx;  // array of previous vx
  float* grid_previous_vy;  // array of previous vy
  // weight accumulators used in particle to grid velocity transfer
  float* vx_weight_acumulator;  // weight accumulators for vx transfer
  float* vy_weight_acumulator;  // weight accumulators for vy transfer
  grid_cell_t* cell_type;       // array of types of cell of grid

 public:
  Grid();

  
// compute interpolation weights and indices for specified velocity component
  InterpolationData getInterpolationData(Particle* particle, VelocityComponent component);

  void transferVelocityfromParticleToGrid(Particle* particle);
  void normalizeGridVelocities();

  void forcingIncompressibility();

  void transferVelocityfromGridToParticle(Particle* particle);
};

#endif