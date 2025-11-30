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
  int size_x, size_y;   // size x * y
  float cell_size;      // physical size of one grid cell
  float inv_cell_size;  // inverse of cell size to avoid division

  float* grid_vx;           // array of vx in cell boundaries
  float* grid_vy;           // array of vy in cell boundaries
  float* grid_previous_vx;  // array of previous vx
  float* grid_previous_vy;  // array of previous vy
  // weight accumulators used in particle to grid velocity transfer
  float* vx_weight_accumulator;  // weight accumulators for vx transfer
  float* vy_weight_accumulator;  // weight accumulators for vy transfer
  grid_cell_t* cell_type;        // array of types of cell of grid
  // add to private section of Grid class

  // particle spatial hash grid (for pushing particles apart)
  int p_num_x;               // number of cells in particle grid x
  int p_num_y;               // number of cells in particle grid y
  float p_inv_spacing;       // 1.0 / (2.2 * particle_radius)
  int* num_cell_particles;   // count of particles per cell
  int* first_cell_particle;  // starting index for each cell in particle list
  int* cell_particle_ids;    // flat list of particle indices sorted by cell

  // particle density tracking (for drift compensation)
  float* particle_density;      // density of particles per cell
  float particle_rest_density;  // average density at rest (computed once)
  float particle_radius;        // radius of particles
 public:
  Grid();

  // compute interpolation weights and indices for specified velocity component
  InterpolationData getInterpolationData(Particle* particle, VelocityComponent component);

  void markCellWalls();
  void markCellWithLiquid(Particle* particle);
  void savePreviousVelocities();
  void clearVelocitiesAndWeights();
  void resetCellTypesToAir();
  void initParticleSpatialHash(int max_particles, float particle_radius);
  void pushParticlesApart(Particle* particles, int num_particles, int num_iterations);
  void updateParticleDensity(Particle* particles, int num_particles);

  void transferVelocityfromParticleToGrid(Particle* particle);
  void normalizeGridVelocities();
  void restoreSolidCellVelocities();

  void forcingIncompressibility();

  void transferVelocityfromGridToParticle(Particle* particle);

  void handleParticleCollision(Particle* particle);
};

#endif