#include "grid.h"

Grid::Grid() {
  size_x = GRID_SIZE_X;
  size_y = GRID_SIZE_Y;
  int num_cells = size_x * size_y;

  // calculate cell spacing based on physical size and grid resolution
  float cell_size_x = PHYSICAL_WIDTH / size_x;
  float cell_size_y = PHYSICAL_HEIGHT / size_y;
  cell_size = (cell_size_x > cell_size_y) ? cell_size_x : cell_size_y;

  // allocate memory for arrays
  grid_vx = new float[num_cells];
  grid_vy = new float[num_cells];
  grid_previous_vx = new float[num_cells];
  grid_previous_vy = new float[num_cells];
  vx_weight_acumulator = new float[num_cells];
  vy_weight_acumulator = new float[num_cells];
  cell_type = new grid_cell_t[num_cells];

  // initialize velocities to zero
  for (int i = 0; i < num_cells; i++) {
    grid_vx[i] = 0.0f;
    grid_vy[i] = 0.0f;
    grid_previous_vx[i] = 0.0f;
    grid_previous_vy[i] = 0.0f;
  }

  // set boundary cells to walls, interior to air
  for (int i = 0; i < size_x; i++) {
    for (int j = 0; j < size_y; j++) {
      int index = i * size_y + j;

      if (i == 0 || i == size_x - 1 || j == 0) {
        cell_type[index] = grid_cell_t::CELL_WALL;
      } else {
        cell_type[index] = grid_cell_t::CELL_AIR;
      }
    }
  }
}

InterpolationData Grid::getInterpolationData(Particle* particle, VelocityComponent component) {
  InterpolationData data;

  // determine offsets based on which velocity component we are interpolating
  // particle positions are shifted since velocities are in the center of sides of cell
  // doing this we transform from the cells grid to the grid formed by
  // the points where the vx velocities are( center of the vertical sides of the cells)
  float offset_x = (component == VelocityComponent::VY) ? cell_size * 0.5 : 0.0;
  float offset_y = (component == VelocityComponent::VX) ? cell_size * 0.5 : 0.0;

  // shift particle position into the velocity components coordinate system
  float particle_pos_x = particle->getX() - offset_x;
  float particle_pos_y = particle->getY() - offset_y;
  // clamp particle position to safe range (one cell away from boundaries)
  particle_pos_x = utils::clamp(particle_pos_x, cell_size, (size_x - 1) * cell_size);
  particle_pos_y = utils::clamp(particle_pos_y, cell_size, (size_y - 1) * cell_size);

  // determine which cell (within the staggered velocity grid)
  uint16_t velocities_grid_cell_x = particle_pos_x / cell_size;
  uint16_t velocities_grid_cell_y = particle_pos_y / cell_size;
  // clamp cell indices so +1 never exceeds bounds
  velocities_grid_cell_x = utils::clamp(velocities_grid_cell_x, 0, size_x - 2);
  velocities_grid_cell_y = utils::clamp(velocities_grid_cell_y, 0, size_y - 2);

  // calculate where inside the cell the particle is
  float delta_x = particle_pos_x - velocities_grid_cell_x * cell_size;
  float delta_y = particle_pos_y - velocities_grid_cell_y * cell_size;

  // calculate weights
  data.weight1 = (1 - (delta_x / cell_size)) * (1 - (delta_y / cell_size));
  data.weight2 = (delta_x / cell_size) * (1 - (delta_y / cell_size));
  data.weight3 = (delta_x / cell_size) * (delta_y / cell_size);
  data.weight4 = (1 - (delta_x / cell_size)) * (delta_y / cell_size);

  // find the index of the four grid corners in the 1D array
  data.index_00 = velocities_grid_cell_x * size_y + velocities_grid_cell_y;              // bottom left
  data.index_10 = (velocities_grid_cell_x + 1) * size_y + velocities_grid_cell_y;        // bottom right
  data.index_11 = (velocities_grid_cell_x + 1) * size_y + (velocities_grid_cell_y + 1);  // top right
  data.index_01 = velocities_grid_cell_x * size_y + (velocities_grid_cell_y + 1);        // top left

  return data;
}

void Grid::transferVelocityfromParticleToGrid(Particle* particle) {
  //***** vx transfer from particle to grid *****

  InterpolationData vx_data = getInterpolationData(particle, VelocityComponent::VX);

  // we will clear the corners once per algorithm iteration in main.cpp, not once per particle
  /*for (int i = 0; i < num_cells; i++) {
    grid_vx[i] = 0;               // clear velocity of corners
    vx_weight_acumulator[i] = 0;  // clear weights
  }*/

  // add particle velocity * weight to each corner
  grid_vx[vx_data.index_00] = grid_vx[vx_data.index_00] + vx_data.weight1 * particle->getVx();
  grid_vx[vx_data.index_10] = grid_vx[vx_data.index_10] + vx_data.weight2 * particle->getVx();
  grid_vx[vx_data.index_11] = grid_vx[vx_data.index_11] + vx_data.weight3 * particle->getVx();
  grid_vx[vx_data.index_01] = grid_vx[vx_data.index_01] + vx_data.weight4 * particle->getVx();

  // add each each corner wright to each corner weight acumulator
  vx_weight_acumulator[vx_data.index_00] = vx_weight_acumulator[vx_data.index_00] + vx_data.weight1;
  vx_weight_acumulator[vx_data.index_10] = vx_weight_acumulator[vx_data.index_10] + vx_data.weight2;
  vx_weight_acumulator[vx_data.index_11] = vx_weight_acumulator[vx_data.index_11] + vx_data.weight3;
  vx_weight_acumulator[vx_data.index_01] = vx_weight_acumulator[vx_data.index_01] + vx_data.weight4;

  // divide by weight acumulator
  grid_vx[vx_data.index_00] = grid_vx[vx_data.index_00] / vx_weight_acumulator[vx_data.index_00];
  grid_vx[vx_data.index_10] = grid_vx[vx_data.index_10] / vx_weight_acumulator[vx_data.index_10];
  grid_vx[vx_data.index_11] = grid_vx[vx_data.index_11] / vx_weight_acumulator[vx_data.index_11];
  grid_vx[vx_data.index_01] = grid_vx[vx_data.index_01] / vx_weight_acumulator[vx_data.index_01];

  //***** vy transfer from particle to grid *****

  InterpolationData vy_data = getInterpolationData(particle, VelocityComponent::VY);

  // add particle velocity * weight to each corner
  grid_vy[vy_data.index_00] = grid_vy[vy_data.index_00] + vy_data.weight1 * particle->getVy();
  grid_vy[vy_data.index_10] = grid_vy[vy_data.index_10] + vy_data.weight2 * particle->getVy();
  grid_vy[vy_data.index_11] = grid_vy[vy_data.index_11] + vy_data.weight3 * particle->getVy();
  grid_vy[vy_data.index_01] = grid_vy[vy_data.index_01] + vy_data.weight4 * particle->getVy();

  // add each each corner wright to each corner weight acumulator
  vy_weight_acumulator[vy_data.index_00] = vy_weight_acumulator[vy_data.index_00] + vy_data.weight1;
  vy_weight_acumulator[vy_data.index_10] = vy_weight_acumulator[vy_data.index_10] + vy_data.weight2;
  vy_weight_acumulator[vy_data.index_11] = vy_weight_acumulator[vy_data.index_11] + vy_data.weight3;
  vy_weight_acumulator[vy_data.index_01] = vy_weight_acumulator[vy_data.index_01] + vy_data.weight4;
}

// normalize cell corner velocities dividing by the weight accumualtor
// once per cell on every cell that had a particle contributing
void Grid::normalizeGridVelocities() {
  int num_cells = size_x * size_y;

  for (int i = 0; i < num_cells; i++) {
    // only divide if some particle contributed to this cell
    if (vx_weight_acumulator[i] > 0.0) {
      grid_vx[i] = grid_vx[i] / vx_weight_acumulator[i];
    }
    if (vy_weight_acumulator[i] > 0.0) {
      grid_vy[i] = grid_vy[i] / vy_weight_acumulator[i];
    }
  }
}

void Grid::forcingIncompressibility() {
  float divergence;
  int index_iplus1_j, index_i_j, index_i_jplus1, index_iminus1_j, index_i_jminus1;
  float sum_s;
  float vx_iplus1_j, vx_i_j, vy_i_jplus1, vy_i_j;

  for (int iter = 0; iter < COMPRENSIBILITY_ITERATIONS; iter++) {
    for (int i = 1; i < size_x - 1; i++) {
      for (int j = 1; j < size_y - 1; j++) {
        index_iplus1_j = (i + 1) * size_y + j;
        index_i_j = i * size_y + j;
        index_i_jplus1 = i * size_y + (j + 1);

        index_iminus1_j = (i - 1) * size_y + j;
        index_i_jminus1 = i * size_y + (j - 1);

        if (cell_type[index_i_j] != grid_cell_t::CELL_LIQUID) {
          continue;
        }

        // s = 1 if not wall, 0 if wall
        float s_iplus1_j = (cell_type[index_iplus1_j] != grid_cell_t::CELL_WALL) ? 1.0 : 0.0;  // 1 if fluid 0 if wall
        float s_iminus1_j = (cell_type[index_iminus1_j] != grid_cell_t::CELL_WALL) ? 1.0 : 0.0;
        float s_i_jplus1 = (cell_type[index_i_jplus1] != grid_cell_t::CELL_WALL) ? 1.0 : 0.0;
        float s_i_jminus1 = (cell_type[index_i_jminus1] != grid_cell_t::CELL_WALL) ? 1.0 : 0.0;

        vx_iplus1_j = grid_vx[index_iplus1_j];  // right vx
        vx_i_j = grid_vx[index_i_j];            // left vx
        vy_i_jplus1 = grid_vy[index_i_jplus1];  // top vy
        vy_i_j = grid_vy[index_i_j];            // bottom vy

        divergence = OVERRELAXATION * (vx_iplus1_j - vx_i_j + vy_i_jplus1 - vy_i_j);

        sum_s = s_iplus1_j + s_iminus1_j + s_i_jplus1 + s_i_jminus1;

        if (sum_s == 0) {
          continue;
        }
        grid_vx[index_i_j] = grid_vx[index_i_j] + (divergence * (s_iminus1_j / sum_s));  // left vx
        grid_vx[index_iplus1_j] = grid_vx[index_iplus1_j] - (divergence * (s_iplus1_j / sum_s));
        grid_vy[index_i_j] = grid_vy[index_i_j] + (divergence * (s_i_jminus1 / sum_s));  // bottom vy // right vx
        grid_vy[index_i_jplus1] = grid_vy[index_i_jplus1] - (divergence * (s_i_jplus1 / sum_s));  // top vy
      }
    }
  }
}

void Grid::transferVelocityfromGridToParticle(Particle* particle) {
  //***** vx transfer from grid to particle *****

  InterpolationData vx_data = getInterpolationData(particle, VelocityComponent::VX);

  // pic velocity = weighted average of current grid velocities
  float pic_vx = (vx_data.weight1 * grid_vx[vx_data.index_00] + vx_data.weight2 * grid_vx[vx_data.index_10] +
                  vx_data.weight3 * grid_vx[vx_data.index_11] + vx_data.weight4 * grid_vx[vx_data.index_01]);

  // flip correction = weighted average of velocity CHANGE
  float corr_vx = (vx_data.weight1 * (grid_vx[vx_data.index_00] - grid_previous_vx[vx_data.index_00]) +
                   vx_data.weight2 * (grid_vx[vx_data.index_10] - grid_previous_vx[vx_data.index_10]) +
                   vx_data.weight3 * (grid_vx[vx_data.index_11] - grid_previous_vx[vx_data.index_11]) +
                   vx_data.weight4 * (grid_vx[vx_data.index_01] - grid_previous_vx[vx_data.index_01]));

  // flip velocity = current particle velocity + grid velocity change
  float flip_vx = particle->getVx() + corr_vx;

  // blend pic and flip based on ratio
  float new_vx = (1.0 - FLIP_RATIO) * pic_vx + FLIP_RATIO * flip_vx;
  particle->setVx(new_vx);

  //***** vy transfer from grid to particle *****

  InterpolationData vy_data = getInterpolationData(particle, VelocityComponent::VY);

  // pic velocity = weighted average of current grid velocities
  float pic_vy = (vy_data.weight1 * grid_vy[vy_data.index_00] + vy_data.weight2 * grid_vy[vy_data.index_10] +
                  vy_data.weight3 * grid_vy[vy_data.index_11] + vy_data.weight4 * grid_vy[vy_data.index_01]);

  // flip correction = weighted average of velocity CHANGE
  float corr_vy = (vy_data.weight1 * (grid_vy[vy_data.index_00] - grid_previous_vy[vy_data.index_00]) +
                   vy_data.weight2 * (grid_vy[vy_data.index_10] - grid_previous_vy[vy_data.index_10]) +
                   vy_data.weight3 * (grid_vy[vy_data.index_11] - grid_previous_vy[vy_data.index_11]) +
                   vy_data.weight4 * (grid_vy[vy_data.index_01] - grid_previous_vy[vy_data.index_01]));

  // flip velocity = current particle velocity + grid velocity change
  float flip_vy = particle->getVy() + corr_vy;

  // blend pic and flip based on ratio
  float new_vy = (1.0 - FLIP_RATIO) * pic_vy + FLIP_RATIO * flip_vy;
  particle->setVy(new_vy);
}