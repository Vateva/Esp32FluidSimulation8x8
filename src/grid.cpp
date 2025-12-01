#include "grid.h"

Grid::Grid() {
  size_x = GRID_SIZE_X;
  size_y = GRID_SIZE_Y;
  int num_cells = size_x * size_y;

  // calculate cell spacing based on physical size and grid resolution
  float cell_size_x = PHYSICAL_WIDTH / size_x;
  float cell_size_y = PHYSICAL_HEIGHT / size_y;
  cell_size = (cell_size_x > cell_size_y) ? cell_size_x : cell_size_y;
  inv_cell_size = 1.0f / cell_size;

  // allocate memory for arrays
  grid_vx = new float[num_cells];
  grid_vy = new float[num_cells];
  grid_previous_vx = new float[num_cells];
  grid_previous_vy = new float[num_cells];
  vx_weight_accumulator = new float[num_cells];
  vy_weight_accumulator = new float[num_cells];
  cell_type = new grid_cell_t[num_cells];
  // set to nullptr so we can check if they're valid before using
  particle_density = nullptr;
  particle_rest_density = 0.0f;
  particle_radius = 0.0f;
  p_num_x = 0;
  p_num_y = 0;
  p_inv_spacing = 0.0f;
  num_cell_particles = nullptr;
  first_cell_particle = nullptr;
  cell_particle_ids = nullptr;

  // initialize velocities to zero
  for (int i = 0; i < num_cells; i++) {
    grid_vx[i] = 0.0f;
    grid_vy[i] = 0.0f;
    grid_previous_vx[i] = 0.0f;
    grid_previous_vy[i] = 0.0f;
  }
}

void Grid::markCellWalls() {
  // reset all to air
  for (int i = 0; i < size_x * size_y; i++) {
    cell_type[i] = grid_cell_t::CELL_AIR;
  }

  // left wall (i = 0)
  for (int j = 0; j < size_y; j++) {
    cell_type[0 * size_y + j] = grid_cell_t::CELL_WALL;
  }

  // right wall (i = size_x - 1)
  for (int j = 0; j < size_y; j++) {
    cell_type[(size_x - 1) * size_y + j] = grid_cell_t::CELL_WALL;
  }

  // bottom wall (j = 0)
  for (int i = 0; i < size_x; i++) {
    cell_type[i * size_y + 0] = grid_cell_t::CELL_WALL;
  }

  // top wall (j = size_y - 1) -n
  for (int i = 0; i < size_x; i++) {
    cell_type[i * size_y + (size_y - 1)] = grid_cell_t::CELL_WALL;
  }
}

void Grid::markCellWithLiquid(Particle* particle) {
  float particle_pos_x = particle->x_pos;
  float particle_pos_y = particle->y_pos;

  // determine which cell contains this particle
  int cell_i = particle_pos_x * inv_cell_size;
  int cell_j = particle_pos_y * inv_cell_size;

  // clamp to valid interior range away from walls
  cell_i = utils::clamp(cell_i, 1, size_x - 2);
  cell_j = utils::clamp(cell_j, 1, size_y - 2);

  int cell_1d_index = cell_i * size_y + cell_j;
  cell_type[cell_1d_index] = grid_cell_t::CELL_LIQUID;
}

void Grid::savePreviousVelocities() {
  for (int i = 0; i < size_x * size_y; i++) {
    grid_previous_vx[i] = grid_vx[i];
    grid_previous_vy[i] = grid_vy[i];
  }
}

void Grid::clearVelocitiesAndWeights() {
  for (int i = 0; i < size_x * size_y; i++) {
    grid_vx[i] = 0.0f;
    grid_vy[i] = 0.0f;
    vx_weight_accumulator[i] = 0.0f;
    vy_weight_accumulator[i] = 0.0f;
  }
}

void Grid::resetCellTypesToAir() {
  for (int i = 0; i < size_x * size_y; i++) {
    if (cell_type[i] != grid_cell_t::CELL_WALL) {
      cell_type[i] = grid_cell_t::CELL_AIR;
    }
  }
}

void Grid::restoreSolidCellVelocities() {
  // after transferring velocities from particles to grid, some velocity
  // may have been scattered onto faces adjacent to solid cells
  // this resets those velocities to preserve boundary conditions

  for (int i = 0; i < size_x; i++) {
    for (int j = 0; j < size_y; j++) {
      int index = i * size_y + j;

      // check if current cell is solid
      bool is_solid = (cell_type[index] == grid_cell_t::CELL_WALL);

      // for u velocity at (i,j): restore if this cell is solid OR left neighbor is solid
      // vx velocity sits on the face between cell (i-1,j) and cell (i,j)
      // if either cell is solid, this velocity should not be modified by particles
      bool left_is_solid = (i > 0 && cell_type[(i - 1) * size_y + j] == grid_cell_t::CELL_WALL);
      if (is_solid || left_is_solid) {
        grid_vx[index] = grid_previous_vx[index];
      }

      // for vy velocity at (i,j): restore if this cell is solid OR bottom neighbor is solid
      // vy velocity sits on the face between cell (i,j-1) and cell (i,j)
      bool bottom_is_solid = (j > 0 && cell_type[i * size_y + (j - 1)] == grid_cell_t::CELL_WALL);
      if (is_solid || bottom_is_solid) {
        grid_vy[index] = grid_previous_vy[index];
      }
    }
  }
}
void Grid::transferVelocityfromParticleToGrid(Particle* particle) {
  // read particle data once to avoid repeated pointer dereferences
  float px = particle->x_pos;
  float py = particle->y_pos;
  float p_vx = particle->vx;
  float p_vy = particle->vy;

  // precompute values used by both vx and vy interpolation
  float half_cell = cell_size * 0.5f;
  float min_pos = cell_size;
  float max_pos_x = (size_x - 1) * cell_size;
  float max_pos_y = (size_y - 1) * cell_size;
  int max_cell_i = size_x - 2;
  int max_cell_j = size_y - 2;

  //***** vx transfer from particle to grid *****

  // determine offsets based on which velocity component we are interpolating
  // particle positions are shifted since velocities are in the center of sides of cell
  // doing this we transform from the cells grid to the grid formed by
  // the points where the vx velocities are (center of the vertical sides of the cells)
  // for vx: offset_x = 0, offset_y = half_cell

  // shift particle position into the velocity components coordinate system
  float vx_particle_pos_x = px;
  float vx_particle_pos_y = py - half_cell;

  // clamp particle position to safe range (one cell away from boundaries)
  vx_particle_pos_x = utils::clamp(vx_particle_pos_x, min_pos, max_pos_x);
  vx_particle_pos_y = utils::clamp(vx_particle_pos_y, min_pos, max_pos_y);

  // determine which cell (within the staggered velocity grid)
  int vx_velocities_grid_cell_i = (int)(vx_particle_pos_x * inv_cell_size);
  int vx_velocities_grid_cell_j = (int)(vx_particle_pos_y * inv_cell_size);

  // clamp cell indices so +1 never exceeds bounds
  vx_velocities_grid_cell_i = utils::clamp(vx_velocities_grid_cell_i, 0, max_cell_i);
  vx_velocities_grid_cell_j = utils::clamp(vx_velocities_grid_cell_j, 0, max_cell_j);

  // calculate where inside the cell the particle is
  float vx_delta_x = vx_particle_pos_x - vx_velocities_grid_cell_i * cell_size;
  float vx_delta_y = vx_particle_pos_y - vx_velocities_grid_cell_j * cell_size;

  // precompute normalized weights to avoid redundant calculations
  float vx_tx = vx_delta_x * inv_cell_size;
  float vx_ty = vx_delta_y * inv_cell_size;
  float vx_sx = 1.0f - vx_tx;
  float vx_sy = 1.0f - vx_ty;

  // calculate weights
  float vx_weight1 = vx_sx * vx_sy;  // weight for bottom left
  float vx_weight2 = vx_tx * vx_sy;  // weight for bottom right
  float vx_weight3 = vx_tx * vx_ty;  // weight for top right
  float vx_weight4 = vx_sx * vx_ty;  // weight for top left

  // find the index of the four grid corners in the 1d array
  int vx_base_index = vx_velocities_grid_cell_i * size_y + vx_velocities_grid_cell_j;
  int vx_index_00 = vx_base_index;               // bottom left
  int vx_index_10 = vx_base_index + size_y;      // bottom right
  int vx_index_11 = vx_base_index + size_y + 1;  // top right
  int vx_index_01 = vx_base_index + 1;           // top left

  // add particle velocity * weight to each corner
  grid_vx[vx_index_00] += vx_weight1 * p_vx;
  grid_vx[vx_index_10] += vx_weight2 * p_vx;
  grid_vx[vx_index_11] += vx_weight3 * p_vx;
  grid_vx[vx_index_01] += vx_weight4 * p_vx;

  // add each corner weight to each corner weight accumulator
  vx_weight_accumulator[vx_index_00] += vx_weight1;
  vx_weight_accumulator[vx_index_10] += vx_weight2;
  vx_weight_accumulator[vx_index_11] += vx_weight3;
  vx_weight_accumulator[vx_index_01] += vx_weight4;

  //***** vy transfer from particle to grid *****

  // determine offsets based on which velocity component we are interpolating
  // particle positions are shifted since velocities are in the center of sides of cell
  // doing this we transform from the cells grid to the grid formed by
  // the points where the vy velocities are (center of the horizontal sides of the cells)
  // for vy: offset_x = half_cell, offset_y = 0

  // shift particle position into the velocity components coordinate system
  float vy_particle_pos_x = px - half_cell;
  float vy_particle_pos_y = py;

  // clamp particle position to safe range (one cell away from boundaries)
  vy_particle_pos_x = utils::clamp(vy_particle_pos_x, min_pos, max_pos_x);
  vy_particle_pos_y = utils::clamp(vy_particle_pos_y, min_pos, max_pos_y);

  // determine which cell (within the staggered velocity grid)
  int vy_velocities_grid_cell_i = (int)(vy_particle_pos_x * inv_cell_size);
  int vy_velocities_grid_cell_j = (int)(vy_particle_pos_y * inv_cell_size);

  // clamp cell indices so +1 never exceeds bounds
  vy_velocities_grid_cell_i = utils::clamp(vy_velocities_grid_cell_i, 0, max_cell_i);
  vy_velocities_grid_cell_j = utils::clamp(vy_velocities_grid_cell_j, 0, max_cell_j);

  // calculate where inside the cell the particle is
  float vy_delta_x = vy_particle_pos_x - vy_velocities_grid_cell_i * cell_size;
  float vy_delta_y = vy_particle_pos_y - vy_velocities_grid_cell_j * cell_size;

  // precompute normalized weights to avoid redundant calculations
  float vy_tx = vy_delta_x * inv_cell_size;
  float vy_ty = vy_delta_y * inv_cell_size;
  float vy_sx = 1.0f - vy_tx;
  float vy_sy = 1.0f - vy_ty;

  // calculate weights
  float vy_weight1 = vy_sx * vy_sy;  // weight for bottom left
  float vy_weight2 = vy_tx * vy_sy;  // weight for bottom right
  float vy_weight3 = vy_tx * vy_ty;  // weight for top right
  float vy_weight4 = vy_sx * vy_ty;  // weight for top left

  // find the index of the four grid corners in the 1d array
  int vy_base_index = vy_velocities_grid_cell_i * size_y + vy_velocities_grid_cell_j;
  int vy_index_00 = vy_base_index;               // bottom left
  int vy_index_10 = vy_base_index + size_y;      // bottom right
  int vy_index_11 = vy_base_index + size_y + 1;  // top right
  int vy_index_01 = vy_base_index + 1;           // top left

  // add particle velocity * weight to each corner
  grid_vy[vy_index_00] += vy_weight1 * p_vy;
  grid_vy[vy_index_10] += vy_weight2 * p_vy;
  grid_vy[vy_index_11] += vy_weight3 * p_vy;
  grid_vy[vy_index_01] += vy_weight4 * p_vy;

  // add each corner weight to each corner weight accumulator
  vy_weight_accumulator[vy_index_00] += vy_weight1;
  vy_weight_accumulator[vy_index_10] += vy_weight2;
  vy_weight_accumulator[vy_index_11] += vy_weight3;
  vy_weight_accumulator[vy_index_01] += vy_weight4;
}

// normalize cell corner velocities dividing by the weight accumualtor
// once per cell on every cell that had a particle contributing
void Grid::normalizeGridVelocities() {
  int num_cells = size_x * size_y;

  for (int i = 0; i < num_cells; i++) {
    // only divide if some particle contributed to this cell
    if (vx_weight_accumulator[i] > 0.0f) {
      grid_vx[i] = grid_vx[i] / vx_weight_accumulator[i];
    }
    if (vy_weight_accumulator[i] > 0.0f) {
      grid_vy[i] = grid_vy[i] / vy_weight_accumulator[i];
    }
  }
}

void Grid::forcingIncompressibility() {
  float divergence;
  int index_iplus1_j, index_i_j, index_i_jplus1, index_iminus1_j, index_i_jminus1;
  float sum_s;
  float vx_iplus1_j, vx_i_j, vy_i_jplus1, vy_i_j;

  for (int iter = 0; iter < INCOMPRESSIBILITY_ITERATIONS; iter++) {
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
        float s_iplus1_j = (cell_type[index_iplus1_j] != grid_cell_t::CELL_WALL) ? 1.0f : 0.0f;
        float s_iminus1_j = (cell_type[index_iminus1_j] != grid_cell_t::CELL_WALL) ? 1.0f : 0.0f;
        float s_i_jplus1 = (cell_type[index_i_jplus1] != grid_cell_t::CELL_WALL) ? 1.0f : 0.0f;
        float s_i_jminus1 = (cell_type[index_i_jminus1] != grid_cell_t::CELL_WALL) ? 1.0f : 0.0f;

        vx_iplus1_j = grid_vx[index_iplus1_j];  // right vx
        vx_i_j = grid_vx[index_i_j];            // left vx
        vy_i_jplus1 = grid_vy[index_i_jplus1];  // top vy
        vy_i_j = grid_vy[index_i_j];            // bottom vy

        // compute velocity divergence (how much flow is leaving the cell)
        divergence = vx_iplus1_j - vx_i_j + vy_i_jplus1 - vy_i_j;

        // drift compensation: if local density is higher than rest density,
        // add extra divergence to push particles apart and prevent volume loss
        if (particle_density != nullptr && particle_rest_density > 0.0f) {
          float compression = particle_density[index_i_j] - particle_rest_density;
          if (compression > 0.0f) {
            // k = 1.0 is the stiffness of the density correction
            divergence = divergence - (K_FACTOR * compression);
          }
        }

        divergence = OVERRELAXATION * divergence;

        sum_s = s_iplus1_j + s_iminus1_j + s_i_jplus1 + s_i_jminus1;
        float inv_sum_s = 1.0f / sum_s;  // compute inverse only one to avoid division as much as possible
        if (sum_s == 0) {
          continue;
        }

        grid_vx[index_i_j] = grid_vx[index_i_j] + (divergence * (s_iminus1_j * inv_sum_s));
        grid_vx[index_iplus1_j] = grid_vx[index_iplus1_j] - (divergence * (s_iplus1_j * inv_sum_s));
        grid_vy[index_i_j] = grid_vy[index_i_j] + (divergence * (s_i_jminus1 * inv_sum_s));
        grid_vy[index_i_jplus1] = grid_vy[index_i_jplus1] - (divergence * (s_i_jplus1 * inv_sum_s));
      }
    }
  }
}

void Grid::transferVelocityfromGridToParticle(Particle* particle) {
  // read particle data once to avoid repeated pointer dereferences
  float px = particle->x_pos;
  float py = particle->y_pos;

  // precompute values used by both vx and vy interpolation
  float half_cell = cell_size * 0.5f;
  float min_pos = cell_size;
  float max_pos_x = (size_x - 1) * cell_size;
  float max_pos_y = (size_y - 1) * cell_size;
  int max_cell_i = size_x - 2;
  int max_cell_j = size_y - 2;

  //***** vx transfer from grid to particle *****

  // determine offsets based on which velocity component we are interpolating
  // particle positions are shifted since velocities are in the center of sides of cell
  // doing this we transform from the cells grid to the grid formed by
  // the points where the vx velocities are (center of the vertical sides of the cells)
  // for vx: offset_x = 0, offset_y = half_cell

  // shift particle position into the velocity components coordinate system
  float vx_particle_pos_x = px;
  float vx_particle_pos_y = py - half_cell;

  // clamp particle position to safe range (one cell away from boundaries)
  vx_particle_pos_x = utils::clamp(vx_particle_pos_x, min_pos, max_pos_x);
  vx_particle_pos_y = utils::clamp(vx_particle_pos_y, min_pos, max_pos_y);

  // determine which cell (within the staggered velocity grid)
  int vx_velocities_grid_cell_i = (int)(vx_particle_pos_x * inv_cell_size);
  int vx_velocities_grid_cell_j = (int)(vx_particle_pos_y * inv_cell_size);

  // clamp cell indices so +1 never exceeds bounds
  vx_velocities_grid_cell_i = utils::clamp(vx_velocities_grid_cell_i, 0, max_cell_i);
  vx_velocities_grid_cell_j = utils::clamp(vx_velocities_grid_cell_j, 0, max_cell_j);

  // calculate where inside the cell the particle is
  float vx_delta_x = vx_particle_pos_x - vx_velocities_grid_cell_i * cell_size;
  float vx_delta_y = vx_particle_pos_y - vx_velocities_grid_cell_j * cell_size;

  // precompute normalized weights to avoid redundant calculations
  float vx_tx = vx_delta_x * inv_cell_size;
  float vx_ty = vx_delta_y * inv_cell_size;
  float vx_sx = 1.0f - vx_tx;
  float vx_sy = 1.0f - vx_ty;

  // calculate weights
  float vx_weight1 = vx_sx * vx_sy;  // weight for bottom left
  float vx_weight2 = vx_tx * vx_sy;  // weight for bottom right
  float vx_weight3 = vx_tx * vx_ty;  // weight for top right
  float vx_weight4 = vx_sx * vx_ty;  // weight for top left

  // find the index of the four grid corners in the 1d array
  int vx_base_index = vx_velocities_grid_cell_i * size_y + vx_velocities_grid_cell_j;
  int vx_index_00 = vx_base_index;               // bottom left
  int vx_index_10 = vx_base_index + size_y;      // bottom right
  int vx_index_11 = vx_base_index + size_y + 1;  // top right
  int vx_index_01 = vx_base_index + 1;           // top left

  // offset to find the neighboring cell that shares this velocity
  // u velocities sit on vertical faces between cell (i-1,j) and cell (i,j)
  // in column-major layout, moving left one cell means subtracting size_y
  int vx_offset = size_y;

  // check if each velocity sample is valid
  // a velocity is valid if at least one of the two cells it borders is not air
  // if both cells are air, no fluid ever contributed to that velocity
  float valid_vx_00 =
      (cell_type[vx_index_00] != grid_cell_t::CELL_AIR || cell_type[vx_index_00 - vx_offset] != grid_cell_t::CELL_AIR)
          ? 1.0f
          : 0.0f;
  float valid_vx_10 =
      (cell_type[vx_index_10] != grid_cell_t::CELL_AIR || cell_type[vx_index_10 - vx_offset] != grid_cell_t::CELL_AIR)
          ? 1.0f
          : 0.0f;
  float valid_vx_11 =
      (cell_type[vx_index_11] != grid_cell_t::CELL_AIR || cell_type[vx_index_11 - vx_offset] != grid_cell_t::CELL_AIR)
          ? 1.0f
          : 0.0f;
  float valid_vx_01 =
      (cell_type[vx_index_01] != grid_cell_t::CELL_AIR || cell_type[vx_index_01 - vx_offset] != grid_cell_t::CELL_AIR)
          ? 1.0f
          : 0.0f;

  // sum of weights from valid samples only
  // we need this because invalid samples are excluded, so weights no longer sum to 1
  float valid_weight_sum_vx =
      valid_vx_00 * vx_weight1 + valid_vx_10 * vx_weight2 + valid_vx_11 * vx_weight3 + valid_vx_01 * vx_weight4;

  // only update particle if we have valid velocity data to interpolate from
  if (valid_weight_sum_vx > 0.0f) {
    // pic velocity = weighted average of current grid velocities
    // each term is multiplied by validity (0 or 1) to exclude air-only samples
    // divide by valid weight sum to get proper weighted average
    float pic_vx = (valid_vx_00 * vx_weight1 * grid_vx[vx_index_00] + valid_vx_10 * vx_weight2 * grid_vx[vx_index_10] +
                    valid_vx_11 * vx_weight3 * grid_vx[vx_index_11] + valid_vx_01 * vx_weight4 * grid_vx[vx_index_01]) /
                   valid_weight_sum_vx;

    // flip correction = weighted average of velocity CHANGE
    // same validity filtering applied here
    float corr_vx = (valid_vx_00 * vx_weight1 * (grid_vx[vx_index_00] - grid_previous_vx[vx_index_00]) +
                     valid_vx_10 * vx_weight2 * (grid_vx[vx_index_10] - grid_previous_vx[vx_index_10]) +
                     valid_vx_11 * vx_weight3 * (grid_vx[vx_index_11] - grid_previous_vx[vx_index_11]) +
                     valid_vx_01 * vx_weight4 * (grid_vx[vx_index_01] - grid_previous_vx[vx_index_01])) /
                    valid_weight_sum_vx;

    // flip velocity = current particle velocity + grid velocity change
    float flip_vx = particle->vx + corr_vx;

    // blend pic and flip based on ratio
    float new_vx = (1.0f - FLIP_RATIO) * pic_vx + FLIP_RATIO * flip_vx;
    particle->vx = new_vx;
  }
  // if no valid samples, particle keeps its current vx unchanged

  //***** vy transfer from grid to particle *****

  // determine offsets based on which velocity component we are interpolating
  // particle positions are shifted since velocities are in the center of sides of cell
  // doing this we transform from the cells grid to the grid formed by
  // the points where the vy velocities are (center of the horizontal sides of the cells)
  // for vy: offset_x = half_cell, offset_y = 0

  // shift particle position into the velocity components coordinate system
  float vy_particle_pos_x = px - half_cell;
  float vy_particle_pos_y = py;

  // clamp particle position to safe range (one cell away from boundaries)
  vy_particle_pos_x = utils::clamp(vy_particle_pos_x, min_pos, max_pos_x);
  vy_particle_pos_y = utils::clamp(vy_particle_pos_y, min_pos, max_pos_y);

  // determine which cell (within the staggered velocity grid)
  int vy_velocities_grid_cell_i = (int)(vy_particle_pos_x * inv_cell_size);
  int vy_velocities_grid_cell_j = (int)(vy_particle_pos_y * inv_cell_size);

  // clamp cell indices so +1 never exceeds bounds
  vy_velocities_grid_cell_i = utils::clamp(vy_velocities_grid_cell_i, 0, max_cell_i);
  vy_velocities_grid_cell_j = utils::clamp(vy_velocities_grid_cell_j, 0, max_cell_j);

  // calculate where inside the cell the particle is
  float vy_delta_x = vy_particle_pos_x - vy_velocities_grid_cell_i * cell_size;
  float vy_delta_y = vy_particle_pos_y - vy_velocities_grid_cell_j * cell_size;

  // precompute normalized weights to avoid redundant calculations
  float vy_tx = vy_delta_x * inv_cell_size;
  float vy_ty = vy_delta_y * inv_cell_size;
  float vy_sx = 1.0f - vy_tx;
  float vy_sy = 1.0f - vy_ty;

  // calculate weights
  float vy_weight1 = vy_sx * vy_sy;  // weight for bottom left
  float vy_weight2 = vy_tx * vy_sy;  // weight for bottom right
  float vy_weight3 = vy_tx * vy_ty;  // weight for top right
  float vy_weight4 = vy_sx * vy_ty;  // weight for top left

  // find the index of the four grid corners in the 1d array
  int vy_base_index = vy_velocities_grid_cell_i * size_y + vy_velocities_grid_cell_j;
  int vy_index_00 = vy_base_index;               // bottom left
  int vy_index_10 = vy_base_index + size_y;      // bottom right
  int vy_index_11 = vy_base_index + size_y + 1;  // top right
  int vy_index_01 = vy_base_index + 1;           // top left

  // offset to find the neighboring cell that shares this velocity
  // v velocities sit on horizontal faces between cell (i,j-1) and cell (i,j)
  // in column-major layout, moving down one cell means subtracting 1
  int vy_offset = 1;

  // check if each velocity sample is valid
  float valid_vy_00 =
      (cell_type[vy_index_00] != grid_cell_t::CELL_AIR || cell_type[vy_index_00 - vy_offset] != grid_cell_t::CELL_AIR)
          ? 1.0f
          : 0.0f;
  float valid_vy_10 =
      (cell_type[vy_index_10] != grid_cell_t::CELL_AIR || cell_type[vy_index_10 - vy_offset] != grid_cell_t::CELL_AIR)
          ? 1.0f
          : 0.0f;
  float valid_vy_11 =
      (cell_type[vy_index_11] != grid_cell_t::CELL_AIR || cell_type[vy_index_11 - vy_offset] != grid_cell_t::CELL_AIR)
          ? 1.0f
          : 0.0f;
  float valid_vy_01 =
      (cell_type[vy_index_01] != grid_cell_t::CELL_AIR || cell_type[vy_index_01 - vy_offset] != grid_cell_t::CELL_AIR)
          ? 1.0f
          : 0.0f;

  // sum of weights from valid samples only
  float valid_weight_sum_vy =
      valid_vy_00 * vy_weight1 + valid_vy_10 * vy_weight2 + valid_vy_11 * vy_weight3 + valid_vy_01 * vy_weight4;

  // only update particle if we have valid velocity data to interpolate from
  if (valid_weight_sum_vy > 0.0f) {
    // pic velocity = weighted average of current grid velocities
    float pic_vy = (valid_vy_00 * vy_weight1 * grid_vy[vy_index_00] + valid_vy_10 * vy_weight2 * grid_vy[vy_index_10] +
                    valid_vy_11 * vy_weight3 * grid_vy[vy_index_11] + valid_vy_01 * vy_weight4 * grid_vy[vy_index_01]) /
                   valid_weight_sum_vy;

    // flip correction = weighted average of velocity CHANGE
    float corr_vy = (valid_vy_00 * vy_weight1 * (grid_vy[vy_index_00] - grid_previous_vy[vy_index_00]) +
                     valid_vy_10 * vy_weight2 * (grid_vy[vy_index_10] - grid_previous_vy[vy_index_10]) +
                     valid_vy_11 * vy_weight3 * (grid_vy[vy_index_11] - grid_previous_vy[vy_index_11]) +
                     valid_vy_01 * vy_weight4 * (grid_vy[vy_index_01] - grid_previous_vy[vy_index_01])) /
                    valid_weight_sum_vy;

    // flip velocity = current particle velocity + grid velocity change
    float flip_vy = particle->vy + corr_vy;

    // blend pic and flip based on ratio
    float new_vy = (1.0f - FLIP_RATIO) * pic_vy + FLIP_RATIO * flip_vy;
    particle->vy = new_vy;
  }
  // if no valid samples, particle keeps its current vy unchanged
}

void Grid::handleParticleCollision(Particle* particle) {
  // boundaries: one cell inward from walls
  float min_x = cell_size + particle_radius;
  float max_x = (size_x - 1) * cell_size - particle_radius;
  float min_y = cell_size + particle_radius;
  float max_y = (size_y - 1) * cell_size - particle_radius;

  float x = particle->x_pos;
  float y = particle->y_pos;
  float vx = particle->vx;
  float vy = particle->vy;

  // bounce off the wall with restitution * velocity
  float restitution = 0.0f;
  //friction that slows particles in direction of wall
  float friction_factor = 0.85f;

  // check left wall
  if (x < min_x) {
    x = min_x;
    vx = -vx * restitution;  // Bounce instead of stick
    vy = vy * friction_factor; // Apply friction to sliding
  }
  // check right wall
  if (x > max_x) {
    x = max_x;
    vx = -vx * restitution;
    vy = vy * friction_factor;
  }
  // check bottom wall
  if (y < min_y) {
    y = min_y;
    vy = -vy * restitution;
    vx = vx * friction_factor;
  }
  // check top wall
  if (y > max_y) {
    y = max_y;
    vy = -vy * restitution;
    vx = vx * friction_factor;
  }

  // update particle (you'll need position setters)
  particle->x_pos = (x);
  particle->y_pos = (y);
  particle->vx = (vx);
  particle->vy = (vy);
}

void Grid::initParticleSpatialHash(int max_particles, float p_radius) {
  // the spatial hash grid is separate from the velocity grid
  // it uses spacing of 2.2 * particle_radius so particles that could
  // potentially collide (distance < 2 * radius) are in same or adjacent cells

  particle_radius = p_radius;
  p_inv_spacing = 1.0f / (2.2f * particle_radius);

  // calculate dimensions of particle grid based on physical size
  p_num_x = (int)(PHYSICAL_WIDTH * p_inv_spacing) + 1;
  p_num_y = (int)(PHYSICAL_HEIGHT * p_inv_spacing) + 1;
  int p_num_cells = p_num_x * p_num_y;

  // allocate arrays for spatial hashing
  num_cell_particles = new int[p_num_cells];
  first_cell_particle = new int[p_num_cells + 1];  // +1 for guard element
  cell_particle_ids = new int[max_particles];

  // allocate particle density array (same size as velocity grid)
  particle_density = new float[size_x * size_y];
  particle_rest_density = 0.0f;
}

void Grid::pushParticlesApart(Particle* particles, int num_particles, int num_iterations) {
  // this function prevents particles from clumping together
  // it uses spatial hashing to efficiently find nearby particles
  // then pushes apart any pair closer than 2 * particle_radius

  int p_num_cells = p_num_x * p_num_y;

  // ----- step 1: count particles per cell -----

  for (int i = 0; i < p_num_cells; i++) {
    num_cell_particles[i] = 0;
  }

  for (int i = 0; i < num_particles; i++) {
    float x = particles[i].x_pos;
    float y = particles[i].y_pos;

    // find which cell this particle belongs to in the spatial hash grid
    int xi = utils::clamp((int)(x * p_inv_spacing), 0, p_num_x - 1);
    int yi = utils::clamp((int)(y * p_inv_spacing), 0, p_num_y - 1);
    int cell_nr = xi * p_num_y + yi;

    num_cell_particles[cell_nr]++;
  }

  // ----- step 2: compute partial sums to get starting indices -----

  // after this, first_cell_particle[i] will point to where cell i's
  // particles END in the cell_particle_ids array (we fill backwards)
  int first = 0;
  for (int i = 0; i < p_num_cells; i++) {
    first += num_cell_particles[i];
    first_cell_particle[i] = first;
  }
  first_cell_particle[p_num_cells] = first;  // guard element

  // ----- step 3: fill particles into cells (backwards) -----

  for (int i = 0; i < num_particles; i++) {
    float x = particles[i].x_pos;
    float y = particles[i].y_pos;

    int xi = utils::clamp((int)(x * p_inv_spacing), 0, p_num_x - 1);
    int yi = utils::clamp((int)(y * p_inv_spacing), 0, p_num_y - 1);
    int cell_nr = xi * p_num_y + yi;

    // decrement first pointer and store particle id there
    first_cell_particle[cell_nr]--;
    cell_particle_ids[first_cell_particle[cell_nr]] = i;
  }

  // ----- step 4: push particles apart -----

  float min_dist = 2.0f * particle_radius;
  float min_dist_squared = min_dist * min_dist;

  for (int iter = 0; iter < num_iterations; iter++) {
    for (int i = 0; i < num_particles; i++) {
      float px = particles[i].x_pos;
      float py = particles[i].y_pos;

      // find which cell this particle is in
      int pxi = (int)(px * p_inv_spacing);
      int pyi = (int)(py * p_inv_spacing);

      // check this cell and all 8 neighbors (3x3 region)
      int x0 = (pxi - 1 > 0) ? pxi - 1 : 0;
      int y0 = (pyi - 1 > 0) ? pyi - 1 : 0;
      int x1 = (pxi + 1 < p_num_x - 1) ? pxi + 1 : p_num_x - 1;
      int y1 = (pyi + 1 < p_num_y - 1) ? pyi + 1 : p_num_y - 1;

      for (int xi = x0; xi <= x1; xi++) {
        for (int yi = y0; yi <= y1; yi++) {
          int cell_nr = xi * p_num_y + yi;

          // iterate over all particles in this cell
          int first_idx = first_cell_particle[cell_nr];
          int last_idx = first_cell_particle[cell_nr + 1];

          for (int j = first_idx; j < last_idx; j++) {
            int id = cell_particle_ids[j];

            // don't compare particle with itself
            if (id == i) continue;
            // symmetric check: skip if we've already handled this pair
            // when we processed particle 'id' we already pushed it away from 'i'
            // t his effectively halves the number of expensive distance checks
            // wqqif (id <= i) continue;

            float qx = particles[id].x_pos;
            float qy = particles[id].y_pos;

            // vector from particle i to particle id
            float dx = qx - px;
            float dy = qy - py;
            float dist_squared = dx * dx + dy * dy;

            // skip if too far apart or exactly overlapping
            if (dist_squared > min_dist_squared || dist_squared == 0.0f) continue;

            // compute actual distance and push amount
            // use quakeii fast inverse sqrt to avoid expensive sqrt and division
            float invDist = utils::fastInvSqrt(dist_squared);
            float push = 0.5f * (min_dist * invDist - 1.0f);

            // scale the direction vector by push amount
            dx *= push;
            dy *= push;

            // push particles apart (each moves half the overlap distance)
            particles[i].x_pos = (px - dx);
            particles[i].y_pos = (py - dy);
            particles[id].x_pos = (qx + dx);
            particles[id].y_pos = (qy + dy);

            // update px, py since particle i moved
            px = particles[i].x_pos;
            py = particles[i].y_pos;
          }
        }
      }
    }
  }
}

void Grid::updateParticleDensity(Particle* particles, int num_particles) {
  // this function computes how many particles are in each cell
  // using bilinear interpolation (same as velocity transfer)
  // the result is used for drift compensation in incompressibility solver

  int n = size_y;
  float h = cell_size;
  float h1 = 1.0f * inv_cell_size;
  float h2 = 0.5f * cell_size;  // half cell size for centering

  // clear density array
  for (int i = 0; i < size_x * size_y; i++) {
    particle_density[i] = 0.0f;
  }

  // scatter particle contributions to grid using bilinear weights
  for (int i = 0; i < num_particles; i++) {
    float x = particles[i].x_pos;
    float y = particles[i].y_pos;

    // clamp position to valid range
    x = utils::clamp(x, h, (size_x - 1) * h);
    y = utils::clamp(y, h, (size_y - 1) * h);

    // find cell indices (offset by h2 because density is at cell centers)
    int x0 = (int)((x - h2) * h1);
    float tx = ((x - h2) - x0 * h) * h1;  // fractional position in cell
    int x1 = (x0 + 1 < size_x - 2) ? x0 + 1 : size_x - 2;

    int y0 = (int)((y - h2) * h1);
    float ty = ((y - h2) - y0 * h) * h1;
    int y1 = (y0 + 1 < size_y - 2) ? y0 + 1 : size_y - 2;

    // bilinear weights
    float sx = 1.0f - tx;
    float sy = 1.0f - ty;

    // add weighted contribution to four surrounding cells
    if (x0 < size_x && y0 < size_y) particle_density[x0 * n + y0] += sx * sy;
    if (x1 < size_x && y0 < size_y) particle_density[x1 * n + y0] += tx * sy;
    if (x1 < size_x && y1 < size_y) particle_density[x1 * n + y1] += tx * ty;
    if (x0 < size_x && y1 < size_y) particle_density[x0 * n + y1] += sx * ty;
  }

  // compute rest density on first call (when it's still zero)
  // this is the average density across all fluid cells
  if (particle_rest_density == 0.0f) {
    float sum = 0.0f;
    int num_fluid_cells = 0;

    for (int i = 0; i < size_x * size_y; i++) {
      if (cell_type[i] == grid_cell_t::CELL_LIQUID) {
        sum += particle_density[i];
        num_fluid_cells++;
      }
    }

    if (num_fluid_cells > 0) {
      particle_rest_density = sum / num_fluid_cells;
    }
  }
}