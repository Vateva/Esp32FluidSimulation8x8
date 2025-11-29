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

void Grid::transferVelocityfromGridToParticle(Particle* particle) {
  // weights for weighted average of grid velocity to particle
  float weight1, weight2, weight3, weight4;
  // distances from cell boundaries to particle to calculate weights
  float delta_x, delta_y;
  // velocity grid cell coordinates to calculate delta_x and delta_y
  uint16_t velocities_grid_cell_x, velocities_grid_cell_y;

  //*****vx transfer from grid to particle*****

  // shifted particle positions since velocities are in the center of sides of cell
  // doing this we transform from the cells grid to the grid formed by
  // the points where the vx velocities are( center of the vertical sides of the cells)
  float particle_pos_x = particle->getX();                    // we dont shift x position when dealing with vx
  float particle_pos_y = particle->getY() - cell_size * 0.5;  // we only shift y position

  // determine which cell (within the staggered velocity grid)
  velocities_grid_cell_x = particle_pos_x / cell_size;
  velocities_grid_cell_y = particle_pos_y / cell_size;

  // calculate where inside the cell the particle is
  delta_x = particle_pos_x - velocities_grid_cell_x * cell_size;
  delta_y = particle_pos_y - velocities_grid_cell_y * cell_size;

  // calculate weights
  weight1 = (1 - (delta_x / cell_size)) * (1 - (delta_y / cell_size));
  weight2 = (delta_x / cell_size) * (1 - (delta_y / cell_size));
  weight3 = (delta_x / cell_size) * (delta_y / cell_size);
  weight4 = (1 - (delta_x / cell_size)) * (delta_y / cell_size);

  // find the index of the four grid_vx corners in the 1D array
  int index_00 = velocities_grid_cell_x * size_y + velocities_grid_cell_y;              // bottom left
  int index_10 = (velocities_grid_cell_x + 1) * size_y + velocities_grid_cell_y;        // bottom right
  int index_11 = (velocities_grid_cell_x + 1) * size_y + (velocities_grid_cell_y + 1);  // top right
  int index_01 = velocities_grid_cell_x * size_y + (velocities_grid_cell_y + 1);        // top left

  // pic velocity = weighted average of current grid velocities
  float pic_vx = (weight1 * grid_vx[index_00] + weight2 * grid_vx[index_10] + weight3 * grid_vx[index_11] +
                  weight4 * grid_vx[index_01]);

  // flip correction = weighted average of velocity CHANGE
  float corr_vx = (weight1 * (grid_vx[index_00] - grid_previous_vx[index_00]) +
                   weight2 * (grid_vx[index_10] - grid_previous_vx[index_10]) +
                   weight3 * (grid_vx[index_11] - grid_previous_vx[index_11]) +
                   weight4 * (grid_vx[index_01] - grid_previous_vx[index_01]));

  float flip_vx = particle->getVx() + corr_vx;

  float new_vx = (1.0 - FLIP_RATIO) * pic_vx + FLIP_RATIO * flip_vx;
  particle->setVx(new_vx);

  //*****vy transfer from grid to particle*****

  // shifted particle positions since velocities are in the center of sides of cell
  // doing this we transform from the cells grid to the grid formed by
  // the points where the vx velocities are( center of the vertical sides of the cells)
  float particle_pos_x = particle->getX() - cell_size * 0.5;  // we only shift x position
  float particle_pos_y = particle->getY();                    // we dont shift y position when dealing with vx

  // determine which cell (within the staggered velocity grid)
  velocities_grid_cell_x = utils::clamp(velocities_grid_cell_x, 0, size_x - 2);
  velocities_grid_cell_y = utils::clamp(velocities_grid_cell_y, 0, size_y - 2);

  // calculate where inside the cell the particle is
  delta_x = particle_pos_x - velocities_grid_cell_x * cell_size;
  delta_y = particle_pos_y - velocities_grid_cell_y * cell_size;

  // calculate weights
  weight1 = (1 - (delta_x / cell_size)) * (1 - (delta_y / cell_size));
  weight2 = (delta_x / cell_size) * (1 - (delta_y / cell_size));
  weight3 = (delta_x / cell_size) * (delta_y / cell_size);
  weight4 = (1 - (delta_x / cell_size)) * (delta_y / cell_size);

  // find the index of the four grid_vx corners in the 1D array
  int index_00 = velocities_grid_cell_x * size_y + velocities_grid_cell_y;              // bottom left
  int index_10 = (velocities_grid_cell_x + 1) * size_y + velocities_grid_cell_y;        // bottom right
  int index_11 = (velocities_grid_cell_x + 1) * size_y + (velocities_grid_cell_y + 1);  // top right
  int index_01 = velocities_grid_cell_x * size_y + (velocities_grid_cell_y + 1);        // top left

  // pic velocity = weighted average of current grid velocities
  float pic_vy = (weight1 * grid_vy[index_00] + weight2 * grid_vy[index_10] + weight3 * grid_vy[index_11] +
                  weight4 * grid_vy[index_01]);

  // flip correction = weighted average of velocity CHANGE
  float corr_vy = (weight1 * (grid_vy[index_00] - grid_previous_vy[index_00]) +
                   weight2 * (grid_vy[index_10] - grid_previous_vy[index_10]) +
                   weight3 * (grid_vy[index_11] - grid_previous_vy[index_11]) +
                   weight4 * (grid_vy[index_01] - grid_previous_vy[index_01]));

  float flip_vy = particle->getVy() + corr_vy;

  float new_vy = (1.0 - FLIP_RATIO) * pic_vy + FLIP_RATIO * flip_vy;
  particle->setVy(new_vy);
}