#include "config.h"
#include "grid.h"
#include "particle.h"
#include "utils.h"

// global instances
Adafruit_NeoPixel matrix = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_RGB + NEO_KHZ800);
Grid grid;                          // single grid instance for the simulation
Particle particles[NUM_PARTICLES];  // array of all particles

// timing control for fixed timestep
unsigned long lastStepTime = 0;
// const unsigned long stepInterval = (unsigned long)(DELTA_T * 1000.0f); // Moved to loop for consistency

// gravity vector (updated from accelerometer each frame)
float gravityX = 0.0f;
float gravityY = -GRAVITY_MAGNITUDE;  // default: pointing down

// particle radius - calculated from cell size like reference code
// reference uses: r = 0.3 * h where h is cell_size
float particleRadius = 0.0f;

// --- Profiling Variables (Time in milliseconds) ---
float t_integrate = 0;  // Movement & Gravity
float t_push = 0;       // Particle Separation (Heavy)
float t_collision = 0;  // Wall handling
float t_grid_prep = 0;  // Resetting/Clearing grid
float t_p2g = 0;        // Particle -> Grid Transfer
float t_density = 0;    // Density calculation
float t_solve = 0;      // Incompressibility (Heavy)
float t_g2p = 0;        // Grid -> Particle Transfer
float t_vis = 0;        // LED Visualization

// function to initialize particles in a block formation
void initializeParticles() {
  // calculate cell size the same way grid does
  // this is needed to properly space particles relative to grid cells
  float cell_size_x = PHYSICAL_WIDTH / GRID_SIZE_X;
  float cell_size_y = PHYSICAL_HEIGHT / GRID_SIZE_Y;
  float cell_size = (cell_size_x > cell_size_y) ? cell_size_x : cell_size_y;

  // particle radius is 0.3 times cell size (matches reference code)
  particleRadius = 0.3f * cell_size;

  // particle spacing: diameter with slight overlap for dense packing
  // reference uses dx = 2.0 * r for horizontal, dy = sqrt(3)/2 * dx for vertical (hexagonal)
  float dx = 2.0f * particleRadius;
  float dy = sqrtf(3.0f) / 2.0f * dx;

  // calculate how many particles fit in each dimension
  // leave padding of one cell from walls on each side
  float available_width = PHYSICAL_WIDTH - 2.0f * cell_size - 2.0f * particleRadius;
  float available_height = PHYSICAL_HEIGHT - 2.0f * cell_size - 2.0f * particleRadius;

  int particles_per_row = (int)(available_width / dx);
  int particles_per_col = (int)(available_height / dy);

  // make sure we don't exceed NUM_PARTICLES
  while (particles_per_row * particles_per_col > NUM_PARTICLES) {
    if (particles_per_row > particles_per_col) {
      particles_per_row--;
    } else {
      particles_per_col--;
    }
  }

  // starting position: one cell in from walls plus particle radius
  float start_x = cell_size + particleRadius;
  float start_y = cell_size + particleRadius;

  int particle_index = 0;
  for (int j = 0; j < particles_per_col && particle_index < NUM_PARTICLES; j++) {
    for (int i = 0; i < particles_per_row && particle_index < NUM_PARTICLES; i++) {
      // hexagonal packing: offset every other row by half dx
      float offset = (j % 2 == 0) ? 0.0f : particleRadius;

      particles[particle_index].x_pos = (start_x + i * dx + offset);
      particles[particle_index].y_pos = (start_y + j * dy);
      particles[particle_index].vx = (0.0f);
      particles[particle_index].vy = (0.0f);
      particle_index++;
    }
  }

  // if we didn't fill all particles, set remaining to a safe position
  // (this shouldn't happen with proper NUM_PARTICLES but just in case)
  while (particle_index < NUM_PARTICLES) {
    particles[particle_index].x_pos = (start_x);
    particles[particle_index].y_pos = (start_y);
    particles[particle_index].vx = (0.0f);
    particles[particle_index].vy = (0.0f);
    particle_index++;
  }

  Serial.print("Particle radius: ");
  Serial.print(particleRadius * 1000.0f, 3);
  Serial.println(" mm");
  Serial.print("Particles arranged: ");
  Serial.print(particles_per_row);
  Serial.print(" x ");
  Serial.println(particles_per_col);
}

// function to map particles to led matrix and display
void visualizeParticles() {
  matrix.clear();

  // create a temporary array to count particles in each LED cell
  // initialize all counts to 0
  int ledCounts[NUM_LEDS] = {0};

  // count particles per LED
  for (int i = 0; i < NUM_PARTICLES; i++) {
    // convert particle position from meters to led pixel coordinates
    int led_x = (int)(particles[i].x_pos / PHYSICAL_WIDTH * 8.0f);
    int led_y = (int)(particles[i].y_pos / PHYSICAL_HEIGHT * 8.0f);

    // ensure we are within grid bounds
    if (led_x >= 0 && led_x < 8 && led_y >= 0 && led_y < 8) {
      int led_index = led_y * 8 + led_x;
      ledCounts[led_index]++;
    }
  }

  // turn on LEDs that meet the threshold
  for (int i = 0; i < NUM_LEDS; i++) {
    if (ledCounts[i] >= PARTICLE_THRESHOLD) {
      // map brightness to density
      // darker blue for low density, bright blue for high density
      /*int brightness = map(ledCounts[i], PARTICLE_THRESHOLD, 20, 50, 255);
      brightness = constrain(brightness, 50, 255);
      matrix.setPixelColor(i, matrix.Color(0, 50, brightness));  // Dynamic brightness*/

      // or simple solid color
      matrix.setPixelColor(i, matrix.Color(40, 60, 255));
    }
  }

  matrix.show();
}

// read accelerometer and update gravity direction
void updateGravityFromSensor() {
  float accelX, accelY, accelZ;
  utils::readQMI8658Accel(&accelX, &accelY, &accelZ);

  // mappings based on your device orientation
  gravityX = -accelY * GRAVITY_MAGNITUDE;
  gravityY = accelX * GRAVITY_MAGNITUDE;
}

// execute one complete flip timestep with profiling
void runFLIPStep() {
  unsigned long t0 = micros();

  // ===== STEP 1: INTEGRATE PARTICLES =====
  // apply gravity and update positions based on velocity
  for (int i = 0; i < NUM_PARTICLES; i++) {
    particles[i].addGravity(gravityX, gravityY);
    particles[i].updatePosition();
  }
  unsigned long t1 = micros();

  // ===== STEP 2: PUSH PARTICLES APART =====
  // prevent particles from clumping together
  grid.pushParticlesApart(particles, NUM_PARTICLES, 2);  // 1 iterations
  unsigned long t2 = micros();

  // ===== STEP 3: HANDLE WALL COLLISIONS =====
  // clamp particles inside boundaries before transferring to grid
  for (int i = 0; i < NUM_PARTICLES; i++) {
    grid.handleParticleCollision(&particles[i]);
  }
  unsigned long t3 = micros();

  // ===== STEP 4-6: CLASSIFY & CLEAR =====
  // reset non-wall cells to air, then mark cells containing particles as liquid
  grid.resetCellTypesToAir();
  for (int i = 0; i < NUM_PARTICLES; i++) {
    grid.markCellWithLiquid(&particles[i]);
  }
  
  // store current grid velocities before modification (used to restoreSolidCellVelocities)
  grid.savePreviousVelocities();
  
  // zero out velocities and weight accumulators
  grid.clearVelocitiesAndWeights();
  unsigned long t4 = micros();

  // ===== STEP 7-9: TRANSFER P->G & NORMALIZE =====
  // each particle scatters its velocity to nearby grid points
  for (int i = 0; i < NUM_PARTICLES; i++) {
    grid.transferVelocityfromParticleToGrid(&particles[i]);
  }
  
  // divide accumulated velocities by accumulated weights
  grid.normalizeGridVelocities();
  
  // restore solid boundary velocities
  grid.restoreSolidCellVelocities();
  unsigned long t5 = micros();

  // ===== STEP 10-11: DENSITY =====
  // compute how many particles are in each cell (using bilinear weights)
  grid.updateParticleDensity(particles, NUM_PARTICLES);
  
  // Save the velocities before forcingIncompressibility so we can calculate the
  // correct change in velocity for the FLIP update.
  grid.savePreviousVelocities();
  unsigned long t6 = micros();

  // ===== STEP 12: SOLVE FOR INCOMPRESSIBILITY =====
  // iteratively adjust velocities to minimize divergence
  grid.forcingIncompressibility();
  unsigned long t7 = micros();

  // ===== STEP 13: TRANSFER VELOCITIES FROM GRID TO PARTICLES =====
  // interpolate corrected grid velocities back to particles
  for (int i = 0; i < NUM_PARTICLES; i++) {
    grid.transferVelocityfromGridToParticle(&particles[i]);
  }
  unsigned long t8 = micros();

  // Convert micros to milliseconds for display
  t_integrate = (t1 - t0) / 1000.0f;
  t_push      = (t2 - t1) / 1000.0f;
  t_collision = (t3 - t2) / 1000.0f;
  t_grid_prep = (t4 - t3) / 1000.0f;
  t_p2g       = (t5 - t4) / 1000.0f;
  t_density   = (t6 - t5) / 1000.0f;
  t_solve     = (t7 - t6) / 1000.0f;
  t_g2p       = (t8 - t7) / 1000.0f;
}

void setup() {
  // initialize serial for debugging
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n================================");
  Serial.println("FLIP Fluid Simulation - ESP32-S3");
  Serial.println("================================\n");

  // initialize led matrix
  matrix.begin();
  matrix.setBrightness(BRIGHTNESS);
  matrix.clear();
  matrix.show();

  // initialize i2c for accelerometer
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // 400khz i2c speed
  delay(100);

  // initialize qmi8658 accelerometer
  Serial.println("Initializing QMI8658 sensor...");
  if (!utils::initQMI8658()) {
    Serial.println("ERROR: QMI8658 initialization failed!");
    // show error pattern on matrix
    while (1) {
      matrix.clear();
      for (int i = 0; i < 8; i++) {
        matrix.setPixelColor(i * 9, matrix.Color(50, 0, 0));  // red diagonal
      }
      matrix.show();
      delay(500);
      matrix.clear();
      matrix.show();
      delay(500);
    }
  }
  Serial.println("QMI8658 initialized successfully!");

  // initialize particles in starting formation
  // this also calculates particleRadius
  Serial.println("Initializing particles...");
  initializeParticles();
  Serial.print("Created ");
  Serial.print(NUM_PARTICLES);
  Serial.println(" particles");

  // initialize spatial hash grid for particle separation
  // this allocates memory for pushParticlesApart() and updateParticleDensity()
  // must be called AFTER particleRadius is calculated in initializeParticles()
  Serial.println("Initializing spatial hash grid...");
  grid.initParticleSpatialHash(NUM_PARTICLES, particleRadius);

  // Initialize grid boundaries (walls) safely after boot
  grid.markCellWalls();

  // print configuration
  Serial.println("\n=== Configuration ===");
  Serial.print("Grid size: ");
  Serial.print(GRID_SIZE_X);
  Serial.print(" x ");
  Serial.println(GRID_SIZE_Y);
  Serial.print("Physical size: ");
  Serial.print(PHYSICAL_WIDTH * 1000.0f);
  Serial.print(" x ");
  Serial.print(PHYSICAL_HEIGHT * 1000.0f);
  Serial.println(" mm");
  Serial.print("Timestep: ");
  Serial.print(DELTA_T * 1000);
  Serial.println(" ms");
  Serial.print("Target FPS: ");
  Serial.println(1.0 / DELTA_T);
  Serial.print("FLIP ratio: ");
  Serial.println(FLIP_RATIO);
  Serial.print("Incompressibility iterations: ");
  Serial.println(INCOMPRESSIBILITY_ITERATIONS);

  Serial.println("\n=== Starting simulation ===");

  // record start time for fixed timestep control
  lastStepTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  const unsigned long stepInterval = (unsigned long)(FRAME_INTERVAL * 1000.0f);
  
  // only run simulation step when enough time has passed for fixed timestep
  if (currentTime - lastStepTime >= stepInterval) {
    lastStepTime = currentTime;

    // read accelerometer and update gravity direction
    updateGravityFromSensor();

    // execute one complete flip simulation step (with profiling internally)
    runFLIPStep();

    // visualize the results on led matrix
    unsigned long t_vis_start = micros();
    visualizeParticles();
    t_vis = (micros() - t_vis_start) / 1000.0f;

    // print debug info every second
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime > 1000) {
      float total_ms = t_integrate + t_push + t_collision + t_grid_prep + t_p2g + t_density + t_solve + t_g2p + t_vis;
      float max_fps = 1000.0f / total_ms; // Theoretical max FPS based on calc time

      Serial.println("\n--- Performance Stats ---");
      Serial.printf("Total Frame Time: %.2f ms | Max FPS: %.1f\n", total_ms, max_fps);
      Serial.println("Breakdown:");
      Serial.printf("  1. Integrate: %.2f ms\n", t_integrate);
      Serial.printf("  2. Separation:%.2f ms\n", t_push);
      Serial.printf("  3. Collision: %.2f ms\n", t_collision);
      Serial.printf("  4. Grid Prep: %.2f ms\n", t_grid_prep);
      Serial.printf("  5. P -> G:    %.2f ms\n", t_p2g);
      Serial.printf("  6. Density:   %.2f ms\n", t_density);
      Serial.printf("  7. Solver:    %.2f ms\n", t_solve);
      Serial.printf("  8. G -> P:    %.2f ms\n", t_g2p);
      Serial.printf("  9. Visualize: %.2f ms\n", t_vis);

      lastPrintTime = currentTime;
    }
  }
}