# esp32-s3-flip-fluid-simulation
FLIP (+ PIC) fluid simulation on ESP32-S3 with 8x8 LED matrix display and accelerometer-controlled gravity.

Main objective is to implement a particle-based fluid dynamics simulation using the FLIP method on embedded hardware, exploring the challenges of real-time physics calculations with limited computational resources while creating an interactive, visually appealing display.

## Hardware Components
- Waveshare ESP32-S3-Matrix Development Board with 8x8 led matrix and QMI8658 6-Axis IMU (Accelerometer + Gyroscope) integrated
- Momentary Push Button
- Small switch
- TP4057 1A
- 220mAh Lipo pouch battery
<img width="650" height="310" alt="hardwareComponents" src="https://github.com/user-attachments/assets/90149b72-0045-4879-8765-851080bced2d" />

## Assembly

### Solder de components together using AWG30 wires
<img width="478" height="350" alt="guts" src="https://github.com/user-attachments/assets/23cf3dd8-04e4-4dfe-a456-4fdc2e495380" />

### Design and print the casing
<img width="482" height="350" alt="casing" src="https://github.com/user-attachments/assets/28c9bfaa-b1ef-40e2-9b1b-5ab7b9efb6e8" />

### Place the components in casing
<img width="350" height="455" alt="gutsincasing" src="https://github.com/user-attachments/assets/609cc651-fc70-4232-86ad-caa702ecf82c" />


### Finished product
<img width="1135" height="1500" alt="finished2" src="https://github.com/user-attachments/assets/a118e2cc-514e-43a4-b093-3f5e72528e4d" />


## Pinout Configuration

### NeoPixel LED Matrix
```cpp
LED_PIN = GPIO 14   // WS2812B data line
NUM_LEDS = 64       // 8x8 matrix
```

### QMI8658 IMU (I2C)
```cpp
SDA_PIN = GPIO 11   // I2C data
SCL_PIN = GPIO 12   // I2C clock
QMI8658_ADDR = 0x6B // I2C device address
I2C_FREQ = 400kHz   // fast mode
```

### Control Button
```cpp
BUTTON_PIN = GPIO 5  // user input (with internal pullup)
```

Button is active low (pressed = LOW, released = HIGH)

## Physical Setup
The simulation maps a simulated physical container of 500mm500mm to the 8x8 LED grid. The LED matrix serves as a low-resolution display. The QMI8658 accelerometer continuously updates the gravity direction, allowing the fluid to respond naturally to device orientation.

## Key Features

### FLIP Fluid Simulation

(* means configurable as parameter in config.h)
Particle-In-Cell hybrid method combining Lagrangian particles with Eulerian grid:
- **850 Particles:** hexagonally packed for optimal density, radius = 0.3 * cell_size(*)
- **32x32 Simulation Grid:** provides velocity field for incompressibility solving(*)
- **FLIP/PIC Blending:** 90% FLIP + 10% PIC for stability while preserving momentum(*)
- **Frame Interval:** 23ms real-time (43.5 fps target)(*)
- **Physics Timestep:** 27.6ms simulated time per frame (1.2x speed multiplier snappier fluid motion)(*)
- **Gravity Magnitude:** 9.81 m/s^2, direction updated from accelerometer each frame(*)

### Incompressibility Solver
Iterative pressure projection using Gauss-Seidel with successive over-relaxation:
- **30 Iterations:** balances accuracy vs. computation time (~9.4ms solve time)(*)
- **Over-Relaxation Factor:** 1.9x to accelerate convergence(*)
- **Density Correction:** compensates for particle drift using K-factor = 1.0(configurable)(*)
- **Grid Classification:** cells marked as LIQUID, AIR, or WALL based on particle occupancy

### Particle Dynamics Pipeline
Nine-stage FLIP cycle executed each frame:
1. **Integration:** apply gravity, update positions based on velocity
2. **Separation:** push overlapping particles apart (2 iterations, spatial hash grid)
3. **Collision:** clamp particles inside boundaries with restitution/friction
4. **Grid Preparation:** reset cell types, mark liquid cells, save previous velocities
5. **Particle → Grid Transfer:** scatter particle velocities to grid using bilinear weights
6. **Normalization:** divide accumulated velocities by weights, restore wall velocities
7. **Density Calculation:** compute particle density per cell for drift compensation
8. **Incompressibility Solve:** iteratively adjust velocities to minimize divergence
9. **Grid → Particle Transfer:** interpolate corrected velocities back to particles with FLIP/PIC blending

### Spatial Hash Grid
Efficient particle separation using uniform grid acceleration structure:
- **Cell Spacing:** 2.2 × particle_radius for optimal bucket size
- **Initial Packing:** particles initialized at 1.8 × radius spacing (tighter than 2.0) to increase rest density
- **Collision Detection:** only checks particles in same/adjacent cells (9 cells total)
- **Separation Force:** pushes particles to minimum separation distance
- **Memory Layout:** flat arrays (num_cell_particles, first_cell_particle, cell_particle_ids)
- Significantly faster than O(n²) all-pairs check for 850 particles

### Bilinear Interpolation
Smooth velocity transfers between particles and staggered MAC grid:
- **Staggered Grid:** vx stored on vertical faces, vy on horizontal faces
- **4-Point Weights:** precomputed for each particle based on position
- **Sub-Cell Accuracy:** particles can be anywhere within cell, not just at centers
- **Weight Accumulation:** prevents velocity discontinuities at cell boundaries
- Computed once per particle per transfer, reused for both velocity components

### Button Control System
State machine with debouncing for three interaction types:
- **Single Tap (< 600ms):** cycle through 7 brightness levels (3-13 intensity)(*)
- **Double Tap (< 250ms between):** cycle through 17 color presets (blues, greens, reds, yellows, purples, pinks, white)(*)
- **Long Press (≥ 600ms):** cycle through 5 foam threshold levels (2-30 particle count)(*)
- **Debouncing:** 25ms settling time eliminates mechanical noise(*)
- **NVS Persistence:** settings saved to flash immediately after change, restored on boot

### Color Presets
17 carefully tuned color combinations optimized for LED visibility:
- **Blues/Cyans (4):** pure blue, cyan, sky blue, aqua
- **Greens (2):** lime green, yellow-green
- **Reds (2):** pure red, crimson
- **Oranges/Yellows (3):** orange, golden yellow, amber
- **Purples (3):** violet, lavender, deep purple
- **Pinks/Magentas (2):** hot pink, magenta
- **White (1):** full spectrum white

Each preset is scaled by current brightness setting (0-255 range)

### Visualization
Particle density mapping to LED colors with foam effect:
- **Liquid Rendering:** cells with >5 particles show full color at current brightness(*)
- **Foam Effect:** cells with 2-5 particles display white at 30% brightness(*)
- **Brightness Scaling:** base color RGB values multiplied by brightness factor (3-13)(*)
- **Update Rate:** ~43 fps theoretical max, ~30-35 fps typical with 23ms physics timestep
- Physical-to-pixel mapping: particle position (meters) → LED coordinates (0-7)

### Accelerometer Integration
Real-time gravity direction from QMI8658 6-axis IMU:
- **Sensor Mode:** accelerometer only (gyroscope disabled for efficiency)
- **Update Frequency:** every frame (~43 Hz)
- **Range:** ±4g with 8192 LSB/g sensitivity
- **Axis Mapping:** device Y-axis → simulation X (horizontal), device X-axis → simulation Y (vertical)
- **Magnitude:** constant 9.81 m/s², direction varies with tilt
- **Response:** fluid immediately reacts to device orientation changes
- **Calibration:** automatic via sensor hardware, no user calibration needed

## Technical Specifications

### Performance
- **Simulation Grid:** 32×32 cells (1024 total)
- **Display Grid:** 8×8 LEDs (64 total, 1:4 downsampling)
- **Particle Count:** 850 active particles
- **Frame Interval:** 23ms target (43.5 fps)
- **Actual Frame Rate:** 43-46 fps typical
- **Frame Breakdown:**
  - Integration: ~0.05ms
  - Separation: ~4.9ms
  - Collision: ~0.21ms
  - Grid Prep: ~0.51ms
  - P→G Transfer: ~1.41ms
  - Density: ~0.53ms
  - Solver: ~9.4ms
  - G→P Transfer: ~2.41ms
  - Visualization: ~2.51ms
  - **Total:** ~22ms per frame

### Memory Usage
- **Particle Array:** 850 · 16 bytes = 13.6 KB
- **Grid Velocities:** 2 · 1024 · 4 bytes = 8 KB (current + previous)
- **Weight Accumulators:** 2 · 1024 · 4 bytes = 8 KB
- **Cell Types:** 1024 ·1 byte = 1 KB
- **Spatial Hash:** ~10 KB (particle bucketing)
- **Density Array:** 1024 · 4 bytes = 4 KB
- **Total:** ~45 KB dynamic allocation

### Compiler Optimizations
Aggressive optimization flags for maximum performance:
- **-O3:** maximum optimization level
- **-ffast-math:** aggressive floating-point optimizations (IEEE compliance relaxed)
- **-funroll-loops:** unroll loops for better pipelining
- **-fomit-frame-pointer:** free up register for computations
- **CPU Frequency:** 240 MHz (maximum ESP32-S3 speed)
- **Flash Frequency:** 80 MHz for fast code fetching

## Algorithm Details

Implemented following Ten Minute Physics tutorial closely (https://www.youtube.com/watch?v=XmzBREkK8kY)

### FLIP Method
- **Particles:** carry velocity and position, no pressure field
- **Grid:** solves for incompressibility, enforces boundary conditions
- **Transfer P→G:** particles scatter velocities to grid with bilinear interpolation
- **Solve Pressure:** grid velocities adjusted to satisfy â‹…u = 0
- **Transfer G→P:** particles updated with FLIP/PIC blend:
  - **PIC (10%):** vnew = vgrid (dissipative, stable)
  - **FLIP (90%):** vnew = vold + (vgrid - vgrid_old) (preserves momentum)

### Pressure Projection
- **Divergence Calculation:** sum of velocity flux through cell faces
- **Pressure Gradient:** distributes correction to neighboring cells
- **Over-Relaxation:** scales correction by 1.9x to accelerate convergence
- **Boundary Conditions:** solid walls have zero normal velocity
- **Density Compensation:** adds drift correction term based on particle deficit/surplus

### Particle Collision
Handles boundaries with configurable restitution and friction:
- **Wall Detection:** clamp position to [particle_radius, domain_size - particle_radius]
- **Restitution Factor:** 0.2 (20% velocity preserved on bounce)(*)
- **Friction Factor:** 0.2 (20% tangential velocity preserved)(*)
- **Clamping:** prevents particles from penetrating walls
- Applied after position update, before grid transfer

## Build Environment

### PlatformIO Configuration
```ini
[env:esp32-s3-matrix]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino

board_build.flash_mode = dio
board_build.flash_size = 4MB
board_build.f_cpu = 240000000L

build_flags = 
    -DARDUINO_USB_CDC_ON_BOOT=1
    -O3
    -ffast-math
    -funroll-loops
    -fomit-frame-pointer
    -DCORE_DEBUG_LEVEL=0
    
monitor_speed = 115200
upload_speed = 921600

lib_deps = 
    adafruit/Adafruit NeoPixel@^1.12.0
```

Upload with default PlatformIO settings - no special bootloader or partition table required.

## Configuration Options

All simulation parameters can be adjusted in `config.h`:

### Physics Constants
```cpp
GRAVITY_MAGNITUDE = 9.81f          // m/s²
FRAME_INTERVAL = 0.023f            // base timestep
SPEED_MULTIPLIER = 1.2f            // time acceleration
FLIP_RATIO = 0.9f                  // FLIP vs PIC blend
INCOMPRESSIBILITY_ITERATIONS = 30  // solver iterations
OVERRELAXATION = 1.9f              // convergence acceleration
```

### Particle Settings
```cpp
NUM_PARTICLES = 850                // total particle count
RESTITUTION_FACTOR = 0.2f          // bounce coefficient
FRICTION_FACTOR = 0.2f             // wall friction
```

### Grid Resolution
```cpp
GRID_SIZE_X = 32                   // simulation cells
GRID_SIZE_Y = 32
PHYSICAL_WIDTH = 0.5f              // meters
PHYSICAL_HEIGHT = 0.5f
```

### Visualization
```cpp
PARTICLE_THRESHOLD = 5.0f          // min particles for liquid color
PARTICLE_THRESHOLD_FOAM = 5.0f     // foam threshold (overridden by button)
```

## Usage

### First Boot
1. Device initializes with default settings (blue color, low brightness, low foam)
2. Particles appear in block formation at bottom of display
3. Tilt device to control gravity direction - fluid responds immediately
### Test


https://github.com/user-attachments/assets/9a86d0c9-9984-4224-b226-0375ce9b3e63



### Button Controls
- **Single Tap:** cycle brightness (3 → 4 → 5 → 6 → 9 → 11 → 13 → 3...)
- **Double Tap:** cycle colors (blue → cyan → green → red → orange → purple → pink → white → blue...)
- **Long Press:** cycle foam threshold (none → low → medium → high → max → none...)
### Button funcionality test


https://github.com/user-attachments/assets/5af7af76-c64d-4482-b536-27835820dfac



### Settings Persistence
All button settings automatically save to flash and restore on next power-up.


## Credit

- MAtthias Müller´s Ten Minute Physics: "18 - How to write a FLIP water / fluid simulation running in your browser"
- https://www.youtube.com/watch?v=XmzBREkK8kY

## License
MIT License - feel free to use, modify, and distribute.

