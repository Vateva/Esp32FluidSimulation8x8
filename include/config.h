// config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Wire.h>

// LED Matrix Configuration
#define LED_PIN 14
#define NUM_LEDS 64
#define BRIGHTNESS 5

// QMI8658 Configuration
#define QMI8658_ADDR 0x6B
#define SDA_PIN 11
#define SCL_PIN 12

// QMI8658 Registers
#define QMI8658_WHO_AM_I 0x00
#define QMI8658_CTRL1 0x02
#define QMI8658_CTRL2 0x03
#define QMI8658_CTRL3 0x04
#define QMI8658_CTRL7 0x08
#define QMI8658_ACC_X_L 0x35
#define QMI8658_GYR_X_L 0x3B
#define QMI8658_TEMP_L 0x33

//button
#define BUTTON_PIN 5

// timing thresholds in milliseconds
#define DEBOUNCE_TIME 25
#define LONG_PRESS_THRESHOLD 600
#define DOUBLE_TAP_WINDOW 250

// NVS namespace for storing settings
#define NVS_NAMESPACE "fluid_settings"
#define NVS_KEY_COLOR "color"
#define NVS_KEY_BRIGHTNESS "brightness"
#define NVS_KEY_FOAM "foam"

//------simulation constants------

// general
#define GRAVITY_MAGNITUDE 9.81f                      // gravity magnitude in m/s^2
#define FRAME_INTERVAL 0.0195f                        // real time between frames
#define SPEED_MULTIPLIER 1.25f                        // adjust this to control speed
#define DELTA_T (FRAME_INTERVAL * SPEED_MULTIPLIER)  // physics timestep
#define FLIP_RATIO 0.8f                              // flip vs pic ratio
#define INCOMPRESSIBILITY_ITERATIONS 20              // number of incompressibility iterations
#define OVERRELAXATION 1.9f                          // overrelaxation factor to speed up convergence
#define K_FACTOR 1.0f                                // stiffness factort for density correction

// particles
#define NUM_PARTICLES 850  // number of fluid particles
#define RESTITUTION_FACTOR 0.0f//percentaje of velocity perserved when bouncing off a wall
#define FRICTION_FACTOR 0.0f // percentaje of conserved velocity in direction of wall

// grid
#define GRID_SIZE_X 32  // grid cells in x direction
#define GRID_SIZE_Y 32 // grid cells in y direction

// physical dimensions of led matrix in meters
#define PHYSICAL_WIDTH 0.5f
#define PHYSICAL_HEIGHT 0.5f

// visualization
#define PARTICLE_THRESHOLD 5.0f
#define PARTICLE_THRESHOLD_FOAM 5.0f

#endif