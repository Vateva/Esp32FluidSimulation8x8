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

//------simulation constants------

// general
#define GRAVITY -9.81f       // gravity in m/s^2
#define DELTA_T 0.025f       // timstep in seconds 1/40s
#define FLIP_RATIO 0.8f      // flip vs pic ratio
#define COMPRENSIBILITY_ITERATIONS 10 //number of comprensibility iterations
#define OVERRELAXATION 1.9f  // overrelaxation factor to speed up convergence

// particles
#define NUM_PARTICLES 64

// grid
#define GRID_SIZE_X 64
#define GRID_SIZE_Y 64

// physical dimensions of led matrix in meters
#define PHYSICAL_WIDTH 0.022f   // 22mm
#define PHYSICAL_HEIGHT 0.022f  // 22mm

#endif
