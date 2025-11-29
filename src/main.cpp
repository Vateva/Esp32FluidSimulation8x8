#include "config.h"
#include "particle.h"



// Function declarations
bool initQMI8658();
void readQMI8658Accel(float* x, float* y, float* z);
void readQMI8658Gyro(float* x, float* y, float* z);
float readQMI8658Temp();
void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);
void updateParticlePhysics(Particle* p, float gravX, float gravY, float deltaTime);
void drawParticle(const Particle* p);
void drawTrail(const Particle* p);

Adafruit_NeoPixel matrix = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_RGB + NEO_KHZ800);


// Trail effect variables
#define TRAIL_LENGTH 5
float trailX[TRAIL_LENGTH];
float trailY[TRAIL_LENGTH];
int trailIndex = 0;

// Timing variables
unsigned long lastUpdateTime = 0;

void setup() {
  // Minimal serial setup for debugging if needed
  // Serial.begin(115200);
  // delay(1000);
  
  // Serial.println("\n\nESP32-S3-Matrix Gravity Particle - Optimized");
  
  // Initialize LED Matrix
  matrix.begin();
  matrix.setBrightness(BRIGHTNESS);
  matrix.clear();
  matrix.show();
 

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C speed
  delay(100);
  
  // Initialize QMI8658
  if(!initQMI8658()) {
    // Show error pattern on matrix if sensor fails
    while(1) {
      matrix.clear();
      for(int i = 0; i < 8; i++) {
        matrix.setPixelColor(i * 9, matrix.Color(50, 0, 0)); // Red diagonal
      }
      matrix.show();
      delay(500);
      matrix.clear();
      matrix.show();
      delay(500);
    }
  }
  
  // Small delay to let sensor stabilize
  delay(100);
  
  lastUpdateTime = millis();
}

void loop() {
  }