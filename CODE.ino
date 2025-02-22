#include <Wire.h>          // I2C library for MPU-6050
#include <ESP32Servo.h>    // Servo library for ESP32

// Uncomment to enable debugging
#define DEBUG 

// Pin assignment
#define SERVO_FL_PIN 4  // Front left servo pin
#define SERVO_FR_PIN 5  // Front right servo pin
#define SERVO_RL_PIN 18  // Rear left servo pin
#define SERVO_RR_PIN 19  // Rear right servo pin
#define MPU_SDA 21       // I2C SDA pin
#define MPU_SCL 22       // I2C SCL pin

// Servo objects
Servo servoFL;
Servo servoFR;
Servo servoRL;
Servo servoRR;

// MPU6050-related variables
float along = 0, across = 0, vertical = 0;

// Function prototypes
void setupMpu6050();
void readMpu6050Data();

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  delay(3000);  // Wait for serial monitor connection
  Serial.println("Starting ESP32 Active Suspension System...");
#endif

  // Attach servos
  servoFL.attach(SERVO_FL_PIN);
  servoFR.attach(SERVO_FR_PIN);
  servoRL.attach(SERVO_RL_PIN);
  servoRR.attach(SERVO_RR_PIN);

  // Initialize I2C for MPU-6050
  Wire.begin(MPU_SDA, MPU_SCL);
  
  // MPU6050 setup
  setupMpu6050();
}

void loop() {
  // Calculate suspension
  calculateSuspension();

  // Write servo positions
  writeServos();
}

// Setup MPU-6050
void setupMpu6050() {
  Wire.beginTransmission(0x68); // MPU-6050 I2C address
  Wire.write(0x6B);             // Power management register
  Wire.write(0);                // Wake up MPU-6050
  Wire.endTransmission(true);

#ifdef DEBUG
  Serial.println("MPU-6050 initialized");
#endif
}

// Read MPU-6050 data
void readMpu6050Data() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true); // Request accelerometer data (6 bytes)

  int16_t accelX = (Wire.read() << 8 | Wire.read());
  int16_t accelY = (Wire.read() << 8 | Wire.read());
  int16_t accelZ = (Wire.read() << 8 | Wire.read());

  along = accelX / 16384.0;   // Convert to g's
  across = accelY / 16384.0;  // Convert to g's
  vertical = accelZ / 16384.0; // Convert to g's

#ifdef DEBUG
  Serial.print("Accel X: "); Serial.print(along);
  Serial.print(" Y: "); Serial.print(across);
  Serial.print(" Z: "); Serial.println(vertical);
#endif

}

// Calculate suspension
void calculateSuspension() {
  readMpu6050Data();
  // Perform calculations or filtering if needed
}

// Smoothing parameters
const float smoothingFactor = 0.1;   // Low-pass filter weight (adjust for smoother/slower response)
const int transitionSpeed = 2;      // Speed of transition between current and target servo positions

// Store previous servo positions for interpolation
int currentServoFL = 90, currentServoFR = 90, currentServoRL = 90, currentServoRR = 90;

// Low-pass filtered MPU values
float filteredAlong = 0, filteredAcross = 0, filteredVertical = 0;

const float deadZoneThreshold = 0.2; // Ignore small MPU readings (increase to reduce jitter)
const int servoChangeThreshold = 2; // Ignore small servo position changes

// Move servo gradually toward target
int moveTowards(int current, int target, int step) {
  if (current < target) {
    return min(current + step, target);
  } else if (current > target) {
    return max(current - step, target);
  } else {
    return current;
  }
}

// Neutral points for each servo
const int neutralFL = 140;  // Front left servo neutral position
const int neutralFR = 60;  // Front right servo neutral position
const int neutralRL = 140;  // Rear left servo neutral position
const int neutralRR = 30;  // Rear right servo neutral position

void writeServos() {
  const float sensitivityFactor = 2.0; // Amplifies small MPU movements
  const float maxInputRange = 1.5;     // Maximum expected value from MPU
  const int maxServoRange = 60;        // Maximum servo deviation

  // Apply low-pass filter to MPU values
  filteredAlong = (smoothingFactor * along) + ((1 - smoothingFactor) * filteredAlong);
  filteredAcross = (smoothingFactor * across) + ((1 - smoothingFactor) * filteredAcross);
  filteredVertical = (smoothingFactor * vertical) + ((1 - smoothingFactor) * filteredVertical);

  // Apply dead zone: Ignore small changes
  float effectiveAlong = (abs(filteredAlong) > deadZoneThreshold) ? filteredAlong : 0;
  float effectiveAcross = (abs(filteredAcross) > deadZoneThreshold) ? filteredAcross : 0;
  float effectiveVertical = (abs(filteredVertical) > deadZoneThreshold) ? filteredVertical : 0;

  // Normalize and amplify MPU readings
  float normalizedAlong = constrain(effectiveAlong, -maxInputRange, maxInputRange) / maxInputRange;
  float normalizedAcross = constrain(effectiveAcross, -maxInputRange, maxInputRange) / maxInputRange;
  float normalizedVertical = constrain(effectiveVertical, -maxInputRange, maxInputRange) / maxInputRange;

  // Reverse the behavior for left/right tilt (across)
  normalizedAlong = -normalizedAlong;

  // Calculate target positions for servos using individual neutral points
  int targetFL = constrain(neutralFL + normalizedAlong * maxServoRange * sensitivityFactor -
                           normalizedAcross * maxServoRange * sensitivityFactor -
                           normalizedVertical * maxServoRange * sensitivityFactor, 0, 180);
  int targetFR = constrain(neutralFR - normalizedAlong * maxServoRange * sensitivityFactor -
                           normalizedAcross * maxServoRange * sensitivityFactor +
                           normalizedVertical * maxServoRange * sensitivityFactor, 0, 180);
  int targetRL = constrain(neutralRL - normalizedAlong * maxServoRange * sensitivityFactor -
                           normalizedAcross * maxServoRange * sensitivityFactor -
                           normalizedVertical * maxServoRange * sensitivityFactor, 0, 180);
  int targetRR = constrain(neutralRR + normalizedAlong * maxServoRange * sensitivityFactor -
                           normalizedAcross * maxServoRange * sensitivityFactor +
                           normalizedVertical * maxServoRange * sensitivityFactor, 0, 180);

  // Gradual transition to target positions only if change is significant
  if (abs(currentServoFL - targetFL) > servoChangeThreshold) {
    currentServoFL = moveTowards(currentServoFL, targetFL, transitionSpeed);
  }
  if (abs(currentServoFR - targetFR) > servoChangeThreshold) {
    currentServoFR = moveTowards(currentServoFR, targetFR, transitionSpeed);
  }
  if (abs(currentServoRL - targetRL) > servoChangeThreshold) {
    currentServoRL = moveTowards(currentServoRL, targetRL, transitionSpeed);
  }
  if (abs(currentServoRR - targetRR) > servoChangeThreshold) {
    currentServoRR = moveTowards(currentServoRR, targetRR, transitionSpeed);
  }

  // Write smoothed positions to servos
  servoFL.write(currentServoFL);
  servoFR.write(currentServoFR);
  servoRL.write(currentServoRL);
  servoRR.write(currentServoRR);

#ifdef DEBUG
  // Debug output
  Serial.print("FL: "); Serial.print(currentServoFL);
  Serial.print(" FR: "); Serial.print(currentServoFR);
  Serial.print(" RL: "); Serial.print(currentServoRL);
  Serial.print(" RR: "); Serial.println(currentServoRR);
#endif
}
