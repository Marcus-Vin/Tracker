// para usar esse codigo tem que ser no arduino IDE e tem que ter baixado as bibliotecas MPU6050 by eletronic cats e a biblioteca de compatibilidade do esp32 com o arduino IDE, além disso, o tipo de esp selecionado foi o "esp32 dev module"
/*
#include "I2Cdev.h"
#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

// Pin definitions for ESP32
#define I2C_SDA 21
#define I2C_SCL 22
// Uncomment these when you add the hardware
// #define BUZZER_PIN 5    // Pin for connecting a buzzer or alarm
// #define LED_PIN 13      // Status LED pin

// Declare global variables for filtering
float filteredPitch = 0;
float filteredRoll = 0;
float filterFactor = 0.1; // Adjust between 0.01 (very smooth) and 0.5 (less filtering)

// Fall detection parameters - optimized for real-world conditions
#define FALL_THRESHOLD_ANGLE 40.0f      // Angle threshold for fall detection
#define FALL_THRESHOLD_ACCEL 1.2f       // Base acceleration threshold (will be dynamically adjusted)
#define FALL_CONFIRMATION_TIME 250      // Time in ms to confirm fall
#define FALL_RESET_TIME 2000            // Time in ms before resetting fall detection
#define STABLE_ANGLE_THRESHOLD 10.0f    // Max angle considered "stable" (not falling)
#define ANGLE_RATE_THRESHOLD 3.5f       // Rate of change threshold for fall detection

// Dynamic baseline tracking parameters
#define ACCEL_ADAPTATION_RATE 0.005f    // How quickly to adapt to new baseline (smaller = slower)
#define MAX_BASELINE_ACCEL 0.8f         // Maximum value for baseline acceleration

// Fall detection state variables
bool fallDetected = false;
bool fallConfirmed = false;
unsigned long fallStartTime = 0;
unsigned long fallConfirmTime = 0;

// Dynamic baseline variables
float baselineAccel = 0.2f;     // Starting baseline for acceleration (will adapt over time)
bool systemCalibrated = false;  // Flag to indicate initial calibration is complete

namespace nsMPU {
    MPU6050 mpu;
}

bool mpuInit()
{
    Serial.println("DEBUG: Starting MPU initialization");

    // Initialize I2C
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(I2C_SDA, I2C_SCL); // Using ESP32 specific pins
        Wire.setClock(400000);
        Serial.println("DEBUG: Wire initialized with SDA=21, SCL=22");
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
        Serial.println("DEBUG: FastWire initialized");
    #endif

    // Initialize MPU
    Serial.println("DEBUG: Initializing MPU...");
    nsMPU::mpu.initialize();

    // Test connection
    Serial.println("DEBUG: Testing MPU connection...");
    bool connected = nsMPU::mpu.testConnection();
    if (!connected) {
        Serial.println("ERROR: MPU connection failed! Check wiring!");
        return false;
    }
    Serial.println("DEBUG: MPU connection successful");

    // Initialize DMP
    Serial.println("DEBUG: Initializing DMP...");
    uint8_t devStatus = nsMPU::mpu.dmpInitialize();

    // Set offsets through calibration
    Serial.println("DEBUG: Setting offsets...");
    mpuCalibrateOffsets();

    // Check DMP init status
    if (devStatus == 0) {
        Serial.println("DEBUG: DMP initialized successfully");

        // Reset FIFO before enabling DMP
        nsMPU::mpu.resetFIFO();
        delay(100);

        Serial.println("DEBUG: Enabling DMP...");
        nsMPU::mpu.setDMPEnabled(true);
        Serial.println("DEBUG: DMP enabled");

        // Reset FIFO again after enabling DMP
        nsMPU::mpu.resetFIFO();

        return true;
    } else {
        Serial.print("ERROR: DMP initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
        return false;
    }
}

// Scales the measured angle to get a full 90 degrees
float scaleAngle(float measuredAngle, float maxMeasured, float targetMax) {
    // Determine if we're dealing with positive or negative angles
    bool isNegative = (measuredAngle < 0);
    float absAngle = abs(measuredAngle);
    float absMaxMeasured = abs(maxMeasured);
    float absTargetMax = abs(targetMax);

    // Apply scaling only if the angle is significant (> 5 degrees)
    if (absAngle > 5.0f) {
        // Proportional scaling
        float scaledAngle = (absAngle / absMaxMeasured) * absTargetMax;

        // For very large angles, cap at maximum
        if (absAngle > absMaxMeasured * 0.95f) {
            scaledAngle = absTargetMax;
        }

        // Restore the sign
        return isNegative ? -scaledAngle : scaledAngle;
    }

    // For small angles, return as is
    return measuredAngle;
}

bool mpuGetYawPitchRoll(float& y, float& p, float& r, float& accelMagnitude)
{
    uint8_t FIFOBuffer[64];
    Quaternion q;
    VectorFloat gravity;
    VectorInt16 accel;
    VectorInt16 accelReal;

    // Get the FIFO count
    uint16_t fifoCount = nsMPU::mpu.getFIFOCount();

    // Check for overflow (common problem)
    if (fifoCount >= 1024) {
        // Reset FIFO
        Serial.println("FIFO overflow! Resetting...");
        nsMPU::mpu.resetFIFO();
        return false;
    }

    // Wait for correct packet size
    if (fifoCount < 42) {
        return false;  // Not enough data yet
    }

    // Check if packet available
    if (nsMPU::mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
        // Get quaternion and gravity
        nsMPU::mpu.dmpGetQuaternion(&q, FIFOBuffer);
        nsMPU::mpu.dmpGetGravity(&gravity, &q);

        // Calculate angles from gravity vector
        float gx = gravity.x;
        float gy = gravity.y;
        float gz = gravity.z;

        // Calculate roll and pitch
        r = atan2(gy, gz) * 180.0 / M_PI;
        p = atan2(-gx, sqrt(gy * gy + gz * gz)) * 180.0 / M_PI;

        // For yaw, use the quaternion
        y = atan2(2.0f * (q.w * q.z + q.x * q.y),
                 1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * 180.0f / M_PI;

        // Scale angles to get full 90 degrees when rotated
        p = scaleAngle(p, -83.0f, -90.0f);
        r = scaleAngle(r, 83.0f, 90.0f);

        // Get acceleration data
        nsMPU::mpu.dmpGetAccel(&accel, FIFOBuffer);
        nsMPU::mpu.dmpGetLinearAccel(&accelReal, &accel, &gravity);

        // Calculate total acceleration magnitude (in g)
        accelMagnitude = sqrt(accelReal.x*accelReal.x +
                             accelReal.y*accelReal.y +
                             accelReal.z*accelReal.z) / 16384.0f; // 16384 LSB/g for ±2g range

        return true;
    }

    return false;
}

void mpuCalibrateOffsets() {
  int16_t ax, ay, az, gx, gy, gz;
  int32_t ax_sum, ay_sum, az_sum;
  int32_t gx_sum, gy_sum, gz_sum;
  int num_readings = 100;
  int num_iterations = 5;  // More iterations for better results

  // Initial offsets
  int16_t gx_offset = 0;
  int16_t gy_offset = 0;
  int16_t gz_offset = 0;
  int16_t ax_offset = 0;
  int16_t ay_offset = 0;
  int16_t az_offset = 0;

  Serial.println("Enhanced MPU6050 calibration - keep PERFECTLY STILL on a FLAT surface");

  // First calibrate gyro (it's easier)
  Serial.println("Step 1: Calibrating gyroscope...");
  for (int iteration = 0; iteration < num_iterations; iteration++) {
    gx_sum = 0; gy_sum = 0; gz_sum = 0;

    // Apply current offsets
    nsMPU::mpu.setXGyroOffset(gx_offset);
    nsMPU::mpu.setYGyroOffset(gy_offset);
    nsMPU::mpu.setZGyroOffset(gz_offset);
    delay(100);

    // Take readings
    for (int i = 0; i < num_readings; i++) {
      nsMPU::mpu.getRotation(&gx, &gy, &gz);
      gx_sum += gx; gy_sum += gy; gz_sum += gz;
      delay(5);
    }

    // Update offsets
    gx_offset -= gx_sum / num_readings;
    gy_offset -= gy_sum / num_readings;
    gz_offset -= gz_sum / num_readings;

    Serial.print("Gyro iteration ");
    Serial.print(iteration + 1);
    Serial.print(": X=");
    Serial.print(gx_sum / num_readings);
    Serial.print(" Y=");
    Serial.print(gy_sum / num_readings);
    Serial.print(" Z=");
    Serial.println(gz_sum / num_readings);
  }

  // Now calibrate accelerometer
  Serial.println("Step 2: Calibrating accelerometer...");
  for (int iteration = 0; iteration < num_iterations; iteration++) {
    ax_sum = 0; ay_sum = 0; az_sum = 0;

    // Apply current offsets
    nsMPU::mpu.setXAccelOffset(ax_offset);
    nsMPU::mpu.setYAccelOffset(ay_offset);
    nsMPU::mpu.setZAccelOffset(az_offset);
    delay(100);

    // Take readings
    for (int i = 0; i < num_readings; i++) {
      nsMPU::mpu.getAcceleration(&ax, &ay, &az);
      ax_sum += ax; ay_sum += ay; az_sum += az;
      delay(5);
    }

    // Calculate averages
    int16_t ax_avg = ax_sum / num_readings;
    int16_t ay_avg = ay_sum / num_readings;
    int16_t az_avg = az_sum / num_readings;

    // Update offsets - when level, we want ax=0, ay=0, az=16384 (1g)
    ax_offset -= ax_avg / 8;  // Divide by 8 for more gradual adjustment
    ay_offset -= ay_avg / 8;  // Divide by 8 for more gradual adjustment

    // Handle Z axis differently - it should equal 1g (16384)
    int16_t az_ideal = 16384;
    az_offset += (az_ideal - az_avg) / 8;  // Divide by 8 for more gradual adjustment

    Serial.print("Accel iteration ");
    Serial.print(iteration + 1);
    Serial.print(": X=");
    Serial.print(ax_avg);
    Serial.print(" Y=");
    Serial.print(ay_avg);
    Serial.print(" Z=");
    Serial.println(az_avg);
  }

  // Apply final offsets
  nsMPU::mpu.setXGyroOffset(gx_offset);
  nsMPU::mpu.setYGyroOffset(gy_offset);
  nsMPU::mpu.setZGyroOffset(gz_offset);
  nsMPU::mpu.setXAccelOffset(ax_offset);
  nsMPU::mpu.setYAccelOffset(ay_offset);
  nsMPU::mpu.setZAccelOffset(az_offset);

  Serial.println("Calibration complete!");
  Serial.print("Gyro offsets: X=");
  Serial.print(gx_offset);
  Serial.print(" Y=");
  Serial.print(gy_offset);
  Serial.print(" Z=");
  Serial.println(gz_offset);

  Serial.print("Accel offsets: X=");
  Serial.print(ax_offset);
  Serial.print(" Y=");
  Serial.print(ay_offset);
  Serial.print(" Z=");
  Serial.println(az_offset);

  delay(200);
}

// Enhanced fall detection with dynamic baseline adjustment
bool detectFall(float pitch, float roll, float accelMagnitude) {
    static float maxAccel = 0;
    static float maxAngle = 0;
    static float previousAngle = 0;

    // Get maximum angle (from horizontal)
    float currentAngle = max(abs(pitch), abs(roll));

    // Calculate the rate of angle change (degrees per loop iteration)
    float angleChangeRate = currentAngle - previousAngle;
    previousAngle = currentAngle;

    // Update max values during fall detection
    maxAccel = max(maxAccel, accelMagnitude);
    maxAngle = max(maxAngle, currentAngle);

    // Dynamic baseline tracking during normal riding (not when a fall is suspected)
    if (!fallDetected && currentAngle < 20.0f) {
        // Update baseline acceleration only if current acceleration is not too high
        if (accelMagnitude < MAX_BASELINE_ACCEL) {
            baselineAccel = baselineAccel * (1.0f - ACCEL_ADAPTATION_RATE) +
                           accelMagnitude * ACCEL_ADAPTATION_RATE;
        }
    }

    // Calculate excess acceleration beyond the baseline
    float excessAccel = accelMagnitude - baselineAccel;

    // Less frequent debug output to avoid flooding the serial monitor
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint >= 500) {
        Serial.print("DEBUG - Angle: "); Serial.print(currentAngle);
        Serial.print(" | Accel: "); Serial.print(accelMagnitude, 4);
        Serial.print(" | Baseline: "); Serial.print(baselineAccel, 4);
        Serial.print(" | Excess: "); Serial.print(excessAccel, 4);
        Serial.print(" | Rate: "); Serial.println(angleChangeRate);
        lastDebugPrint = millis();
    }

    // First stage fall detection (potential fall)
    if (!fallDetected) {
        // MIXED LOGIC: Both simple cases and combined detection
        // Detect fall if EITHER:
        // 1. Very steep angle (> 60 degrees) - almost certainly a fall
        // 2. Moderate angle (> 40) AND significant excess acceleration or rate
        if (currentAngle > 60.0f ||
            (currentAngle > FALL_THRESHOLD_ANGLE &&
             (excessAccel > FALL_THRESHOLD_ACCEL || angleChangeRate > ANGLE_RATE_THRESHOLD))) {

            fallDetected = true;
            fallStartTime = millis();
            Serial.println("POTENTIAL FALL DETECTED!");
            Serial.print("Angle: "); Serial.print(currentAngle);
            Serial.print(" | Accel: "); Serial.print(accelMagnitude);
            Serial.print(" | Excess: "); Serial.print(excessAccel);
            Serial.print(" | Rate: "); Serial.println(angleChangeRate);

            // Visual indication - uncomment when you add an LED
            // digitalWrite(LED_PIN, HIGH);

            return false; // Not confirmed yet
        }
    }
    // Second stage: confirm fall
    else if (!fallConfirmed) {
        // If we return to stable position quickly, it's not a fall
        if (currentAngle < STABLE_ANGLE_THRESHOLD && (millis() - fallStartTime > 100)) {
            // Reset fall detection
            fallDetected = false;
            maxAccel = 0;
            maxAngle = 0;
            Serial.println("FALSE ALARM - Returned to stable position");

            // Turn off visual indication - uncomment when you add an LED
            // digitalWrite(LED_PIN, LOW);

            return false;
        }

        // Confirm fall after confirmation duration
        if (millis() - fallStartTime > FALL_CONFIRMATION_TIME) {
            fallConfirmed = true;
            fallConfirmTime = millis();
            Serial.println("FALL CONFIRMED!");
            Serial.print("Max Angle: "); Serial.print(maxAngle);
            Serial.print(" | Max Accel: "); Serial.println(maxAccel);

            // Determine fall severity
            String fallSeverity;
            if (maxAngle > 70.0f && maxAccel > 2.0f) {
                fallSeverity = "Severe";
            } else if (maxAngle > 50.0f || maxAccel > 1.5f) {
                fallSeverity = "Moderate";
            } else {
                fallSeverity = "Minor";
            }
            Serial.print("Fall severity: ");
            Serial.println(fallSeverity);

            return true;
        }
    }
    // Reset fall detection after reset duration
    else if (millis() - fallConfirmTime > FALL_RESET_TIME) {
        fallDetected = false;
        fallConfirmed = false;
        maxAccel = 0;
        maxAngle = 0;
        Serial.println("Fall detection reset");

        // Turn off visual indication - uncomment when you add an LED
        // digitalWrite(LED_PIN, LOW);
    }

    return fallConfirmed;
}

void setup() {
    // Initialize serial
    Serial.begin(115200);
    delay(1000); // Give time for serial to initialize

    // Initialize pins - uncomment when you add hardware
    // pinMode(LED_PIN, OUTPUT);
    // pinMode(BUZZER_PIN, OUTPUT);

    // Initial LED blink to show system is starting - uncomment when you add an LED
    // digitalWrite(LED_PIN, HIGH);
    // delay(100);
    // digitalWrite(LED_PIN, LOW);

    Serial.println("\n\n");
    Serial.println("=======================");
    Serial.println("Bicycle Fall Detection System");
    Serial.println("=======================");

    // Initialize MPU
    if (!mpuInit()) {
        Serial.println("Failed to initialize MPU. Halting.");
        // Error pattern - uncomment when you add an LED
        // while (1) {
        //     digitalWrite(LED_PIN, HIGH);
        //     delay(100);
        //     digitalWrite(LED_PIN, LOW);
        //     delay(100);
        // }
        while (1) {
            delay(100); // Just wait if no LED
        }
    }

    // Add a warm-up period
    Serial.println("Warming up... Please wait.");
    for (int i = 5; i > 0; i--) {
        Serial.print(i);
        Serial.println(" seconds remaining...");
        delay(1000);
    }

    // System ready indication - uncomment when you add an LED
    // digitalWrite(LED_PIN, HIGH);
    // delay(500);
    // digitalWrite(LED_PIN, LOW);

    Serial.println("Setup complete. Fall detection active.");
}

void loop() {
    static unsigned long lastPrint = 0;
    static unsigned long noDataCount = 0;
    static unsigned long totalLoops = 0;
    static float lastPitch = 0;
    static float lastRoll = 0;

    float y = 0, p = 0, r = 0, accelMag = 0;

    totalLoops++;

    // Get YPR values and acceleration
    bool gotData = mpuGetYawPitchRoll(y, p, r, accelMag);

    if (gotData) {
        // Apply the spike detection code for erroneous readings
        if (abs(p - lastPitch) > 30) {
            // Likely a transient spike, use the last good value
            p = lastPitch;
        }

        if (abs(r - lastRoll) > 30) {
            r = lastRoll;
        }

        // Store values for next comparison
        lastPitch = p;
        lastRoll = r;

        // Then apply the low-pass filter to smooth the values
        filteredPitch = (p * filterFactor) + (filteredPitch * (1.0 - filterFactor));
        filteredRoll = (r * filterFactor) + (filteredRoll * (1.0 - filterFactor));

        // Check for falls using filtered values for more stability
        bool fall = detectFall(filteredPitch, filteredRoll, accelMag);

        // Take action on fall detection
        if (fall) {
            // Actions when fall is confirmed
            // Uncomment this section when you add a buzzer
            // digitalWrite(BUZZER_PIN, HIGH);
            // delay(1000);
            // digitalWrite(BUZZER_PIN, LOW);

            // Additional code could be added here to send an alert,
            // activate an alarm, send SMS, etc.
        }

        // Print values every 500ms or when fall detected
        if (millis() - lastPrint >= 500 || fall) {
            Serial.print("Y: ");
            Serial.print(y);
            Serial.print(" | P: ");
            Serial.print(filteredPitch);
            Serial.print(" | R: ");
            Serial.print(filteredRoll);
            Serial.print(" | Accel: ");
            Serial.print(accelMag);
            Serial.print("g");

            if (fallDetected && !fallConfirmed) {
                Serial.print(" | POTENTIAL FALL");
            } else if (fallConfirmed) {
                Serial.print(" | FALL CONFIRMED");
            }

            Serial.println();
            lastPrint = millis();
        }
    } else {
        noDataCount++;

        // Only print "no data" message occasionally
        if (millis() - lastPrint >= 2000) {
            Serial.print("No data available (");
            Serial.print((float)noDataCount / totalLoops * 100.0);
            Serial.println("% of readings failed)");
            lastPrint = millis();
        }
    }

    delay(10);  // 10ms delay gives approximately 100Hz update rate
}
  */