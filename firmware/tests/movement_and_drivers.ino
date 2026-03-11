// ESP32 S3/S2/C3 Motor Control with Quadrature Encoders (Digital HIGH/LOW Test)
#include <Arduino.h>

// --- Pin Definitions ---
// Motor Driver Pins (DRV8833 Parallel Config)
#define MOTOR_A_IN1 9
#define MOTOR_A_IN2 8
#define MOTOR_B_IN1 11
#define MOTOR_B_IN2 10
#define MOTOR_EEP 12 // Sleep/Enable pin for both drivers

// Encoder Pins
#define ENC_A_PHASE_A 4
#define ENC_A_PHASE_B 5
#define ENC_B_PHASE_A 6
#define ENC_B_PHASE_B 7

// --- Global Variables ---
volatile int32_t countA = 0;
volatile int32_t countB = 0;
uint8_t prevABA = 0;
uint8_t prevABB = 0;

// Quadrature lookup table
static const int8_t QDEC_LUT[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

// Change this to your actual gear motor's total counts per revolution
const float CPR_X4 = 3840.0f; 

// --- Interrupt Service Routines (ISRs) ---
void IRAM_ATTR isrA() {
  uint8_t curr = (digitalRead(ENC_A_PHASE_A) << 1) | digitalRead(ENC_A_PHASE_B);
  countA += QDEC_LUT[(prevABA << 2) | curr];
  prevABA = curr;
}

void IRAM_ATTR isrB() {
  uint8_t curr = (digitalRead(ENC_B_PHASE_A) << 1) | digitalRead(ENC_B_PHASE_B);
  countB += QDEC_LUT[(prevABB << 2) | curr];
  prevABB = curr;
}

// --- Motor Control Helper (Digital HIGH/LOW Test) ---
void setMotorSpeed(int speedA, int speedB) {
  // --- Motor A Control (Normal) ---
  if (speedA > 0) {
    digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);
  } else if (speedA < 0) {
    digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, HIGH);
  } else {
    digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, LOW);
  }

  // --- Motor B Control (INVERTED for physical mirroring) ---
  if (speedB > 0) {
    digitalWrite(MOTOR_B_IN1, LOW); digitalWrite(MOTOR_B_IN2, HIGH);
  } else if (speedB < 0) {
    digitalWrite(MOTOR_B_IN1, HIGH); digitalWrite(MOTOR_B_IN2, LOW);
  } else {
    digitalWrite(MOTOR_B_IN1, LOW); digitalWrite(MOTOR_B_IN2, LOW);
  }
}

void setup() {
  Serial.begin(115200);

  // Setup Motor Pins as standard digital outputs
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  
  // Setup and WAKE UP the DRV8833 drivers
  pinMode(MOTOR_EEP, OUTPUT);
  digitalWrite(MOTOR_EEP, HIGH); 

  // Ensure motors are stopped on boot
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);

  // Setup Encoders
  pinMode(ENC_A_PHASE_A, INPUT_PULLUP);
  pinMode(ENC_A_PHASE_B, INPUT_PULLUP);
  pinMode(ENC_B_PHASE_A, INPUT_PULLUP);
  pinMode(ENC_B_PHASE_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PHASE_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PHASE_B), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PHASE_A), isrB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PHASE_B), isrB, CHANGE);

  Serial.println("System Ready. Commands: 'f' (Forward), 'b' (Back), 'a' (Test A), 'c' (Test B), 's' (Stop)");
}

void loop() {
  static uint32_t lastMs = 0;
  static int32_t lastCountA = 0;
  static int32_t lastCountB = 0;

  // 1. Handle Serial Commands
  if (Serial.available()) {
    char cmd = Serial.read();
    
    // Clear out any newline characters from the serial monitor
    if (cmd == '\n' || cmd == '\r') return; 

    if (cmd == 'f') {
      Serial.println("---> STATE: FORWARD");
      setMotorSpeed(200, 200);
    } 
    else if (cmd == 'b') {
      Serial.println("---> STATE: BACKWARD");
      setMotorSpeed(-200, -200);
    } 
    else if (cmd == 's') {
      Serial.println("---> STATE: STOP");
      setMotorSpeed(0, 0);
    }
    // --- Independent Motor Testing ---
    else if (cmd == 'a') {
      Serial.println("---> STATE: TEST MOTOR A ONLY (Fwd)");
      setMotorSpeed(200, 0);
    }
    else if (cmd == 'c') {
      Serial.println("---> STATE: TEST MOTOR B ONLY (Fwd)");
      setMotorSpeed(0, 200);
    }
    // ... existing f, b, s, a, c commands ...

    // --- Automated Hardware Diagnostic (ROS-style self-test) ---
    else if (cmd == 't') {
      Serial.println("\n--- RUNNING ENCODER/MOTOR MAPPING TEST ---");
      
      // Test Motor A
      noInterrupts(); countA = 0; countB = 0; interrupts();
      Serial.println("Spinning Motor A for 1 second...");
      setMotorSpeed(200, 0);
      delay(1000);
      setMotorSpeed(0, 0);
      Serial.printf("Result -> Enc A: %d | Enc B: %d\n", countA, countB);
      if (abs(countB) > abs(countA)) Serial.println(">> ERROR: Motor A is driving Encoder B!");
      
      delay(1000);

      // Test Motor B
      noInterrupts(); countA = 0; countB = 0; interrupts();
      Serial.println("Spinning Motor B for 1 second...");
      setMotorSpeed(0, 200);
      delay(1000);
      setMotorSpeed(0, 0);
      Serial.printf("Result -> Enc A: %d | Enc B: %d\n", countA, countB);
      if (abs(countA) > abs(countB)) Serial.println(">> ERROR: Motor B is driving Encoder A!");
      
      Serial.println("------------------------------------------\n");
    }
  }

  // 2. Calculate and Report RPM every 200ms
  uint32_t now = millis();
  if (now - lastMs >= 200) {
    int32_t curA, curB;
    
    noInterrupts();
    curA = countA;
    curB = countB;
    interrupts();

    float dt = (now - lastMs) / 1000.0f;
    float rpmA = ((curA - lastCountA) / dt / CPR_X4) * 60.0f;
    float rpmB = ((curB - lastCountB) / dt / CPR_X4) * 60.0f;

    Serial.printf("A: %d | RPM: %.2f  ---  B: %d | RPM: %.2f\n", curA, rpmA, curB, rpmB);

    lastCountA = curA;
    lastCountB = curB;
    lastMs = now;
  }
}
