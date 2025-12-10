/*
  Parkinson's Disease Monitor
  - Tremor Detection
  - Dyskinesia Detection
  - Freezing of Gait (FOG) Detection
  - BLE Communication (String Characteristics)

  Uses LSM6DSL Accelerometer and Gyroscope
  FFT for motion analysis
  STM32duinoBLE for BLE communication
  Shake the board to simulate motion data for testing.

  LIGHT INDICATORS:
  LD1: Green (Tremor)
  LD2: Green (Dyskinesia)
  LD3: Red (FOG)
  LD4: Blue (BLE Status always on)
  LD5: Green (Power always on)
  LD6: Yellow (ST-Link always on)
*/

#include <Arduino.h>
#include <Wire.h>
#include "LSM6DSLSensor.h"
#include "arduinoFFT.h"
#include <STM32duinoBLE.h>

// ==========================================
// 1. Hardware pin definition
// ==========================================
#define PIN_LED_TREMOR    PA5   // LD1 (Green)
#define PIN_LED_DYSK      PB14  // LD2 (Green)
#define PIN_LED_FOG       PC9   // LD3 (Yellow)
#define PIN_LED_BLE       PE1   // LD4 (Blue)

#define LED_ON  HIGH
#define LED_OFF LOW

// ==========================================
// 2. Parameter configuration
// ==========================================
#define SAMPLES 128             
#define SAMPLING_FREQUENCY 52.0 

// Threshold 
#define THRESHOLD_MAG         2500.0  
#define THRESHOLD_WALKING     800.0   
#define THRESHOLD_FREEZE      400.0   





// Time parameter (ms)
#define WALK_CONFIRM_TIME     5000 
#define STEP_TIMEOUT          4000 
#define FOG_TRIGGER_DELAY     1000  

// ==========================================
// 3. Global variables
// ==========================================
TwoWire my_i2c(PB11, PB10); 
LSM6DSLSensor *AccGyr;
arduinoFFT FFT = arduinoFFT();

double vReal[SAMPLES];
double vImag[SAMPLES];
int sampleIndex = 0;
unsigned long lastSampleTime = 0;
const unsigned long samplingPeriod = 1000 / SAMPLING_FREQUENCY; 

// state management
enum DetectionState { STATE_IDLE, STATE_TREMOR, STATE_DYSKINESIA };
DetectionState currentMotionState = STATE_IDLE;

// FOG variable
bool isWalkingConfirmed = false;    
bool isFogActive = false;           
unsigned long lastHighEnergyTime = 0; 
unsigned long walkSessionStartTime = 0; 

// Timer
unsigned long debugTimer = 0;

// BLE
BLEService parkinsonService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic tremorChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 20);
BLEStringCharacteristic dyskinesiaChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 20);
BLEStringCharacteristic fogChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 20);

void checkFOG_V3(double mag);
void performFFTAnalysis();
void updateMotionLEDs(DetectionState state);

// ==========================================
// 4. Setup
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(2000); 
  Serial.println("\n=== Parkinson's Disease Monitor  ===");

  pinMode(PIN_LED_TREMOR, OUTPUT);
  pinMode(PIN_LED_DYSK, OUTPUT);
  pinMode(PIN_LED_FOG, OUTPUT);
  pinMode(PIN_LED_BLE, OUTPUT);
  
  digitalWrite(PIN_LED_TREMOR, LED_OFF);
  digitalWrite(PIN_LED_DYSK, LED_OFF);
  digitalWrite(PIN_LED_FOG, LED_OFF);
  digitalWrite(PIN_LED_BLE, LED_ON); 
  
  my_i2c.begin();
  AccGyr = new LSM6DSLSensor(&my_i2c, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW);
  
  if(AccGyr->begin() != LSM6DSL_STATUS_OK) {
    Serial.println("Sensor Error!");
    while(1) { digitalWrite(PIN_LED_BLE, !digitalRead(PIN_LED_BLE)); delay(100); }
  }

  AccGyr->Enable_X();
  AccGyr->Set_X_FS(2);     
  AccGyr->Set_X_ODR(52.0f); 
  
  if (!BLE.begin()) { Serial.println("BLE Failed!"); while (1); }
  BLE.setLocalName("Group28_Monitor");
  BLE.setAdvertisedService(parkinsonService);
  
  parkinsonService.addCharacteristic(tremorChar);
  parkinsonService.addCharacteristic(dyskinesiaChar);
  parkinsonService.addCharacteristic(fogChar);
  BLE.addService(parkinsonService);
  
  tremorChar.writeValue("Normal");
  dyskinesiaChar.writeValue("Normal");
  fogChar.writeValue("Normal");
  
  BLE.advertise();
  
  Serial.println("System Ready.");
}

// ==========================================
// 5. Loop
// ==========================================
void loop() {
  BLE.poll();
  digitalWrite(PIN_LED_BLE, LED_ON);

  if (millis() - lastSampleTime >= samplingPeriod) {
    lastSampleTime = millis();

    int32_t accelerometer[3];
    AccGyr->Get_X_Axes(accelerometer);
    
    double x = (double)accelerometer[0];
    double y = (double)accelerometer[1];
    double z = (double)accelerometer[2];
    double magnitude = sqrt(x*x + y*y + z*z);
    double cleanMag = abs(magnitude - 1000.0);
    
    // FOG 
    checkFOG_V3(cleanMag); 

    // FFT
    if (sampleIndex < SAMPLES) {
      vReal[sampleIndex] = magnitude; 
      vImag[sampleIndex] = 0;
      sampleIndex++;
    }
    else {
      performFFTAnalysis();
      sampleIndex = 0; 
    }
  }
}

// ==========================================
// 6. FFT Analysis
// ==========================================
void performFFTAnalysis() {
  double mean = 0;
  for (int i = 0; i < SAMPLES; i++) { mean += vReal[i]; }
  mean /= SAMPLES;
  for (int i = 0; i < SAMPLES; i++) { vReal[i] -= mean; }

  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  double peakEnergy = 0;
  double peakFreq = 0;
  
  for (int i = 8; i < (SAMPLES / 2); i++) {
    if (vReal[i] > peakEnergy) {
      peakEnergy = vReal[i];
      peakFreq = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;
    }
  }

  DetectionState newState = STATE_IDLE;
  if (peakEnergy > THRESHOLD_MAG) {
    if (peakFreq >= 3.0 && peakFreq < 5.0) {
       newState = STATE_TREMOR;
    } 
    else if (peakFreq >= 5.0 && peakFreq <= 7.0) { 
       newState = STATE_DYSKINESIA;
    }
  }

  if (newState != currentMotionState) {
    currentMotionState = newState;
    updateMotionLEDs(currentMotionState);
  }
}

void updateMotionLEDs(DetectionState state) {
  digitalWrite(PIN_LED_TREMOR, LED_OFF);
  digitalWrite(PIN_LED_DYSK, LED_OFF);
  if (isFogActive) {
      tremorChar.writeValue("Normal");
      dyskinesiaChar.writeValue("Normal");
      return; 
  }

  switch (state) {
    case STATE_TREMOR:
      Serial.println(">> DETECTED: TREMOR"); 
      digitalWrite(PIN_LED_TREMOR, LED_ON);
      tremorChar.writeValue("Tremor Detected"); 
      dyskinesiaChar.writeValue("Normal");
      break;
      
    case STATE_DYSKINESIA:
      Serial.println(">> DETECTED: DYSKINESIA"); 
      digitalWrite(PIN_LED_DYSK, LED_ON);
      tremorChar.writeValue("Normal");
      dyskinesiaChar.writeValue("Dyskinesia!"); 
      break;
      
    case STATE_IDLE:
      tremorChar.writeValue("Normal");
      dyskinesiaChar.writeValue("Normal");
      break;
  }
}

// ==========================================
// 7. FOG Logic
// ==========================================
void checkFOG_V3(double mag) {
  
  unsigned long now = millis();

  // 1. Detect large movements
  if (mag > THRESHOLD_WALKING) {
    lastHighEnergyTime = now; 
    
    if (walkSessionStartTime == 0) {
      walkSessionStartTime = now;
    }
    
    // While walking -> Turn off FOG
    if (isFogActive) {
      isFogActive = false;
      digitalWrite(PIN_LED_FOG, LED_OFF);
      
      fogChar.writeValue("Walking..."); 
      Serial.println(">>> FOG Cleared");
    }
  }

  // 2. logic
  if (walkSessionStartTime > 0) {
    
    unsigned long walkDuration = now - walkSessionStartTime;
    unsigned long timeSinceLastMove = now - lastHighEnergyTime;

    // A. Confirm walking
    if (!isWalkingConfirmed && walkDuration > WALK_CONFIRM_TIME) {
      isWalkingConfirmed = true;
      Serial.println(">>> Walking Confirmed!");
      fogChar.writeValue("Walking Mode"); 
    }

    // B. Trigger FOG
    if (isWalkingConfirmed) {
      if (timeSinceLastMove > FOG_TRIGGER_DELAY && mag < THRESHOLD_FREEZE) {
        if (!isFogActive) {
          isFogActive = true;
          digitalWrite(PIN_LED_FOG, LED_ON);
          
          fogChar.writeValue("FOG ACTIVE!"); 
          Serial.println("!!! FOG DETECTED !!!");
        }
      }
    }

    // C. End the Session
    if (timeSinceLastMove > STEP_TIMEOUT) {
      if (walkSessionStartTime != 0) {
        walkSessionStartTime = 0; 
        isWalkingConfirmed = false;
        
        if (isFogActive) {
          isFogActive = false;
          digitalWrite(PIN_LED_FOG, LED_OFF);
          
          fogChar.writeValue("Normal"); 
          Serial.println("... Normal Stop ...");
        } else {
             fogChar.writeValue("Normal"); 
        }
      }
    }
  }
}