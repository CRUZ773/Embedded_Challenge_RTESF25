/*
  Parkinson's Disease Monitor - Group Project
  - Tremor Detection (3-4.8 Hz)
  - Dyskinesia Detection (4.8-7 Hz)
  - Freezing of Gait (FOG) Detection (5.5-8 Hz)
  - BLE Communication with 3 separate characteristics

  Uses LSM6DSL sensor with Arduino framework
  tremor service integrated
  3-second FFT window @ 52 Hz with zero-padding to 256-point FFT
*/

#include <Arduino.h>
#include <Wire.h>
#include <LSM6DSLSensor.h>
#include <arduinoFFT.h>
#include <STM32duinoBLE.h>

// Debug flag to keep serial prints for tuning
#ifndef DEBUG_VERBOSE
#define DEBUG_VERBOSE 1
#endif

// ==========================================
// HARDWARE PINS
// ==========================================
#define PIN_LED_TREMOR    PA5   // LD1 (Green)
#define PIN_LED_DYSK      PB14  // LD2 (Green)
#define PIN_LED_FOG       PC9   // LD3 (Red/Yellow)
#define PIN_LED_BLE       PB7   // LD4 (Blue)

#define LED_ON  HIGH
#define LED_OFF LOW

// ==========================================
// DETECTION PARAMETERS
// ==========================================
#define FFT_SIZE 256             // FFT size (power of 2)
#define SAMPLING_FREQUENCY 52.0 // Hz
#define WINDOW_SECONDS 2.0      // analysis window (seconds)
#define WINDOW_SAMPLES ((int)(SAMPLING_FREQUENCY * WINDOW_SECONDS))

// Frequency ranges (NON-OVERLAPPING)
#define TREMOR_LOW_HZ   3.0
#define TREMOR_HIGH_HZ  5.0   // tighten to reduce overlap with dyskinesia
#define DYSK_LOW_HZ     5.0   // start dysk slightly above tremor ceiling
#define DYSK_HIGH_HZ    8.0
#define FOG_LOW_HZ      5.5   // widen slightly to ease FOG detection
#define FOG_HIGH_HZ     8.0

// Detection thresholds
#define THRESHOLD_ENERGY      0.8    // Minimum energy to detect anything
#define THRESHOLD_REST        0.6    // Below this = at rest (legacy quick check)
#define REST_RMS_THRESHOLD    5.0    // mg-level RMS motion gate to flag rest (looser so freezes pass)
#define TREMOR_POWER_RATIO    0.30   // tremor ratio threshold
#define DYSK_POWER_RATIO      0.25   // dysk ratio threshold (harder to beat FOG)
#define LOCO_POWER_MIN   80000.0     // floor locomotion power to stabilize FOG ratio
#define WALK_MASK_RATIO   0.60       // if locomotion dominates total energy, suppress tremor/dysk
#define FOG_INDEX_THRESH  0.30       // freeze index needed to fire (easier)
#define FOG_LP_DROP_RATIO 0.60       // current loco power must drop below 60% of previous to signal freeze

// ==========================================
// GLOBAL VARIABLES
// ==========================================
TwoWire my_i2c(PB11, PB10); 
LSM6DSLSensor *AccGyr;
arduinoFFT FFT = arduinoFFT();

// FFT buffers
double vReal[FFT_SIZE];
double vImag[FFT_SIZE];
int sampleIndex = 0;
unsigned long lastSampleTime = 0;
const unsigned long samplingPeriod = 1000000 / (unsigned long)SAMPLING_FREQUENCY; // microseconds

// Detection state
bool is_tremor = false;
bool is_dyskinesia = false;
bool is_fog = false;
bool is_resting = true;

float dominant_freq = 0;
float tremor_power = 0;
float dysk_power = 0;
float fog_power = 0;
float total_energy = 0;

// FOG specific variables
float fog_index = 0;
float instant_energy_sum = 0;
int samples_since_last_analysis = 0;
float prev_loco_power = 0; // remembers last locomotion energy

// BLE Setup (3 characteristics as required)
BLEService parkinsonService("A0E1B2C3-D4E5-F6A7-B8C9-D0E1F2A3B4C5");
BLEStringCharacteristic tremorChar("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C6", BLERead | BLENotify, 20);
BLEStringCharacteristic dyskinesiaChar("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C7", BLERead | BLENotify, 20);
BLEStringCharacteristic fogChar("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C8", BLERead | BLENotify, 20);

// ==========================================
// FUNCTION PROTOTYPES
// ==========================================
void performFFTAnalysis();
void updateBLECharacteristics();
void updateLEDs();

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(2000); 
  Serial.println("\n========================================");
  Serial.println("  Parkinson's Disease Monitor");
  Serial.println("========================================");

  // Initialize LEDs
  pinMode(PIN_LED_TREMOR, OUTPUT);
  pinMode(PIN_LED_DYSK, OUTPUT);
  pinMode(PIN_LED_FOG, OUTPUT);
  pinMode(PIN_LED_BLE, OUTPUT);
  
  digitalWrite(PIN_LED_TREMOR, LED_OFF);
  digitalWrite(PIN_LED_DYSK, LED_OFF);
  digitalWrite(PIN_LED_FOG, LED_OFF);
  digitalWrite(PIN_LED_BLE, LED_OFF);
  
  // Initialize I2C and sensor
  my_i2c.begin();
  AccGyr = new LSM6DSLSensor(&my_i2c, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW);
  
  if(AccGyr->begin() != LSM6DSL_STATUS_OK) {
    Serial.println("ERROR: Sensor initialization failed!");
    while(1) { 
      digitalWrite(PIN_LED_BLE, !digitalRead(PIN_LED_BLE)); 
      delay(100); 
    }
  }

  AccGyr->Enable_X();
  AccGyr->Set_X_FS(2);      // ±2g range
  AccGyr->Set_X_ODR(52.0f); // 52 Hz sampling
  Serial.println("Sensor: OK");
  
  // Initialize BLE
  if (!BLE.begin()) { 
    Serial.println("ERROR: BLE initialization failed!"); 
    while (1); 
  }
  
  BLE.setLocalName("PD-Monitor-G28");
  BLE.setAdvertisedService(parkinsonService);
  
  parkinsonService.addCharacteristic(tremorChar);
  parkinsonService.addCharacteristic(dyskinesiaChar);
  parkinsonService.addCharacteristic(fogChar);
  BLE.addService(parkinsonService);
  
  tremorChar.writeValue("TRM:0,0.00,0.000");
  dyskinesiaChar.writeValue("DSK:0,0.00,0.000");
  fogChar.writeValue("FOG:0,0.00,0.00");
  
  BLE.advertise();
  digitalWrite(PIN_LED_BLE, LED_ON);
  
  Serial.println("BLE: OK");
  Serial.println("Service UUID: A0E1B2C3-D4E5-F6A7-B8C9-D0E1F2A3B4C5");
  Serial.println("Tremor:  A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C6");
  Serial.println("Dysk:    A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C7");
  Serial.println("FOG:     A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C8");
  Serial.println("========================================\n");
  Serial.println("System Ready. Waiting for data...\n");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  // Handle BLE connections
  BLE.poll();
  
  unsigned long currentMicros = micros();
  
  // Sample at 52 Hz
  if (currentMicros - lastSampleTime >= samplingPeriod) {
    lastSampleTime = currentMicros;

    // Read accelerometer
    int32_t accelerometer[3];
    AccGyr->Get_X_Axes(accelerometer);
    
    double x = (double)accelerometer[0];
    double y = (double)accelerometer[1];
    double z = (double)accelerometer[2];
    
    // Calculate magnitude
    double magnitude = sqrt(x*x + y*y + z*z);
    
    // Track energy for rest detection
    instant_energy_sum += abs(magnitude - 1000.0);
    samples_since_last_analysis++;

    // Fill FFT buffer
    if (sampleIndex < WINDOW_SAMPLES) {
      vReal[sampleIndex] = magnitude;
      vImag[sampleIndex] = 0;
      sampleIndex++;
    }

    // If we have 3 seconds of data, zero-pad and analyze
    if (sampleIndex >= WINDOW_SAMPLES) {
      for (int i = sampleIndex; i < FFT_SIZE; i++) {
        vReal[i] = 0;
        vImag[i] = 0;
      }

      performFFTAnalysis();
      updateBLECharacteristics();
      updateLEDs();
      
      sampleIndex = 0;
      instant_energy_sum = 0;
      samples_since_last_analysis = 0;
    }
  }
}

// ==========================================
// FFT ANALYSIS (Your tremor_service logic)
// ==========================================
void performFFTAnalysis() {
  
  // Quick rest check (legacy energy gate). Allow bypass if recent locomotion could indicate freeze.
  if (instant_energy_sum < THRESHOLD_REST * samples_since_last_analysis && prev_loco_power <= LOCO_POWER_MIN) {
    is_resting = true;
    is_tremor = false;
    is_dyskinesia = false;
    is_fog = false;
    tremor_power = 0;
    dysk_power = 0;
    fog_power = 0;
    dominant_freq = 0;
    total_energy = 0;
    fog_index = 0;
    if (DEBUG_VERBOSE) {
      Serial.print("Rest gate (energy) | inst_sum: ");
      Serial.print(instant_energy_sum, 2);
      Serial.print(" | samples: ");
      Serial.println(samples_since_last_analysis);
    }
    return;
  }
  
  // Remove DC component only over the captured 3 s window
  double mean = 0;
  for (int i = 0; i < WINDOW_SAMPLES; i++) { mean += vReal[i]; }
  mean /= WINDOW_SAMPLES;
  for (int i = 0; i < WINDOW_SAMPLES; i++) { vReal[i] -= mean; }
  for (int i = WINDOW_SAMPLES; i < FFT_SIZE; i++) { vReal[i] = 0; }

  // RMS gate: if very little motion energy, treat as rest and skip FFT noise
  double sum_sq = 0;
  for (int i = 0; i < WINDOW_SAMPLES; i++) { sum_sq += vReal[i] * vReal[i]; }
  double rms_motion = sqrt(sum_sq / WINDOW_SAMPLES);
  if (rms_motion < REST_RMS_THRESHOLD && prev_loco_power <= LOCO_POWER_MIN) {
    is_resting = true;
    is_tremor = false;
    is_dyskinesia = false;
    is_fog = false;
    tremor_power = 0;
    dysk_power = 0;
    fog_power = 0;
    dominant_freq = 0;
    total_energy = 0;
    fog_index = 0;
    if (DEBUG_VERBOSE) {
      Serial.print("Rest gate (RMS) | rms: ");
      Serial.print(rms_motion, 2);
      Serial.print(" mg | threshold: ");
      Serial.println(REST_RMS_THRESHOLD, 2);
    }
    return;
  }

  // Apply windowing
  FFT.Windowing(vReal, FFT_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  
  // Perform FFT
  FFT.Compute(vReal, vImag, FFT_SIZE, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, FFT_SIZE);

  // Calculate band powers
  tremor_power = 0;
  dysk_power = 0;
  fog_power = 0;
  total_energy = 0;
  
  float loco_power = 0; // For FOG index calculation
  
  int peak_bin = 1;
  double peak_magnitude = vReal[1];
  
  for (int i = 1; i < (FFT_SIZE / 2); i++) {
    float freq = (i * SAMPLING_FREQUENCY) / FFT_SIZE;
    if (freq < 0.5f || freq > 15.0f) {
      continue; // Ignore bins outside physiological band
    }
    double power = vReal[i] * vReal[i]; // Power = magnitude^2
    
    // Tremor band (3-4.8 Hz)
    if (freq >= TREMOR_LOW_HZ && freq < TREMOR_HIGH_HZ) {
      tremor_power += power;
    }
    
    // Dyskinesia band (4.8-7 Hz)
    if (freq >= DYSK_LOW_HZ && freq < DYSK_HIGH_HZ) {
      dysk_power += power;
    }
    
    // FOG band (5-8 Hz)
    if (freq >= FOG_LOW_HZ && freq <= FOG_HIGH_HZ) {
      fog_power += power;
    }
    
    // Locomotion band (0.5-5 Hz) for FOG index
    if (freq >= 0.5 && freq <= 5.0) {
      loco_power += power;
    }
    
    // Total energy (0.5-15 Hz)
    if (freq >= 0.5 && freq <= 15.0) {
      total_energy += power;
    }
    
    // Track peak for dominant frequency within 0.5-15 Hz band
    if (vReal[i] > peak_magnitude) {
      peak_magnitude = vReal[i];
      peak_bin = i;
    }
  }
  
  dominant_freq = (peak_bin * SAMPLING_FREQUENCY) / FFT_SIZE;
  
  // Calculate FOG index (freeze/locomotion ratio) with locomotion floor
  float raw_loco_power = loco_power;
  if (loco_power < LOCO_POWER_MIN) loco_power = LOCO_POWER_MIN;
  fog_index = fog_power / loco_power;
  
  // Check if enough energy to analyze
  if (total_energy < THRESHOLD_ENERGY) {
    is_resting = true;
    is_tremor = false;
    is_dyskinesia = false;
    is_fog = false;
    return;
  }
  
  is_resting = false;
  
  // Calculate power ratios
  float tremor_ratio = tremor_power / total_energy;
  float dysk_ratio = dysk_power / total_energy;
  float loco_ratio = raw_loco_power / total_energy;
  
  // Detection logic with priority: FOG > Dyskinesia > Tremor
  bool locomotion_recent = prev_loco_power > LOCO_POWER_MIN;
  bool locomotion_drop = locomotion_recent && raw_loco_power < (prev_loco_power * FOG_LP_DROP_RATIO);

  // Walking mask: if locomotion dominates and no freeze signature, suppress tremor/dysk but still evaluate FOG
  if (loco_ratio > WALK_MASK_RATIO && fog_index < 2.0f) {
    is_tremor = false;
    is_dyskinesia = false;
    // do not return; allow FOG check to proceed
    if (DEBUG_VERBOSE) {
      Serial.print("Walking mask | loco_ratio: ");
      Serial.println(loco_ratio, 2);
    }
  }

  // Priority 1: FOG when recent locomotion drops sharply and freeze index is present (band optional if index strong)
  if (locomotion_drop && (
        fog_index > FOG_INDEX_THRESH ||
        (fog_index > (FOG_INDEX_THRESH * 0.8f) && dominant_freq >= FOG_LOW_HZ && dominant_freq <= FOG_HIGH_HZ)
      )) {
    is_fog = true;
    is_tremor = false;
    is_dyskinesia = false;
  }
  // Priority 2: Dyskinesia (4.8-7 Hz)
  else if (!locomotion_drop && dysk_ratio > DYSK_POWER_RATIO && 
           dominant_freq >= DYSK_LOW_HZ && dominant_freq < DYSK_HIGH_HZ &&
           dysk_power > (tremor_power * 1.5f)) {
    is_dyskinesia = true;
    is_tremor = false;
    is_fog = false;
  }
  // Priority 3: Tremor (3-4.8 Hz)
  else if (tremor_ratio > TREMOR_POWER_RATIO && 
           dominant_freq >= TREMOR_LOW_HZ && dominant_freq < TREMOR_HIGH_HZ) {
    is_tremor = true;
    is_dyskinesia = false;
    is_fog = false;
  }
  else {
    is_tremor = false;
    is_dyskinesia = false;
    is_fog = false;
  }

  // Remember locomotion energy for next-window FOG gating
  prev_loco_power = raw_loco_power;

  if (DEBUG_VERBOSE) {
    Serial.print("Freq: ");
    Serial.print(dominant_freq, 2);
    Serial.print(" Hz | FOG Index: ");
    Serial.print(fog_index, 2);
    Serial.print(" | tremor_ratio: ");
    Serial.print(tremor_ratio, 3);
    Serial.print(" | dysk_ratio: ");
    Serial.print(dysk_ratio, 3);
    Serial.print(" | loco_ratio: ");
    Serial.print(loco_ratio, 3);
    Serial.print(" | loco_drop: ");
    Serial.print(locomotion_drop ? 1 : 0);
    Serial.print(" | total_energy: ");
    Serial.println(total_energy, 3);
  }
}

// ==========================================
// UPDATE BLE CHARACTERISTICS
// ==========================================
void updateBLECharacteristics() {
  char buffer[20];
  
  // Update Tremor characteristic
  if (is_tremor) {
    snprintf(buffer, 20, "TRM:1,%.2f,%.3f", dominant_freq, tremor_power);
    Serial.println(">>> TREMOR DETECTED!");
  } else {
    snprintf(buffer, 20, "TRM:0,%.2f,%.3f", dominant_freq, tremor_power);
  }
  tremorChar.writeValue(buffer);
  
  // Update Dyskinesia characteristic
  if (is_dyskinesia) {
    snprintf(buffer, 20, "DSK:1,%.2f,%.3f", dominant_freq, dysk_power);
    Serial.println(">>> DYSKINESIA DETECTED!");
  } else {
    snprintf(buffer, 20, "DSK:0,%.2f,%.3f", dominant_freq, dysk_power);
  }
  dyskinesiaChar.writeValue(buffer);
  
  // Update FOG characteristic
  if (is_fog) {
    snprintf(buffer, 20, "FOG:1,%.2f,%.2f", fog_index, dominant_freq);
    Serial.println(">>> FOG DETECTED!");
  } else {
    snprintf(buffer, 20, "FOG:0,%.2f,%.2f", fog_index, dominant_freq);
  }
  fogChar.writeValue(buffer);
  
  // Debug output
  if (!is_resting) {
    Serial.print("Freq: ");
    Serial.print(dominant_freq, 2);
    Serial.print(" Hz | FOG Index: ");
    Serial.println(fog_index, 2);
  } else {
    Serial.println("At rest");
  }
}

// ==========================================
// UPDATE LED INDICATORS
// ==========================================
void updateLEDs() {
  digitalWrite(PIN_LED_TREMOR, is_tremor ? LED_ON : LED_OFF);
  digitalWrite(PIN_LED_DYSK, is_dyskinesia ? LED_ON : LED_OFF);
  digitalWrite(PIN_LED_FOG, is_fog ? LED_ON : LED_OFF);
}
