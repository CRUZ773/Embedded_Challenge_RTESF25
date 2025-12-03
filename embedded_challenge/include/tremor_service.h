// Tremor & Dyskinesia Detection Service
// Matches architecture of FOG_Service

#ifndef TREMOR_SERVICE_H
#define TREMOR_SERVICE_H

#include "mbed.h"
#include "arm_math.h"
#include <cstring>

// Math constant
#ifndef PI
#define PI 3.14159265358979323846f
#endif

// FFT configuration
#define FFT_SIZE 256            // Must be power of 2
#define NEW_SAMPLES 10          // Process every 10th sample
#define SAMPLE_RATE 52.0f       // Hz (matches LSM6DSL ODR at 52Hz)

class Tremor_Service {
public:
    // Public state variables - accessible by main.cpp
    bool is_tremor;
    bool is_dyskinesia;
    bool is_resting;
    
    float tremor_power;
    float dyskinesia_power;
    float dominant_freq;
    float total_energy;
    
    // Public methods
    
    // Initialize service - call at startup
    void init();
    
    // Update with new sensor data
    // Parameters:
    //   x: accelerometer X-axis (g)
    //   y: accelerometer Y-axis (g)
    //   z: accelerometer Z-axis (g)
    // Returns: true when analysis batch is completed
    bool update(float x, float y, float z);
    
private:
    // Private member variables
    
    // ARM DSP FFT instance
    arm_rfft_fast_instance_f32 S;
    
    // Circular buffer for magnitude data
    float inputData[FFT_SIZE];
    
    // FFT output buffers
    float fftOutput[FFT_SIZE];
    float fftMag[FFT_SIZE/2];
    
    // Sample counter for batching
    int new_samples_collected;
    
    // Instant energy tracking for rest detection
    float instant_energy_sum;
    
    // Private methods
    
    // Apply Hanning window to reduce spectral leakage
    // Parameters:
    //   data: input data array
    //   length: number of samples
    void applyHanningWindow(float* data, int length);
    
    // Calculate power in a frequency band
    // Parameters:
    //   start_bin: starting FFT bin index
    //   end_bin: ending FFT bin index
    // Returns: total power in the band
    float getBandPower(int start_bin, int end_bin);
    
    // Find dominant frequency in the spectrum
    // Returns: dominant frequency in Hz
    float findDominantFrequency();
};

#endif // TREMOR_SERVICE_H