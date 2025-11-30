#ifndef FOG_SERVICE_H
#define FOG_SERVICE_H

#include "mbed.h"
#include "arm_math.h"

// Configuration
#define FFT_SIZE 128
#define NEW_SAMPLES 10 

class FOG_Service {
private:
    // Internal "Private" variables (Your teammates don't need to see these)
    float inputData[FFT_SIZE];
    float fftOutput[FFT_SIZE];
    float fftMag[FFT_SIZE / 2];
    arm_rfft_fast_instance_f32 S;
    
    int new_samples_collected; // Counter for our sliding window

public:
    // Public Results (Teammates can read these)
    float current_index = 0.0f;
    float current_energy = 0.0f;
    bool is_frozen = false;
    bool is_resting = false;

    // Functions
    void init();
    bool update(float x, float y, float z); // Returns TRUE if a new calculation finished
};

#endif