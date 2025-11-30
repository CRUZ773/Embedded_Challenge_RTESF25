#include "fog_service.h"

void FOG_Service::init() {
    // Initialize ARM DSP
    arm_rfft_fast_init_f32(&S, FFT_SIZE);
    
    // Fill buffer with dummy data so it doesn't crash on start
    for(int i=0; i<FFT_SIZE; i++) inputData[i] = 1.0f;
    
    new_samples_collected = 0;
}

bool FOG_Service::update(float x, float y, float z) {
    
    // 1. Calculate Magnitude
    float mag = sqrtf(x*x + y*y + z*z);
    
    // 2. Shift Buffer
    for (int i = 0; i < FFT_SIZE - 1; i++) {
        inputData[i] = inputData[i+1];
    }
    inputData[FFT_SIZE - 1] = mag; 
    
    new_samples_collected++;

    // --- 3. THE INSTANT STOP FIX ---
    static float instant_energy_sum = 0;
    instant_energy_sum += fabsf(mag - 1.0f); 

    // 4. Batch Check
    if (new_samples_collected < NEW_SAMPLES) {
        return false; 
    }
    
    // --- BATCH PROCESSING ---
    
    // A. INSTANT GATE CHECK
    // CHANGED: 0.8 -> 0.7
    // Lower number = Harder to trigger "Resting". 
    // This allows smooth movements to stay "Walking".
    if (instant_energy_sum < 0.6f) { 
        current_energy = 0;
        current_index = 0;
        is_resting = true;
        is_frozen = false;
        
        instant_energy_sum = 0;
        new_samples_collected = 0;
        return true; 
    }
    
    instant_energy_sum = 0; 
    new_samples_collected = 0; 

    // --- STANDARD FFT LOGIC ---
    float fftBuffer[FFT_SIZE];
    memcpy(fftBuffer, inputData, sizeof(inputData));

    float mean;
    arm_mean_f32(fftBuffer, FFT_SIZE, &mean);
    arm_offset_f32(fftBuffer, -mean, fftBuffer, FFT_SIZE);
    arm_rfft_fast_f32(&S, fftBuffer, fftOutput, 0);
    arm_cmplx_mag_f32(fftOutput, fftMag, FFT_SIZE / 2);

    float loco_energy = 0;
    float freeze_energy = 0;

    for (int i = 1; i < (FFT_SIZE / 2); i++) {
        float freq = i * 0.406f;
        
        // KEEP: 0.5f start frequency
        // This ensures slow/smooth walking is captured.
        if (freq >= 0.5f && freq <= 3.0f) loco_energy += fftMag[i];
        if (freq > 3.0f && freq <= 8.0f)  freeze_energy += fftMag[i];
    }
    
    // Logic Updates
    current_energy = loco_energy + freeze_energy;
    
    // KEEP: Sensitive Ratio
    if (loco_energy < 0.5f) loco_energy = 0.5f; 
    current_index = freeze_energy / loco_energy;

    // B. TOTAL ENERGY GATE
    // CHANGED: 1.0 -> 0.8
    // This prevents it from cutting out when you walk slowly.
    if (current_energy < 0.8f) {
        is_resting = true;
        is_frozen = false;
    } else {
        is_resting = false;
        if (current_index > 2.0f) is_frozen = true;
        else is_frozen = false;
    }

    return true; 
}