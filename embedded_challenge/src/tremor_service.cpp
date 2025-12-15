#include "tremor_service.h"

// PROJECT SPEC COMPLIANT: Frequency band definitions
#define TREMOR_LOW_HZ 3.0f
#define TREMOR_HIGH_HZ 5.0f      // Spec: 3-5 Hz for tremors
#define DYSK_LOW_HZ 5.0f         // Spec: 5-7 Hz for dyskinesia
#define DYSK_HIGH_HZ 7.0f        

// AGGRESSIVE detection thresholds for better tremor detection
#define REST_THRESHOLD 0.3f			// Below this = resting (very sensitive)
#define ENERGY_THRESHOLD 0.4f		// Min energy for detection (very sensitive)
#define TREMOR_RATIO 0.08f			// Tremor power must be >8% of total (AGGRESSIVE)
#define DYSKINESIA_RATIO 0.15f		// Dysk power must be >15% of total

void Tremor_Service::init(){
    // init arm dsp fft
    arm_rfft_fast_init_f32(&S, FFT_SIZE);
    
    // init buffer with baseline 
    for (int i = 0; i < FFT_SIZE; i++){
        inputData[i] = 1.0f;
    }
    
    // init public state vars
    is_tremor = false;
    is_dyskinesia = false;
    is_resting = true;
    tremor_power = 0;
    dyskinesia_power = 0;
    dominant_freq = 0;
    total_energy = 0;
    new_samples_collected = 0;
    instant_energy_sum = 0;
}

bool Tremor_Service::update(float x, float y, float z){
    // 1. calc acceleration magnitude
    float mag = sqrtf(x*x+y*y+z*z);
    
    // 2. shift circular buffer
    for (int i = 0; i < FFT_SIZE - 1; i++){
        inputData[i] = inputData[i+1];
    }
    inputData[FFT_SIZE - 1] = mag;
    
    new_samples_collected++;
    
    // 3. instant energy tracking
    // accumulate deviation from 1g
    instant_energy_sum += fabsf(mag - 1.0f);
    
    // 4. batch check - only process every NEW_SAMPLES
    if (new_samples_collected < NEW_SAMPLES){
        return false;  // not ready yet
    }
    
    // BATCH PROCESSING (every 10th samples)
    // if barely any movement detected, mark as resting immediately
    if (instant_energy_sum < REST_THRESHOLD) {
        // reset all detection flags
        is_tremor = false;
        is_dyskinesia = false;
        is_resting = true;
        tremor_power = 0;
        dyskinesia_power = 0;
        dominant_freq = 0;
        total_energy = 0;
        
        // reset counters
        instant_energy_sum = 0;
        new_samples_collected = 0;
        return true;   // analysis completed (detected rest)
    }
    
    // reset for next batch
    instant_energy_sum = 0;
    new_samples_collected = 0;
    
    // FFT (frequency analysis)
    // B. prepare FFT buffer
    float fftBuffer[FFT_SIZE];
    memcpy(fftBuffer, inputData, sizeof(inputData));
    
    // C. remove DC component (mean)
    float mean;
    arm_mean_f32(fftBuffer, FFT_SIZE, &mean);
    arm_offset_f32(fftBuffer, -mean, fftBuffer, FFT_SIZE);
    
    // D. apply hanning window to reduce spectral leakage
    applyHanningWindow(fftBuffer, FFT_SIZE);
    
    // E. perform FFT
    arm_rfft_fast_f32(&S, fftBuffer, fftOutput, 0);
    
    // F. calculate magnitude spectrum
    arm_cmplx_mag_f32(fftOutput, fftMag, FFT_SIZE/2);
    
    // Band Power Calculation
    
    // freq resolution: sample rate / fft size
    float freq_resolution = SAMPLE_RATE / FFT_SIZE;
    
    // calculate bin indices for freq bands (SPEC COMPLIANT)
    int tremor_low_bin = (int)(TREMOR_LOW_HZ / freq_resolution);
    int tremor_high_bin = (int)(TREMOR_HIGH_HZ / freq_resolution);
    int dysk_low_bin = (int)(DYSK_LOW_HZ / freq_resolution);
    int dysk_high_bin = (int)(DYSK_HIGH_HZ / freq_resolution);
    
    // get power in each band - STORE IN MEMBER VARIABLES
    tremor_power = getBandPower(tremor_low_bin, tremor_high_bin);
    dyskinesia_power = getBandPower(dysk_low_bin, dysk_high_bin);
    
    int total_low_bin = (int)(0.5f/freq_resolution);
    int total_high_bin = (int)(15.0f / freq_resolution);
    total_energy = getBandPower(total_low_bin, total_high_bin);
    
    // find dominant frequency - STORE IN MEMBER VARIABLE
    dominant_freq = findDominantFrequency();
    
    // Classification logic (SPEC ALIGNED)
    
    // G. check if enough energy to analyze
    if (total_energy < ENERGY_THRESHOLD){
        is_resting = true;
        is_tremor = false;
        is_dyskinesia = false;
    } else{
        is_resting = false;
        
        // H. calc power ratios
        float tremor_ratio = tremor_power / total_energy;
        float dysk_ratio = dyskinesia_power / total_energy;
        
        // DEBUG OUTPUT - Shows what's happening
        printf("[DEBUG] Freq:%.2f | T_pow:%.3f(%.1f%%) | D_pow:%.3f(%.1f%%)\r\n",
               dominant_freq, tremor_power, tremor_ratio*100.0f, 
               dyskinesia_power, dysk_ratio*100.0f);
        
        // Calculate detection conditions
        bool tremor_power_ok = (tremor_ratio > TREMOR_RATIO);
        bool dysk_power_ok = (dysk_ratio > DYSKINESIA_RATIO);
        
        // I. IMPROVED PRIORITY-BASED DETECTION
        // Reset flags first
        is_tremor = false;
        is_dyskinesia = false;
        
        // Priority 1: Tremor (3-5 Hz) - Primary PD symptom
        // Extended to 5.5Hz to catch edge cases
        if (tremor_power_ok && dominant_freq >= 3.0f && dominant_freq <= 5.5f) {
            // Check if it's CLEARLY tremor (3-4.5 Hz with good power)
            if (dominant_freq < 4.5f) {
                is_tremor = true;
                printf("    [TREMOR] Detected at %.2f Hz (%.1f%% power)\r\n", 
                       dominant_freq, tremor_ratio*100.0f);
            }
            // Edge case: 4.5-5.5 Hz - could be tremor OR dyskinesia
            // Use power comparison to decide
            else if (tremor_power > dyskinesia_power) {
                is_tremor = true;
                printf("    [TREMOR] High-freq tremor at %.2f Hz (%.1f%% power)\r\n", 
                       dominant_freq, tremor_ratio*100.0f);
            }
        }
        
        // Priority 2: Dyskinesia (5-7 Hz) - Only if NOT tremor
        if (!is_tremor && dysk_power_ok && dominant_freq >= 5.0f && dominant_freq <= 7.5f) {
            // Must have clear dyskinesia frequency (>5.5 Hz) OR significantly more dysk power
            if (dominant_freq > 5.5f || dyskinesia_power > tremor_power * 1.2f) {
                is_dyskinesia = true;
                printf("    [DYSK] Detected at %.2f Hz (%.1f%% power)\r\n", 
                       dominant_freq, dysk_ratio*100.0f);
            }
        }
    }
    
    return true; // analysis complete
}

// Private helper funcs

void Tremor_Service::applyHanningWindow(float* data, int length){
    for (int i = 0; i < length; i++){
        // Hanning window: 0.5 * (1-cos(2pi*i/(N-1)))
        float window = 0.5f * (1.0f-arm_cos_f32(2.0f * PI * i / (length-1)));
        data[i] *= window;
    }
}

float Tremor_Service::getBandPower(int low_bin, int high_bin){
    float power = 0;
    for (int i = low_bin; i <= high_bin && i < FFT_SIZE/2; i++){
        power += fftMag[i];
    }
    return power;
}

float Tremor_Service::findDominantFrequency(){
    // find peak in relevant range (2-8Hz to cover both conditions)
    float freq_resolution = SAMPLE_RATE / FFT_SIZE;
    int start_bin = (int)(2.0f / freq_resolution);
    int end_bin = (int)(8.0f / freq_resolution);
    
    int peak_bin = start_bin;
    float peak_mag = fftMag[start_bin];
    
    for (int i = start_bin; i <= end_bin && i < FFT_SIZE/2; i++){
        if (fftMag[i] > peak_mag){
            peak_mag = fftMag[i];
            peak_bin = i;
        }
    }
    
    // convert bin to frequency
    return peak_bin * freq_resolution;
}