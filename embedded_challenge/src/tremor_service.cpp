#include "tremor_service.h"

// Frequency band definitions
#define TREMOR_LOW_HZ 3.0f
#define TREMOR_HIGH_HZ 5.0f
#define DYSK_LOW_HZ 5.0f
#define DYSK_HIGH_HZ 7.0f

// detection thresholds
#define REST_THRESHOLD 0.6f        // below this = resting
#define ENERGY_THRESHOLD 0.8f      // min energy for detection
#define TREMOR_RATIO 0.20f         // tremor power must be >20% of total
#define DYSKINESIA_RATIO 0.20f     // dysk power must be >20% of total

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
    float fftBuffer[FFT_SIZE];  // FIXED: was 'flaot'
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
    
    // calculate bin indices for freq bands
    int tremor_low_bin = (int)(TREMOR_LOW_HZ / freq_resolution);
    int tremor_high_bin = (int)(TREMOR_HIGH_HZ / freq_resolution);
    int dysk_low_bin = (int)(DYSK_LOW_HZ / freq_resolution);
    int dysk_high_bin = (int)(DYSK_HIGH_HZ / freq_resolution);
    
    // get power in each band
    tremor_power = getBandPower(tremor_low_bin, tremor_high_bin);
    dyskinesia_power = getBandPower(dysk_low_bin, dysk_high_bin);
    
    int total_low_bin = (int)(0.5f/freq_resolution);
    int total_high_bin = (int)(15.0f / freq_resolution);
    total_energy = getBandPower(total_low_bin, total_high_bin);
    
    // find dominant frequency
    dominant_freq = findDominantFrequency();
    
    // Classification logic
    
    // G. check if enough energy to analyze
    if (total_energy < ENERGY_THRESHOLD){  // FIXED: was ENERGY_TRESHOLD
        is_resting = true;
        is_tremor = false;
        is_dyskinesia = false;
    } else{
        is_resting = false;
        
        // H. calc power ratios
        float tremor_ratio = tremor_power / total_energy;
        float dysk_ratio = dyskinesia_power / total_energy;
        
        // I. detect tremor (3-5Hz)
        if (tremor_ratio > TREMOR_RATIO &&
            dominant_freq >= TREMOR_LOW_HZ &&
            dominant_freq <= TREMOR_HIGH_HZ){
            is_tremor = true;    
        } else{
            is_tremor = false;
        }

        // J. detect dyskinesia (5-7Hz)
        if (dysk_ratio > DYSKINESIA_RATIO &&
            dominant_freq >= DYSK_LOW_HZ &&
            dominant_freq <= DYSK_HIGH_HZ){
            is_dyskinesia = true;
        } else{
            is_dyskinesia = false;
        }
    }
    
    return true; // analysis complete
}  // FIXED: Added missing closing brace

// Private helper funcs

void Tremor_Service::applyHanningWindow(float* data, int length){
    for (int i = 0; i < length; i++){
        // Hanning window: 0.5 * (1-cos(2pi*i/(N-1)))
        float window = 0.5f * (1.0f-arm_cos_f32(2.0f * PI * i / (length-1)));
        data[i] *= window;
    }
    // FIXED: Removed misplaced 'return power;'
}

float Tremor_Service::getBandPower(int low_bin, int high_bin){
    float power = 0;  // FIXED: Added declaration
    for (int i = low_bin; i <= high_bin && i < FFT_SIZE/2; i++){
        power += fftMag[i];
    }
    return power;
}

float Tremor_Service::findDominantFrequency(){
    // find peak in relevant range (1-10Hz)
    float freq_resolution = SAMPLE_RATE / FFT_SIZE;
    int start_bin = (int)(1.0f / freq_resolution);
    int end_bin = (int)(10.0f / freq_resolution);
    
    int peak_bin = start_bin;  // FIXED: Added declaration
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