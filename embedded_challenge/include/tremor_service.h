// Tremor & Dyskinesia Detection Service
// Match arch of FOG_service

#ifndef TREMOR_SERVICE_H
#define TREMOR_SERVICE_H

#include "mbed.h"
#include "arm_math.h"
#include <cstring>

// FFT configuration
#define FFT_SIZE 256			// Must be power of 2
#define NEW_SAMPLES 10			// process every 10th sample
#define SAMPLE_RATE 104.0f		// Hz (matches LSM6DSL ODR)

class Tremor_Service{
public:
	// accessible by main.cpp
	bool is_tremor;
	bool is_dyskinesia;
	bool is_resting;
	
	float tremor_power;
	float dyskinesia_power;
	float dominant_freq;
	float total_energy;
	
	// public methods
	// init service
	// call at startup
	void init();
	
	// update with new sensor data
	// param x: accelerometer X-axis
	// param y: accelerometer Y-axis
	// param z: accelerometer Z-axis
	// return: true when analysis is completed
	bool update(float x, float y, float z);
	
private:
	// private vars
	// arm dsp fft instance
	arm_rfft_fast_instance_f32 S;
	
	// circular buffer for magnitude data
	float inputdata[FFT_SIZE];
	
	// FFT output buffers
	float fftOutput[FFT_SIZE];
	float fftMag[FFT_SIZE/2];
	
	// Sample counter
	int new_samples_colelcted;
	
	// Instant energy tracking for rest detection
	float instant_energy_sum;
	
	// private methods
	
	// apply hanning window to reduce spectral leakage
	void applyHanningWindow(float* data, int length);
	
	// calculate power in a frequency band
	float getBandPower(int start_bin, int end_bin);
	
	// find dominant frequency in the spectrum 
	float findDominantFrequency();
	
}

#endif // TREMOR_SERVICE_H


class FrequencyAnalyzer{
public:
	// init the FFT module
	// call this once at startup before using any other functions
	// RETURN - true if init successful
	static bool init();
	
	// param data - array of raw accelerometer samples (e.g., acc_x or acc_magnitude)
	// param length - # of samples (should be FFT_SIZE)
	// returns Dominant frequency in Hz (-1.0 on error)
	
	static float getDominantFrequency(float* data, uint32_t length);
	
	// get total power in freq band
	// param data - array of raw accelerometer samples (e.g., acc_x or acc_magnitude)
	// param length - # of samples (should be FFT_SIZE)
	// param low_freq = lower freq bound (Hz)
	// param high_freq = upper freq bound (Hz)
	// return - Total Power in band (-1.0 on error)
	static float getBandPower(float* data, uint32_t length,
								float low_freq, float high_freq);
	
	// Analyze movement and classify as tremor or dyskinesia
	// returns MovementClassification Structure with results
	static MovementClassification analyzeMovement(float* data, uint32_t length);
	
	// calcualte magnitude from 3-axis acceleromter
	// mag = sqrt(x^2 + y^2 + z^2)
	// param acc_x - X-axis samples
	// param acc_y - Y-axis samples
	// param acc_z - Z-axis samples
	// param output - Output buffer for magnitude
	// param length: # of samples
	
	static void calculateMagnitude(float* acc_x, float* acc_y, float* acc_z,
									float output, uint32_t length);
									
	// helper functions ---
	
	//  get current FFT size
	static uint32_t getFFTSize();
	
	// get sample rate in Hz
	static float getSampleRate();
	
	// get freq resolution (Hz per FFT bin)
	static float getFrequencyResolution();
	
private:
	// apply Hanning window to data
	// reduce spectral leakage in FFT
	static void applyHanningWindow(float* data, uint32_t length);
};

#endif // FREQUENCY_ANALYZER_H
