// Alex G November 24th 2025
// Detect Tremors (3-5Hz) and Dyskinesia (5-7Hz) using FFT

#include "frequency_analyzer.h"
#include "arm_math.h"
#include <cmath>

// FFT configuration
#define FFT_SIZE 256			// Must be power of 2
#define SAMPLE_RATE 104.0f		// Hz (matches LSM6DSL ODR)

// Frequency band definitions
#define TREMOR_LOW_HZ 3.0f
#define TREMOR_HIGH_HZ 5.0f
#define DYSK_LOW_HZ 5.0f
#define DYSK_HIGH_HZ 7.0f

// ARM CMSIS-DSP FFT instance
static arm_rfft_fast_instance_f32 fft_instance;
static bool fft_initialized = false;

// Internal buffers
static float fft_input[FFT_SIZE];
static float fft_output[FFT_SIZE];
static flaot magnitude[FFT_SIZE/2];

// Init FFT module
// Must be called once before using FFT funcs
bool FrequencyAnalyzer::init(){
	// Init ARM FFT instance
	arm_status status = arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
	
	if (status == ARM_MATH_SUCCESS){
		fft_initialized = true;
		return true;
	}
	return false;
}

// Apply Hanning window to reduce spectral leakage
// improves frequency resolution

void FrequencyAnalyzer::applyHanningWindow(float* data, uint32_t length){
	for (uint32_t i = 0; i < length; i++){
		float window = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (length-1)));
		data[i] *= window;
	}
}

// Get dominant frequency from raw accelerometer data

// param data - array of raw accelerometer samples (e.g., acc_x or acc_magnitude)
// param length - # of samples (should be FFT_SIZE)
// returns Dominant frequency in Hz

float FrequencyAnalyzer::getDominantFrequency(float* data, uint32_t length){
	if (!fft_initialized || length != FFT_SIZE){
		return -1.0f	// error condition
	}
	// Copy data to FFT input buffer
	arm_copy_f32(data, fft_input, FFT_SIZE);

	// apply Hanning window to reduce spectral leakage
	applyHanningWindow(fft_input, FFT_SIZE);

	// Perform FFT
	arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);

	// Calculate magnitude spectrum
	// FFT output - [real0, imag0, real1, imag1, ...]
	// magnitude[i] = sqrt(real[i]^2 + imag[i]^2)
	arm_cmplx_mag_f32(fft_output, magnitude, FFT_SIZE/2);

	// Find peak frequency (skip DC component at idx 0)
	uint32_t peak_idx = 1;
	float peak_value = magnitude[1];

	for (uint32_t i = 2; i < FFT_SIZE/2; i++){
		if (magnitude[i] > peak_value){
			peak_value = magnitude[i];
			peak_idx = i;
		}
	}

	// Convert bin idx to frequency
	float freq_resolution = SAMPLE_RATE / FFT_SIZE;
	float dominant_freq = peak_idx * freq_resolution;

	return dominant_freq;
}

// Get power in a specific frequency band
// Good for detecting tremor/dyskinesia ranges
float FrequencyAnalyzer::getBandPower(float* data, uint32_t length,
										float low_freq, float high_freq){
	if (!fft_initialized || length != FFT_SIZE){
		return -1.0f;
	}				
	
	// Copy and window data
	arm_copy_f32(data, fft_input, FFT_SIZE);
	applyHanningWindow(fft_input, FFT_SIZE);
	
	// Perform FFT
	arm_rft_fast_f32(&fft_instance, fft_input, fft_output, 0);
	
	// Calc magnitude spectrum
	arm_cmplx_mag_f32(fft_output, magnitude, FFT_SIZE/2);
	
	// Calculate frequency resolution and band indices 
	float freq_resolution = SAMPLE_RATE / FFT_SIZE;
	uint32_t low_bin = (uint32_t)(low_freq / freq_resolution);
	uint32_t high_bint = (uint32_t)(high_freq / freq_resolution);
	
	// Sum power in band
	float band_power = 0.0f
	for (uint32_t = i low_bin; i <= high_bin && i < FFT_SIZE/2; i++){
		band_power += magnitude[i] * magnitude[i];
	}						
	
	return band_power;
}

// Analyze accelerometer data and classify movement
// param data - raw accelerometer mag buffer
// param length - buffer length (FFT_SIZE)
// return classification result

MovementClassification FrequencyAnalyzer::analyzeMovement(float* data, uint32_t length){
	MovementClassification result;
	result.dominant_freq = 0.0f;
	result.tremor_detected = false;
	result.dyskinesia_detected = false;
	result.tremor_power = 0.0f;
	result.dyskinesia_power = 0.0f;
	
	if (!fft_initialized || length != FFT_SIZE){
		return result;
	}
	
	// Get dominant frequency
	result.dominant_freq = getDominantFrequency(data, length);
	
	// Get power in tremor band (3-5Hz)
	result.tremor_power = getBandPower(data, length, TREMOR_LOW_HZ, TREMOR_HIGH_HZ);
	
	// Get power in dysk band (5-7Hz)
	result.dyskinesia_power = getBandPower(data, length, DYSK_LOW_HZ, DYSK_HIGH_HZ);
	
	// Get total power for normalization
	float total_power = getBandPower(data, length, 0.5f, 15.0f);
	
	// Threshold for detection (TENTATIVE VALUE)
	const float DETECTION_THRESHOLD = 0.15f;	// 15% of total power
	
	// Classify based on dominant frequency and band power
	// Tremor
	if (result.tremor_power / total_power > DETECTION_THRESHOLD){
		if (result.dominant_freq >= TREMOR_LOW_HZ &&
			result.dominant_freq <= TREMOR_HIGH_HZ){
				result.tremor_detected = true;
			}
	}
	
	// Dysk
	if (result.dyskinesia_power / total_power > DETECTION_THRESHOLD){
		if (result.dominant_freq >= DYSK_LOW_HZ &&
			result.dominant_freq <= DYSK_HIGH_HZ){
				result.dyskinesia_detected = true;
			}
	}
	
	return result;
}


// Calculate magnitude of 3-axis accelerometer data
// mag = sqrt(x^2 + y^2 + z^2)
// combine all three into a single value

void FrequencyAnalyzer::calculateMagnitude(float* acc_x, float* acc_y, float* acc_z,
											float* output, uint32_t length){
	for (uint32_t i = 0; i < length; i++){
		float x2 = acc_x[i]* acc_x[i];
		float y2 = acc_y[i]* acc_y[i];
		float z2 = acc_z[i]* acc_z[i];
		arm_sqrt_f32(x2+y2+z2, &output[i]);
	}											
}

// Trouble Shooting Functions

// Get current FFT SIZE configuration
uint32_t FrequencyAnalyzer::getFFTSize(){
	return FFT_SIZE;
}

// Get current sample rate
uint32_t FrequencyAnalyzer::getSampleRate(){
	return FFT_SIZE;
}

// get frequency resolution (Hz per bin)
uint32_t FrequencyAnalyzer::getFrequencyResolution(){
	return SAMPLE_RATE / FFT_SIZE;
}































