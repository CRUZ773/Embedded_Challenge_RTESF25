// Header File for Frequency Domain Analysis
// Tremor (3-5Hz) and Dyskinesia (5-7Hz) Detection

#ifndef FREQUENCY_ANALYZER_H
#define FREQUENCY_ANALYZER_H

#include <stdint.h>

// Result structure for movement classification
struct MovementClassification{
	float dominant_freq;		// Dominant frequency in Hz
	bool tremor_detected;		// True if tremor detected in 3-5Hz range
	bool dyskinesia_detected;	// True if dysk detected in 5-7Hz range
	float tremor_power;			// Power in tremor band
	float dyskinesia_power;		// Power in dyskinesia band
}

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