// Integration
// implement FrequencyAnalyzer module
// detect tremors and dysk in real time

#include <mbed.h>
#include "frequency_analyzer.h"

// I2C and sensor configuration
I2C i2c(PB_11, PB_10);
InterruptIn int1(PD_11, PullDown);

#define LSM6DSL_ADDR	(0x6A << 1)
#define WHO_AM_I		0x0F
#define CTRL1_XL		0x10
#define CTRL2_G			0x11
#define CTRL3_C			0x12
#define DRDY_PULSE_CFG	0x0B
#define INT1_CTRL		0x0D
#define STATUS_REG		0x1E
#define OUTX_L_XL		0x28

// circular buffer for accelerometer data
#define BUFFER_SIZE 256	// Must match FFT_SIZE
float acc_x_buffer[BUFFER_SIZE];
float acc_y_buffer[BUFFER_SIZE];
float acc_z_buffer[BUFFER_SIZE];
float acc_mag_buffer[BUFFER_SIZE];

volatile uint32_t buffer_idx = 0;
volatile bool buffer_full = false;
volatile bool data_ready = false;

// ISR for data ready interrupt
void data_ready_isr(){
	data_ready = true;
}

// I2C Helper funcs
bool write_reg(uint8_t reg, uint8_t val){
	char buf[2] = {(char)reg, (char)val};
	return (i2c.write(LSM6DSL_ADDR, buf, 2) == 0);
}

bool read_reg(uint8_t reg, uint8_t &val){
	char r = (char)reg;
	if (i2c.write(LSM6DSL_ADDR, &r, 1, true) != 0) return false;
	if (i2c.read(LSM6DSL_ADDR, &r, 1) != 0) return false;
	val = (uint8_t)r;
	return true;
}

bool read_int16(uint8 reg_low, int16_t &val){
	uint8_t lo, hi;
	if (!read_reg(reg_low,lo)) return false;
	if (!read_reg(reg_low + 1, hi)) return false;
	val = (int16_t)((hi << 8) | lo);
	return true;
}

// Init LSM6DSL Sensor
bool init_sensor(){
	uint8_t who;
	if (!read_reg(WHO_AM_I, who)|| who!=0x6A){
		printf("Sensor not found!\r\n");
		return false;
	}
	
	// configure sensor for 104Hz sampling
	write_reg(CTRL3_C, 0x44);
	write_reg(CTRL1_XL, 0x54);
	write_reg(CTRL2_G, 0x50);
	write_reg(INT1_CTRL, 0x03);
	write_reg(DRDY_PULSE_CFG, 0x80);
	
	ThisThread::sleep_for(100ms);
	
	// Clear old data
	uint8_t dummy;
	read_reg(STATUS_REG, dummy);
	int16_t temp;
	for(int i = 0; i < 6; i++){
		read_int16(OUTX_L_XL + i*2, temp);
	}
	
	// Attach interrupt
	int1.rise(&data_ready_isr);
	
	return true;
}

// read sensor and add to buffer
void read_and_buffer_data(){
	int16_t acc_raw[3];
	
	// Read 3-axis accelerometer data
	for (int i = 0; i < 3; i++){
		read_int16(OUTX_L_XL + i * 2, acc_raw[i]);
	}
	
	// Convert to g (+/- 1g range: 0.488 mg/LSB)
	float ax = acc_raw[0] * 0.000488f;
	float ay = acc_raw[1] * 0.000488f;
	float az = acc_raw[2] * 0.000488f;
	
	// Add to circular buffer
	acc_x_buffer[buffer_idx] = ax;
	acc_y_buffer[buffer_idx] = ay;
	acc_z_buffer[buffer_idx] = az;
	
	buffer_idx++;
	
	// Check if buffer is full
	if (buffer_idx >= BUFFER_SIZE){
		buffer_idx = 0;
		buffer_full = true;
	}
}

// Analyze buffer and detect tremor/dyskinesia
void analyze_movement(){
	if(!buffer_full){
		printf("Collecting data")
		return;
	}
	
	// Calculate magnitude of acceleration
	FrequencyAnalyzer::calculateMagnitude(acc_x_buffer, acc_y_buffer, acc_z_buffer,
											acc_mag_buffer, BUFFER_SIZE);
	// Perform frequency analysis
	MovementClassification result = FrequencyAnalyzer::analyzeMovement(acc_mag_buffer, BUFFER_SIZE);
	
    // Print results
    printf("\r\n=== Movement Analysis ===\r\n");
    printf("Dominant Frequency: %.2f Hz\r\n", result.dominant_freq);
    printf("Tremor Power: %.3f\r\n", result.tremor_power);
    printf("Dyskinesia Power: %.3f\r\n", result.dyskinesia_power);
    
    if (result.tremor_detected) {
        printf(">>> TREMOR DETECTED (3-5 Hz) <<<\r\n");
    }
    
    if (result.dyskinesia_detected) {
        printf(">>> DYSKINESIA DETECTED (5-7 Hz) <<<\r\n");
    }
    
    if (!result.tremor_detected && !result.dyskinesia_detected) {
        printf("No abnormal movement detected\r\n");
    }
    
    printf("=======================\r\n\r\n");
    
    // Optional: Send to Teleplot for visualization
    printf(">dominant_freq:%.2f\n", result.dominant_freq);
    printf(">tremor_power:%.3f\n", result.tremor_power);
    printf(">dysk_power:%.3f\n", result.dyskinesia_power);
}

int main(){
	static BufferedSerial pc(USBTX, USBR, 115200);
	
	printf("=== Tremor & Dyskinesia Detection System ===\r\n");
	
	// init I2C
	i2c.frequency(400000);
	
	// init sensor
	if (!init_sensor()){
		printf("Failed to initialize sensor!\r\n");
		while(1){ThisThread::sleep_for(1s);}
	}
	printf("LSMDSL sensor initialized (256-point FFT)\r\n");
    printf("Frequency resolution: %.3f Hz/bin\r\n", FrequencyAnalyzer::getFrequencyResolution());
    printf("\r\nCollecting data...\r\n\r\n");
    
    // Timer for periodic analysis
    Timer for periodic analysis
    Timer analysis_timer;
    analysis_timer.start();
    
    while(true){
    	// read data when available
    	if (data_ready){
    		data_ready = false;
    		read_and_buffer_data();
    	}
    	
    	// Perform analysis every 3s
    	if (analysis_timer.elapsed_time() >= 3s){
    		analysis_timer.reset();
    		analyze_movement();
    	}
    	ThisThread::sleep_for(1ms);
    }
}

/* 
 * PLATFORMIO CONFIGURATION (platformio.ini)
 * ==========================================
 * 
 * Add this to platformio.ini file:
 * 
 * [env:disco_l475vg_iot01a]
 * platform = ststm32
 * board = disco_l475vg_iot01a
 * framework = mbed
 * 
 * lib_deps = 
 *     ARM-software/CMSIS-DSP@^1.15.0
 * 
 * build_flags = 
 *     -DARM_MATH_CM4
 *     -D__FPU_PRESENT=1
 *     -DARM_MATH_LOOPUNROLL
 * 
 */









