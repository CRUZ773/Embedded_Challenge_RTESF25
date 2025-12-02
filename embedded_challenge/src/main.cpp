// combines FOG detection + Tremor/Dyskinesia detection
// both services run simultaneously on the same sensor data

#include "mbed.h"
#include "fog_service.h"
#include "tremor_service.h"

// Hardware Config
#define SENSORS_SDA PB_11
#define SENSORS_SCL PB_10
#define LSM6DSL_ADDR 0xD4
#define PERIOD_US 19230		// 52 Hz sampling

BufferedSerial pc(USBTX, USBRX);
I2C i2c(SENSORS_SDA, SENSORS_SCL);
Timer timer;

// Service instances
FOG_Service fogDetector;
Tremor_Service tremorDetector;

// Sensor Driver
void initSensor(){
	i2c.frequency(400000);
	char cmd[2] = {0x10, 0x60};
	i2c.write(LSM6DSL_ADDR, cmd, 2);
} 

void readAccel(float &x, float &y, float &z){
	char reg = 0x28;
	char data[6];
	i2c.write(LSM6DSL_ADDR, &reg, 1, true);
	i2c.read(LSM6DSL_ADDR, data, 6);
	int16_t rawX = (data[1] << 8) | data[0];
	int16_t rawY = (data[3] << 8) | data[2];
	int16_t rawZ = (data[5] << 8) | data[4];
	x = rawX * 0.000061f;
	y = rawY * 0.000061f;
	z = rawZ * 0.000061f;
}

int main(){
	pc.set_baud(115200);
	printf("\r\n");
    printf("===========================================\r\n");
    printf("  Parkinson's Disease Monitoring System   \r\n");
    printf("===========================================\r\n");
    printf("  FOG Detection:    ENABLED\r\n");
    printf("  Tremor Detection: ENABLED\r\n");
    printf("  Dyskinesia:       ENABLED\r\n");
    printf("  Sample Rate:      52 Hz\r\n");
    printf("===========================================\r\n\r\n");
	
	initSensor();
	printf("LSM6DSL Sensor Initialized \r\n");
	
	// init both services
	fogDetector.init();
	printf("FOG Service Initialized \r\n");

	tremorDetector.init();
	printf("Tremor Service Initialized \r\n");

	timer.start();
	
	while(true){
		int start_t = timer.read_us();
		
		// 1. read sensor data ONCE
		float x, y, z;
		readAccel(x,y,z);
		
		// 2. update FOG service
		bool fog_ready = fogDetector.update(x,y,z);
		
		// 3. update tremor service (using same data)
		bool tremor_ready = tremorDetector.update(x,y,z);
		
		// 4. display results when both are ready
		// Note: both process every 10th sample, so they'll be ready together
        if (fog_ready && tremor_ready) {
            
            printf("\r\n--- Analysis Update ---\r\n");
            
            // Display FOG status
            printf("[FOG] ");
            if (fogDetector.is_frozen) {
                printf(">>> FREEZE DETECTED! Index: %.2f\r\n", fogDetector.current_index);
            } else if (fogDetector.is_resting) {
                printf("Resting\r\n");
            } else {
                printf("Walking (Index: %.2f)\r\n", fogDetector.current_index);
            }
            
            // Display Tremor status
            printf("[TREMOR] ");
            if (tremorDetector.is_tremor) {
                printf(">>> TREMOR DETECTED! (%.2f Hz)\r\n", tremorDetector.dominant_freq);
            } else if (tremorDetector.is_resting) {
                printf("At rest\r\n");
            } else {
                printf("Normal movement (%.2f Hz)\r\n", tremorDetector.dominant_freq);
            }
            
            // Display Dyskinesia status
            printf("[DYSK] ");
            if (tremorDetector.is_dyskinesia) {
                printf(">>> DYSKINESIA DETECTED! (%.2f Hz)\r\n", tremorDetector.dominant_freq);
            } else if (!tremorDetector.is_resting) {
                printf("Normal (Power: %.3f)\r\n", tremorDetector.dyskinesia_power);
            } else {
                printf("At rest\r\n");
            }
            
            printf("----------------------\r\n");
            
            // Optional: Teleplot output for visualization
            printf(">fog_index:%.2f\n", fogDetector.current_index);
            printf(">fog_energy:%.2f\n", fogDetector.current_energy);
            printf(">tremor_freq:%.2f\n", tremorDetector.dominant_freq);
            printf(">tremor_power:%.3f\n", tremorDetector.tremor_power);
            printf(">dysk_power:%.3f\n", tremorDetector.dyskinesia_power);
	}
	
	// 5. Maintain 52Hz timing
	while((timer.read_us() - start_t) < PERIOD_US);
}