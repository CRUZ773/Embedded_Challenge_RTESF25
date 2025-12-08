
// Integrated FOG detection + Tremor/Dyskinesia detection
// FOG takes priority (5-7Hz), then Tremor (3-5Hz) and Dyskinesia (3-5Hz)

#include "mbed.h"
#include "fog_service.h"
#include "tremor_service.h"
#include <stdio.h>
#include <stdlib.h>

// Hardware Config
#define SENSORS_SDA PB_11
#define SENSORS_SCL PB_10
#define LSM6DSL_ADDR 0xD4
#define PERIOD_US 19230     // 52 Hz sampling

BufferedSerial pc(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &pc;
}

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

void flush_serial(){
    fflush(stdout);
}

int main(){
    pc.set_baud(115200);
    printf("\r\n");
    printf("  Parkinson's Disease Monitoring System   \r\n");
    printf("===========================================\r\n");
    printf("  FOG Detection:    ENABLED\r\n");
    printf("  Tremor Detection: ENABLED\r\n");
    printf("  Dyskinesia:       ENABLED\r\n");
    printf("  Sample Rate:      52 Hz\r\n");

    
    initSensor();
    printf("LSM6DSL Sensor Initialized \r\n");
    
    // init both services
    fogDetector.init();
    printf("FOG Service Initialized \r\n");

    tremorDetector.init();
    printf("Tremor Service Initialized \r\n");

    timer.start();
    
    while(true){
        auto start_t = timer.elapsed_time();
        
        // 1. read sensor data ONCE
        float x, y, z;
        readAccel(x, y, z);
        
        // 2. update both services
        bool fog_ready = fogDetector.update(x, y, z);
        bool tremor_ready = tremorDetector.update(x, y, z);
        
        // 3. display results when both are ready
        if (fog_ready && tremor_ready) {
            printf("\r\n========== Analysis Update ==========\r\n");
            
            // Check for resting state first (applies to all)
            bool is_resting = fogDetector.is_resting || tremorDetector.is_resting;
            
            if (is_resting) {
                printf("[STATUS] Patient at rest\r\n");
                printf("[FOG]    Not walking\r\n");
                printf("[TREMOR] Not detected (at rest)\r\n");
                printf("[DYSK]   Not detected (at rest)\r\n");
            } else {
                // Convert floats to integers for display
                int fog_index_int = (int)(fogDetector.current_index * 1000);
                int freq_int = (int)(tremorDetector.dominant_freq * 100);
                int dysk_power_int = (int)(tremorDetector.dyskinesia_power * 10000);
                int tremor_power_int = (int)(tremorDetector.tremor_power * 10000);
                
                // Get actual frequency value
                float freq = tremorDetector.dominant_freq;
                
                // Classification based on dominant frequency and detection flags
                // Priority: FOG (5-7Hz) > Dyskinesia (5-7Hz) > Tremor (3-5Hz)
                
                bool fog_detected = fogDetector.is_frozen;
                bool tremor_detected = tremorDetector.is_tremor;
                bool dysk_detected = tremorDetector.is_dyskinesia;
                
                // Determine what's actually happening based on frequency
                bool in_fog_range = (freq >= 5.0f && freq <= 7.0f);
                bool in_tremor_range = (freq >= 3.0f && freq <= 5.0f);
                
                if (fog_detected && in_fog_range) {
                    // TRUE FOG: High freeze index AND frequency in 5-7Hz
                    printf("[FOG]    >>> FREEZE DETECTED! <<<\r\n");
                    printf("         Freeze Index: %d.%03d\r\n", fog_index_int/1000, abs(fog_index_int%1000));
                    printf("         Frequency: %d.%02d Hz\r\n", freq_int/100, abs(freq_int%100));
                    printf("[TREMOR] Not detected (FOG priority)\r\n");
                    printf("[DYSK]   Not detected (FOG priority)\r\n");
                    
                } else if (dysk_detected && in_fog_range && !fog_detected) {
                    // DYSKINESIA in 5-7Hz range (but no FOG)
                    printf("[FOG]    Walking normally\r\n");
                    printf("         Freeze Index: %d.%03d\r\n", fog_index_int/1000, abs(fog_index_int%1000));
                    printf("[TREMOR] Not detected (freq in dysk range)\r\n");
                    printf("[DYSK]   >>> DETECTED! <<<\r\n");
                    printf("         Frequency: %d.%02d Hz\r\n", freq_int/100, abs(freq_int%100));
                    printf("         Power: %d.%04d\r\n", dysk_power_int/10000, abs(dysk_power_int%10000));
                    
                } else if (tremor_detected && in_tremor_range) {
                    // TREMOR in 3-5Hz range
                    printf("[FOG]    Walking normally\r\n");
                    printf("         Freeze Index: %d.%03d\r\n", fog_index_int/1000, abs(fog_index_int%1000));
                    printf("[TREMOR] >>> DETECTED! <<<\r\n");
                    printf("         Frequency: %d.%02d Hz\r\n", freq_int/100, abs(freq_int%100));
                    printf("         Power: %d.%04d\r\n", tremor_power_int/10000, abs(tremor_power_int%10000));
                    printf("[DYSK]   Not detected\r\n");
                    
                } else if (fog_detected && !in_fog_range) {
                    // False positive - maybe we have to tweak this to see if our parameters have to be more accurate
                    printf("[FOG]    High FOG index, low freq --> (freq: %d.%02d Hz, need 5-7Hz)\r\n", 
                           freq_int/100, abs(freq_int%100));
                    printf("         Freeze Index: %d.%03d\r\n", fog_index_int/1000, abs(fog_index_int%1000));
                    printf("[TREMOR] Not detected\r\n");
                    printf("[DYSK]   Not detected\r\n");
                    
                } else {
                    // NORMAL MOVEMENT
                    printf("[FOG]    Walking normally\r\n");
                    printf("         Freeze Index: %d.%03d\r\n", fog_index_int/1000, abs(fog_index_int%1000));
                    printf("[TREMOR] Not detected\r\n");
                    printf("         Frequency: %d.%02d Hz\r\n", freq_int/100, abs(freq_int%100));
                    printf("[DYSK]   Not detected\r\n");
                    printf("         Power: %d.%04d\r\n", dysk_power_int/10000, abs(dysk_power_int%10000));
                }
            }
            
            printf("=====================================\r\n");
        }
    
        // 5. Maintain 52Hz timing
        while((timer.elapsed_time() - start_t).count() < PERIOD_US);
    }  
}
