#include "mbed.h"
#include "fog_service.h" 

// --- HARDWARE CONFIGURATION ---
#define SENSORS_SDA PB_11
#define SENSORS_SCL PB_10
#define LSM6DSL_ADDR 0xD4 
#define PERIOD_US 19230 

BufferedSerial pc(USBTX, USBRX);
I2C i2c(SENSORS_SDA, SENSORS_SCL);
Timer timer;

FOG_Service fogDetector; 

// --- SENSOR DRIVER ---
void initSensor() {
    i2c.frequency(400000);
    char cmd[2] = {0x10, 0x60}; 
    i2c.write(LSM6DSL_ADDR, cmd, 2);
}

void readAccel(float &x, float &y, float &z) {
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

int main() {
    pc.set_baud(115200);
    printf("--- SYSTEM START ---\r\n");

    initSensor();
    
    // Initialize Services
    fogDetector.init();
    
    timer.start();

    while (true) {
        int start_t = timer.read_us();

        // 1. Read Sensor ONCE
        float x, y, z;
        readAccel(x, y, z);

        // 2. Pass data to FOG Service
        // It returns 'true' only every 10th sample (when it finishes math)
        if (fogDetector.update(x, y, z)) {
            
            // 3. Check Results
            if (fogDetector.is_frozen) {
                printf(">>> FREEZE DETECTED! (Index: %d.%02d)\r\n", 
                       (int)fogDetector.current_index, 
                       abs((int)((fogDetector.current_index - (int)fogDetector.current_index) * 100)));
            } else if (fogDetector.is_resting) {
                printf("Resting...\r\n");
            } else {
                printf("Walking...\r\n");
            }
        }
        
        // 4. Pass data to Tremor Service (Your teammate will add this)
        // tremorDetector.update(x, y, z);

        // 5. Wait for 52Hz
        while ((timer.read_us() - start_t) < PERIOD_US);
    }
}