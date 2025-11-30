#include "mbed.h"
#include "arm_math.h" 

// --- 1. CONFIGURATION (Keep these at the top!) ---
#define SENSORS_SDA PB_11
#define SENSORS_SCL PB_10
#define LSM6DSL_ADDR 0xD4 

#define FFT_SIZE 128            // Total buffer size
#define NEW_SAMPLES 10          // <--- HERE IS THE SLIDING WINDOW SIZE (0.2 seconds)
#define SAMPLING_FREQ 52.0f     
#define PERIOD_US 19230         // 1/52Hz = ~19230 microseconds

// --- 2. GLOBAL OBJECTS ---
BufferedSerial pc(USBTX, USBRX);
I2C i2c(SENSORS_SDA, SENSORS_SCL);
Timer timer;
arm_rfft_fast_instance_f32 S;

// --- 3. BUFFERS ---
float inputData[FFT_SIZE];      // Main data history
float fftOutput[FFT_SIZE];      // Complex FFT Output
float fftMag[FFT_SIZE / 2];     // Magnitude Output

// --- 4. SENSOR HELPERS ---
void initSensor() {
    i2c.frequency(400000);
    char cmd[2] = {0x10, 0x60}; // 416Hz ODR, 2g, 200Hz Filter
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

// --- 5. MAIN ---
int main() {
    // CRITICAL: Set baud rate first!
    pc.set_baud(115200);
    
    printf("--- FOG Detection Ready (Sliding Window) ---\r\n");
    
    initSensor();
    timer.start();
    arm_rfft_fast_init_f32(&S, FFT_SIZE);

    // Pre-fill buffer to avoid initial zeros
    for(int i=0; i<FFT_SIZE; i++) inputData[i] = 1.0f; 

    while (true) {
        // --- STEP A: SHIFT OLD DATA ---
        // Move history to the left to make room for new samples
        for (int i = 0; i < (FFT_SIZE - NEW_SAMPLES); i++) {
            inputData[i] = inputData[i + NEW_SAMPLES];
        }

        // --- STEP B: COLLECT NEW DATA (Fast!) ---
        for (int i = (FFT_SIZE - NEW_SAMPLES); i < FFT_SIZE; i++) {
            int start_t = timer.read_us();
            
            float x, y, z;
            readAccel(x, y, z);
            inputData[i] = sqrtf(x*x + y*y + z*z);
            
            // Wait to keep 52Hz timing
            while ((timer.read_us() - start_t) < PERIOD_US);
        }

        // --- STEP C: PROCESS FFT ---
        // Copy to temp buffer (FFT breaks original data)
        float fftBuffer[FFT_SIZE];
        memcpy(fftBuffer, inputData, sizeof(inputData));

        float mean;
        arm_mean_f32(fftBuffer, FFT_SIZE, &mean);
        arm_offset_f32(fftBuffer, -mean, fftBuffer, FFT_SIZE);
        arm_rfft_fast_f32(&S, fftBuffer, fftOutput, 0);
        arm_cmplx_mag_f32(fftOutput, fftMag, FFT_SIZE / 2);

        // --- STEP D: ENERGY ---
        float loco_energy = 0;
        float freeze_energy = 0;

        for (int i = 1; i < (FFT_SIZE / 2); i++) {
            float freq = i * 0.406f;
            if (freq >= 0.5f && freq <= 3.0f) loco_energy += fftMag[i];
            if (freq > 3.0f && freq <= 8.0f)  freeze_energy += fftMag[i];
        }

        // --- STEP E: LOGIC ---
        float total_energy = loco_energy + freeze_energy;
        
        // Prevent divide by zero
        if (loco_energy < 0.01f) loco_energy = 0.01f;
        float index = freeze_energy / loco_energy;

        // GATE: Check if total movement is significant
        if (total_energy < 1.0f) {
             printf("Status: RESTING  | Index: %d.%02d\r\n", 
                    (int)index, abs((int)((index - (int)index) * 100)));
        } 
        else {
            // MOVEMENT DETECTED: Check for Freeze Ratio
            // Threshold 2.0 = Twice as much shaking as walking
            if (index > 2.0f) { 
                 printf(">>> FREEZE DETECTED! <<< | Index: %d.%02d\r\n", 
                        (int)index, abs((int)((index - (int)index) * 100)));
            } else {
                 printf("Status: WALKING          | Index: %d.%02d\r\n", 
                        (int)index, abs((int)((index - (int)index) * 100)));
            }
        }
        
        fflush(stdout); 
    }
}