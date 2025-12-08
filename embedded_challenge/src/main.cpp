///////////////////////////////////////////////////////// BLE APPLICATION //////////////////////////////////////////////////////

// Combined Parkinson's Disease Monitoring System with BLE
// Detects FOG, Tremor, and Dyskinesia and broadcasts via Bluetooth

#include "mbed.h"
#include "ble/BLE.h"
#include "ble/gatt/GattService.h"
#include "ble/gatt/GattCharacteristic.h"
#include "ble/Gap.h"
#include "ble/gap/AdvertisingDataBuilder.h"
#include "events/EventQueue.h"
#include "fog_service.h"
#include "tremor_service.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace ble;
using namespace events;

// Hardware Config
#define SENSORS_SDA PB_11
#define SENSORS_SCL PB_10
#define LSM6DSL_ADDR 0xD4
#define PERIOD_US 19230  // 52 Hz sampling

BufferedSerial pc(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &pc;
}

I2C i2c(SENSORS_SDA, SENSORS_SCL);
Timer timer;
DigitalOut led(LED1);

// Service instances
FOG_Service fogDetector;
Tremor_Service tremorDetector;

// BLE setup
BLE &ble_interface = BLE::Instance();
EventQueue event_queue;

// BLE UUIDs
const UUID PD_SERVICE_UUID("A0E1B2C3-D4E5-F6A7-B8C9-D0E1F2A3B4C5");
const UUID STATUS_CHAR_UUID("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C6");

// Status strings
const char* STATUS_NONE = "NONE";
const char* STATUS_FOG = "FOG";
const char* STATUS_TREMOR = "TREMOR";
const char* STATUS_DYSKINESIA = "DYSKINESIA";
const char* STATUS_RESTING = "RESTING";

#define MAX_STATUS_STRING_LEN 15
uint8_t statusValue[MAX_STATUS_STRING_LEN];

// BLE Characteristic
ReadOnlyArrayGattCharacteristic<uint8_t, MAX_STATUS_STRING_LEN> statusCharacteristic(
    STATUS_CHAR_UUID,
    statusValue,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

GattCharacteristic *charTable[] = { &statusCharacteristic };
GattService pdService(PD_SERVICE_UUID, charTable, 1);

// Current detection state
char currentStatus[MAX_STATUS_STRING_LEN] = "NONE";

// Sensor Driver Functions
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

// Update BLE status
void updateBLEStatus(const char* newStatus) {
    if (strcmp(currentStatus, newStatus) != 0) {
        strcpy(currentStatus, newStatus);
        strcpy((char*)statusValue, newStatus);
        
        ble_interface.gattServer().write(
            statusCharacteristic.getValueHandle(),
            statusValue,
            strlen((char*)statusValue) + 1
        );
        
        led = !led;
        printf("[BLE] Status updated: %s\n", newStatus);
    }
}

// Sensor processing task
void processSensors() {
    float x, y, z;
    readAccel(x, y, z);
    
    bool fog_ready = fogDetector.update(x, y, z);
    bool tremor_ready = tremorDetector.update(x, y, z);
    
    if (fog_ready && tremor_ready) {
        printf("\r\n========== Analysis Update ==========\r\n");
        
        bool is_resting = fogDetector.is_resting || tremorDetector.is_resting;
        
        if (is_resting) {
            printf("[STATUS] Patient at rest\r\n");
            printf("[FOG]    Not walking\r\n");
            printf("[TREMOR] Not detected (at rest)\r\n");
            printf("[DYSK]   Not detected (at rest)\r\n");
            updateBLEStatus(STATUS_RESTING);
        } else {
            int fog_index_int = (int)(fogDetector.current_index * 1000);
            int freq_int = (int)(tremorDetector.dominant_freq * 100);
            int dysk_power_int = (int)(tremorDetector.dyskinesia_power * 10000);
            int tremor_power_int = (int)(tremorDetector.tremor_power * 10000);
            
            float freq = tremorDetector.dominant_freq;
            
            bool fog_detected = fogDetector.is_frozen;
            bool tremor_detected = tremorDetector.is_tremor;
            bool dysk_detected = tremorDetector.is_dyskinesia;
            
            bool in_fog_range = (freq >= 5.0f && freq <= 7.0f);
            bool in_tremor_range = (freq >= 3.0f && freq <= 5.0f);
            
            if (fog_detected && in_fog_range) {
                printf("[FOG]    >>> FREEZE DETECTED! <<<\r\n");
                printf("         Freeze Index: %d.%03d\r\n", fog_index_int/1000, abs(fog_index_int%1000));
                printf("         Frequency: %d.%02d Hz\r\n", freq_int/100, abs(freq_int%100));
                printf("[TREMOR] Not detected (FOG priority)\r\n");
                printf("[DYSK]   Not detected (FOG priority)\r\n");
                updateBLEStatus(STATUS_FOG);
                
            } else if (dysk_detected && in_fog_range && !fog_detected) {
                printf("[FOG]    Walking normally\r\n");
                printf("         Freeze Index: %d.%03d\r\n", fog_index_int/1000, abs(fog_index_int%1000));
                printf("[TREMOR] Not detected (freq in dysk range)\r\n");
                printf("[DYSK]   >>> DETECTED! <<<\r\n");
                printf("         Frequency: %d.%02d Hz\r\n", freq_int/100, abs(freq_int%100));
                printf("         Power: %d.%04d\r\n", dysk_power_int/10000, abs(dysk_power_int%10000));
                updateBLEStatus(STATUS_DYSKINESIA);
                
            } else if (tremor_detected && in_tremor_range) {
                printf("[FOG]    Walking normally\r\n");
                printf("         Freeze Index: %d.%03d\r\n", fog_index_int/1000, abs(fog_index_int%1000));
                printf("[TREMOR] >>> DETECTED! <<<\r\n");
                printf("         Frequency: %d.%02d Hz\r\n", freq_int/100, abs(freq_int%100));
                printf("         Power: %d.%04d\r\n", tremor_power_int/10000, abs(tremor_power_int%10000));
                printf("[DYSK]   Not detected\r\n");
                updateBLEStatus(STATUS_TREMOR);
                
            } else if (fog_detected && !in_fog_range) {
                // FALSE POSITIVE
                printf("[FOG]    Low frequency but not FOG (freq: %d.%02d Hz, need 5-7Hz)\r\n", 
                       freq_int/100, abs(freq_int%100));
                printf("         Freeze Index: %d.%03d\r\n", fog_index_int/1000, abs(fog_index_int%1000));
                printf("[TREMOR] Not detected\r\n");
                printf("[DYSK]   Not detected\r\n");
                updateBLEStatus(STATUS_NONE);
                
            } else {
                printf("[FOG]    Walking normally\r\n");
                printf("         Freeze Index: %d.%03d\r\n", fog_index_int/1000, abs(fog_index_int%1000));
                printf("[TREMOR] Not detected\r\n");
                printf("         Frequency: %d.%02d Hz\r\n", freq_int/100, abs(freq_int%100));
                printf("[DYSK]   Not detected\r\n");
                printf("         Power: %d.%04d\r\n", dysk_power_int/10000, abs(dysk_power_int%10000));
                updateBLEStatus(STATUS_NONE);
            }
        }
        
        printf("=====================================\r\n");
    }
}

// BLE initialization callback
void on_ble_init_complete(BLE::InitializationCompleteCallbackContext *params) {
    if (params->error != BLE_ERROR_NONE) {
        printf("BLE initialization failed.\n");
        return;
    }
    
    strcpy((char*)statusValue, STATUS_NONE);
    ble_interface.gattServer().addService(pdService);
    
    uint8_t adv_buffer[LEGACY_ADVERTISING_MAX_SIZE];
    AdvertisingDataBuilder adv_data(adv_buffer);
    adv_data.setFlags();
    adv_data.setName("PD-Monitor");
    
    ble_interface.gap().setAdvertisingParameters(
        LEGACY_ADVERTISING_HANDLE,
        AdvertisingParameters(advertising_type_t::CONNECTABLE_UNDIRECTED, adv_interval_t(160))
    );
    
    ble_interface.gap().setAdvertisingPayload(LEGACY_ADVERTISING_HANDLE, adv_data.getAdvertisingData());
    ble_interface.gap().startAdvertising(LEGACY_ADVERTISING_HANDLE);
    
    printf("BLE advertising started as PD-Monitor\n");
}

// BLE event scheduler
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(callback(&ble_interface, &BLE::processEvents));
}

int main(){
    pc.set_baud(115200);
    printf("\r\n");
    printf("===========================================\r\n");
    printf("  Parkinson's Disease Monitoring System   \r\n");
    printf("  with Bluetooth Low Energy (BLE)         \r\n");
    printf("===========================================\r\n");
    printf("  FOG Detection:    ENABLED\r\n");
    printf("  Tremor Detection: ENABLED\r\n");
    printf("  Dyskinesia:       ENABLED\r\n");
    printf("  BLE Broadcasting: ENABLED\r\n");
    printf("  Sample Rate:      52 Hz\r\n");
    printf("===========================================\r\n\r\n");
    
    // Initialize sensor
    initSensor();
    printf("LSM6DSL Sensor Initialized \r\n");
    
    // Initialize detection services
    fogDetector.init();
    printf("FOG Service Initialized \r\n");
    
    tremorDetector.init();
    printf("Tremor Service Initialized \r\n");
    
    // Initialize BLE
    ble_interface.onEventsToProcess(schedule_ble_events);
    ble_interface.init(on_ble_init_complete);
    printf("BLE Service Initialized \r\n\r\n");
    
    timer.start();
    
    // Main loop: process sensors at 52Hz, BLE events handled by event queue
    while(true){
        auto start_t = timer.elapsed_time();
        
        // Process sensor data
        processSensors();
        
        // Maintain 52Hz timing
        while((timer.elapsed_time() - start_t).count() < PERIOD_US);
        
        // Process BLE events (non-blocking)
        event_queue.dispatch(0);
    }
}