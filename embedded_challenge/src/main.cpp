// combines FOG detection + Tremor/Dyskinesia detection
// both services run simultaneously on the same sensor data

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
#define PERIOD_US 19230		// 52 Hz sampling

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
BLE & ble_interface = BLE::Instance();
EventQueue event_queue;

// BLE UUIDs - One service, 3 characteristics
const UUID PD_SERVICE_UUID("A0E1B2C3-D4E5-F6A7-B8C9-D0E1F2A3B4C5");
const UUID FOG_CHAR_UUID("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C6");      // FOG characteristic
const UUID TREMOR_CHAR_UUID("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C7");   // Tremor characteristic
const UUID DYSK_CHAR_UUID("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C8");     // Dyskinesia characteristic

// Data buffers for each characteristic
#define MAX_VALUE_LEN 20
uint8_t fogValue[MAX_VALUE_LEN];
uint8_t tremorValue[MAX_VALUE_LEN];
uint8_t dyskValue[MAX_VALUE_LEN];

// THREE BLE Characteristics (one for each condition)
ReadOnlyArrayGattCharacteristic<uint8_t, MAX_VALUE_LEN> fogCharacteristic(
	FOG_CHAR_UUID,
	fogValue,
	GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ
);

ReadOnlyArrayGattCharacteristic<uint8_t, MAX_VALUE_LEN> tremorCharacteristic(
	TREMOR_CHAR_UUID,
	tremorValue,
	GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ
);

ReadOnlyArrayGattCharacteristic<uint8_t, MAX_VALUE_LEN> dyskCharacteristic(
	DYSK_CHAR_UUID,
	dyskValue,
	GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ
);

// Characteristic table with all three
GattCharacteristic *charTable[] = {&fogCharacteristic, &tremorCharacteristic, &dyskCharacteristic};
GattService pdService(PD_SERVICE_UUID, charTable, 3);

// Check BLE connection status
class MyGapEventHandler : public Gap::EventHandler{
	virtual void onConnectionComplete(const ConnectionCompleteEvent &event){
		printf("BLE Connect! Handle: %d\n", event.getConnectionHandle());
		led = 1;
	}
	
	virtual void onDisconnectionComplete(const DisconnectionCompleteEvent &event){
		printf("BLE Disconnected\n");
		led = 0;
		ble_interface.gap().startAdvertising(LEGACY_ADVERTISING_HANDLE);
	}
};

MyGapEventHandler gapHandler;

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

// Update individual BLE characteristic

// Send Fog Update
void updateFOGCharacteristic(bool detected, float index, float freq){
	char buffer[MAX_VALUE_LEN];
	if (detected){
        snprintf(buffer, MAX_VALUE_LEN, "FOG:1,%.2f,%.2f", index, freq);
	} else{
        snprintf(buffer, MAX_VALUE_LEN, "FOG:0,%.2f,%.2f", index, freq);
		
	}
	
	memcpy(fogValue, buffer, strlen(buffer) + 1);
	ble_interface.gattServer().write(
		fogCharacteristic.getValueHandle(),
		fogValue,
		strlen(buffer) + 1
	);
	printf("[BLE FOG] %s\n", buffer);
}

// Send Tremor Update
void updateTremorCharacteristic(bool detected, float freq, float power){
	char buffer[MAX_VALUE_LEN];
	if (detected){
        snprintf(buffer, MAX_VALUE_LEN, "TREMOR:1,%.2f,%.3f", freq, power);
	} else{
        snprintf(buffer, MAX_VALUE_LEN, "TREMOR:0,%.2f,%.3f", freq, power);
		
	}
	
	memcpy(tremorValue, buffer, strlen(buffer) + 1);
	ble_interface.gattServer().write(
		tremorCharacteristic.getValueHandle(),
		tremorValue,
		strlen(buffer) + 1
	);
	printf("[BLE TREMOR] %s\n", buffer);
}

// Send Dyskinesia Update
void updateDyskCharacteristic(bool detected, float freq, float power){
	char buffer[MAX_VALUE_LEN];
	if (detected){
        snprintf(buffer, MAX_VALUE_LEN, "DYSK:1,%.2f,%.3f", freq, power);
	} else{
        snprintf(buffer, MAX_VALUE_LEN, "DYSK:0,%.2f,%.3f", freq, power);
		
	}
	
	memcpy(dyskValue, buffer, strlen(buffer) + 1);
	ble_interface.gattServer().write(
		dyskCharacteristic.getValueHandle(),
		dyskValue,
		strlen(buffer) + 1
	);
	printf("[BLE DYSK] %s\n", buffer);
}

// Sensor processing task with FIXED LOGIC
void processSensors(){
	float x, y, z;
	readAccel(x,y,z);
	
	bool fog_ready = fogDetector.update(x,y,z);
	bool tremor_ready = tremorDetector.update(x,y,z);
	
	if (fog_ready && tremor_ready){
        printf("\r\n========== Analysis Update ==========\r\n");
        
        bool is_resting = fogDetector.is_resting || tremorDetector.is_resting;
        
        if (is_resting) {
            printf("[STATUS] Patient at rest\r\n");
            updateFOGCharacteristic(false, 0, 0);
            updateTremorCharacteristic(false, 0, 0);
            updateDyskCharacteristic(false, 0, 0);
            
        } else {
            float freq = tremorDetector.dominant_freq;
            float fog_index = fogDetector.current_index;
            float tremor_power = tremorDetector.tremor_power;
            float dysk_power = tremorDetector.dyskinesia_power;
            
            // FIXED: Non-overlapping frequency ranges
            // FOG: 5-8 Hz (higher frequency)
            // Dyskinesia: 4-6 Hz (mid frequency) 
            // Tremor: 3-4 Hz (lower frequency)
            
            bool fog_detected = false;
            bool tremor_detected = false;
            bool dysk_detected = false;
            
            // Priority 1: FOG detection (highest priority, 5-8 Hz)
            if (fogDetector.is_frozen && freq >= 5.0f && freq <= 8.0f) {
                fog_detected = true;
                printf("[FOG]    >>> FREEZE DETECTED! <<<\r\n");
                printf("         Freeze Index: %.3f\r\n", fog_index);
                printf("         Frequency: %.2f Hz\r\n", freq);
            } else {
                printf("[FOG]    Walking normally (Index: %.3f)\r\n", fog_index);
            }
            
            // Priority 2: Dyskinesia (4-6 Hz) - only if NOT FOG
            if (!fog_detected && tremorDetector.is_dyskinesia && 
                freq >= 4.0f && freq <= 6.0f && dysk_power > tremor_power) {
                dysk_detected = true;
                printf("[DYSK]   >>> DYSKINESIA DETECTED! <<<\r\n");
                printf("         Frequency: %.2f Hz\r\n", freq);
                printf("         Power: %.4f\r\n", dysk_power);
            } else {
                printf("[DYSK]   Not detected (Power: %.4f)\r\n", dysk_power);
            }
            
            // Priority 3: Tremor (3-4 Hz) - only if NOT FOG and NOT Dyskinesia
            if (!fog_detected && !dysk_detected && tremorDetector.is_tremor && 
                freq >= 3.0f && freq < 4.0f) {
                tremor_detected = true;
                printf("[TREMOR] >>> TREMOR DETECTED! <<<\r\n");
                printf("         Frequency: %.2f Hz\r\n", freq);
                printf("         Power: %.4f\r\n", tremor_power);
            } else {
                printf("[TREMOR] Not detected (Freq: %.2f Hz)\r\n", freq);
            }
            
            // Update all three BLE characteristics
            updateFOGCharacteristic(fog_detected, fog_index, freq);
            updateTremorCharacteristic(tremor_detected, freq, tremor_power);
            updateDyskCharacteristic(dysk_detected, freq, dysk_power);
            
            // LED blink if any condition detected
            if (fog_detected || tremor_detected || dysk_detected) {
                led = !led;
            }
        }
        
        printf("=====================================\r\n");
        fflush(stdout);
    }
}

void on_ble_init_complete(BLE::InitializationCompleteCallbackContext *params){
	if (params->error != BLE_ERROR_NONE){
		printf("BLE Initialization failed. \n");
		return;
	}
	
	// init all 3 characteristics with default values
	strcpy((char*)fogValue, "FOG:0,0.00,0.00");
	strcpy((char*)tremorValue, "TREMOR:0,0.00,0.00");
	strcpy((char*)dyskValue, "DYSK:0,0.00,0.00");
	
	ble_interface.gattServer().addService(pdService);
	
	uint8_t adv_buffer[LEGACY_ADVERTISING_MAX_SIZE];
	AdvertisingDataBuilder adv_data(adv_buffer);
	adv_data.setFlags();
	adv_data.setName("PD-Monitor");
	
	ble_interface.gap().setAdvertisingParameters(
		LEGACY_ADVERTISING_HANDLE,
		AdvertisingParameters(advertising_type_t::CONNECTABLE_UNDIRECTED,adv_interval_t(160))
	);
	
	ble_interface.gap().setAdvertisingPayload(LEGACY_ADVERTISING_HANDLE, adv_data.detAdvertisingData());
	ble_interface.gap().startAdvertising(LEGACY_ADVERTISING_HANDLE);
	
	ble_interface.gap().setEventHandler(&gapHandler);

    printf("BLE advertising started as 'PD-Monitor'\n");
    printf("Service UUID: A0E1B2C3-D4E5-F6A7-B8C9-D0E1F2A3B4C5\n");
    printf("FOG Char:     A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C6\n");
    printf("Tremor Char:  A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C7\n");
    printf("Dysk Char:    A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C8\n");
}

// BLE event scheduler
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context){
	event_queue.call(callback(&ble_interface, &BLE::processEvents));
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

	// init BLE
	ble_interface.onEventsToProcess(schedule_ble_events);
	ble_interface.init(on_ble_init_complete);
	printf("BLE Service Initialized\r\n\r\n");
	
	
	timer.start();
	
	while(true){
		auto start_t = timer.elapsed_time();
		
		// Process sensor data
		processSensors();
		
		// Maintain 52Hz timing
		while((timer.elapsed_time()-start_t).count() < PERIOD_US);
		
		// Process BLE events (non-blocking)
		event_queue_dispatch(0);
	}
}