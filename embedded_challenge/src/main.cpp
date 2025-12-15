// Parkinson's Disease Monitoring with 3-minute Diagnosis
// Tracks symptoms over time to determine PD likelihood

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

// Diagnosis Parameters
#define DIAGNOSIS_WINDOW_MS 180000  // 3 minutes = 180,000 ms
#define MIN_TREMOR_DETECTIONS 20    // Minimum tremor events needed
#define MIN_DYSK_DETECTIONS 15      // Minimum dyskinesia events needed
#define MIN_FOG_DETECTIONS 10       // Minimum FOG events needed
#define CONSISTENCY_THRESHOLD 0.3f   // At least 30% of samples should show symptoms

BufferedSerial pc(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
	return &pc;
}

I2C i2c(SENSORS_SDA, SENSORS_SCL);
Timer timer;
Timer diagnosisTimer;
DigitalOut led(LED1);

// Service instances
FOG_Service fogDetector;
Tremor_Service tremorDetector;

// BLE setup
BLE & ble_interface = BLE::Instance();
EventQueue event_queue;

// Diagnosis tracking structure
struct DiagnosisTracker {
    int tremor_count;
    int dyskinesia_count;
    int fog_count;
    int total_samples;
    bool diagnosis_complete;
    bool has_parkinsons;
    
    void reset() {
        tremor_count = 0;
        dyskinesia_count = 0;
        fog_count = 0;
        total_samples = 0;
        diagnosis_complete = false;
        has_parkinsons = false;
    }
    
    void update(bool tremor, bool dysk, bool fog_detected) {
        if (tremor) tremor_count++;
        if (dysk) dyskinesia_count++;
        if (fog_detected) fog_count++;
        total_samples++;
    }
    
    void evaluate() {
        if (diagnosis_complete) return;
        
        printf("\r\n╔══════════════════════════════════════════╗\r\n");
        printf("║   3-MINUTE DIAGNOSIS ANALYSIS COMPLETE   ║\r\n");
        printf("╚══════════════════════════════════════════╝\r\n");
        printf("Total samples analyzed: %d\r\n", total_samples);
        printf("Tremor detections:      %d\r\n", tremor_count);
        printf("Dyskinesia detections:  %d\r\n", dyskinesia_count);
        printf("FOG detections:         %d\r\n", fog_count);
        printf("------------------------------------------\r\n");
        
        // Calculate consistency ratios
        float tremor_ratio = (float)tremor_count / total_samples;
        float dysk_ratio = (float)dyskinesia_count / total_samples;
        float fog_ratio = (float)fog_count / total_samples;
        
        // Diagnosis criteria:
        // PD is indicated if ANY of the following:
        // 1. Persistent tremor (high count + consistency)
        // 2. Frequent dyskinesia (high count)
        // 3. Multiple FOG episodes (high count)
        // 4. Combination of multiple symptoms
        
        int symptom_score = 0;
        
        if (tremor_count >= MIN_TREMOR_DETECTIONS && tremor_ratio >= CONSISTENCY_THRESHOLD) {
            printf("✓ Persistent tremor detected (%.1f%% consistency)\r\n", tremor_ratio * 100);
            symptom_score += 2;
        }
        
        if (dyskinesia_count >= MIN_DYSK_DETECTIONS && dysk_ratio >= CONSISTENCY_THRESHOLD * 0.7f) {
            printf("✓ Frequent dyskinesia detected (%.1f%% consistency)\r\n", dysk_ratio * 100);
            symptom_score += 2;
        }
        
        if (fog_count >= MIN_FOG_DETECTIONS && fog_ratio >= CONSISTENCY_THRESHOLD * 0.5f) {
            printf("✓ Multiple FOG episodes detected (%.1f%% consistency)\r\n", fog_ratio * 100);
            symptom_score += 2;
        }
        
        // Check for multiple symptoms (even if individually below threshold)
        int symptoms_present = 0;
        if (tremor_count >= MIN_TREMOR_DETECTIONS * 0.5f) symptoms_present++;
        if (dyskinesia_count >= MIN_DYSK_DETECTIONS * 0.5f) symptoms_present++;
        if (fog_count >= MIN_FOG_DETECTIONS * 0.5f) symptoms_present++;
        
        if (symptoms_present >= 2) {
            printf("✓ Multiple symptom types detected\r\n");
            symptom_score++;
        }
        
        printf("------------------------------------------\r\n");
        printf("Symptom score: %d/6\r\n", symptom_score);
        
        // Diagnosis threshold: score >= 2 indicates PD
        has_parkinsons = (symptom_score >= 2);
        diagnosis_complete = true;
        
        if (has_parkinsons) {
            printf("\r\n⚠️  DIAGNOSIS: Parkinson's Disease Indicators Detected\r\n");
            printf("    Recommendation: Consult a neurologist\r\n");
        } else {
            printf("\r\n✓ DIAGNOSIS: No significant PD indicators\r\n");
            printf("  Movement patterns within normal range\r\n");
        }
        printf("╔══════════════════════════════════════════╗\r\n\r\n");
    }
};

DiagnosisTracker diagnosis;

// BLE UUIDs - One service, 4 characteristics (added diagnosis)
const UUID PD_SERVICE_UUID("A0E1B2C3-D4E5-F6A7-B8C9-D0E1F2A3B4C5");
const UUID FOG_CHAR_UUID("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C6");
const UUID TREMOR_CHAR_UUID("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C7");
const UUID DYSK_CHAR_UUID("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C8");
const UUID DIAGNOSIS_CHAR_UUID("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C9");

// Data buffers
#define MAX_VALUE_LEN 20
uint8_t fogValue[MAX_VALUE_LEN];
uint8_t tremorValue[MAX_VALUE_LEN];
uint8_t dyskValue[MAX_VALUE_LEN];
uint8_t diagnosisValue[MAX_VALUE_LEN];

// BLE Characteristics
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

ReadOnlyArrayGattCharacteristic<uint8_t, MAX_VALUE_LEN> diagnosisCharacteristic(
	DIAGNOSIS_CHAR_UUID,
	diagnosisValue,
	GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ
);

// Characteristic table with all four
GattCharacteristic *charTable[] = {&fogCharacteristic, &tremorCharacteristic, &dyskCharacteristic, &diagnosisCharacteristic};
GattService pdService(PD_SERVICE_UUID, charTable, 4);

// BLE Connection Handler
class MyGapEventHandler : public Gap::EventHandler{
	virtual void onConnectionComplete(const ConnectionCompleteEvent &event){
		printf("BLE Connected! Handle: %d\n", event.getConnectionHandle());
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

// Update BLE Characteristics
void updateFOGCharacteristic(bool detected, float index, float freq){
	char buffer[MAX_VALUE_LEN];
	snprintf(buffer, MAX_VALUE_LEN, "FOG:%d,%.2f,%.2f", detected ? 1 : 0, index, freq);
	memcpy(fogValue, buffer, strlen(buffer) + 1);
	ble_interface.gattServer().write(
		fogCharacteristic.getValueHandle(),
		fogValue,
		strlen(buffer) + 1
	);
}

void updateTremorCharacteristic(bool detected, float freq, float power){
	char buffer[MAX_VALUE_LEN];
	snprintf(buffer, MAX_VALUE_LEN, "TREMOR:%d,%.2f,%.3f", detected ? 1 : 0, freq, power);
	memcpy(tremorValue, buffer, strlen(buffer) + 1);
	ble_interface.gattServer().write(
		tremorCharacteristic.getValueHandle(),
		tremorValue,
		strlen(buffer) + 1
	);
}

void updateDyskCharacteristic(bool detected, float freq, float power){
	char buffer[MAX_VALUE_LEN];
	snprintf(buffer, MAX_VALUE_LEN, "DYSK:%d,%.2f,%.3f", detected ? 1 : 0, freq, power);
	memcpy(dyskValue, buffer, strlen(buffer) + 1);
	ble_interface.gattServer().write(
		dyskCharacteristic.getValueHandle(),
		dyskValue,
		strlen(buffer) + 1
	);
}

void updateDiagnosisCharacteristic(bool has_pd, int tremor_cnt, int dysk_cnt, int fog_cnt){
	char buffer[MAX_VALUE_LEN];
	// Format: "PD:status,T:count,D:count,F:count"
	// Status: 0=No PD, 1=PD Detected, 2=Analyzing
	int status = diagnosis.diagnosis_complete ? (has_pd ? 1 : 0) : 2;
	snprintf(buffer, MAX_VALUE_LEN, "PD:%d,%d,%d,%d", status, tremor_cnt, dysk_cnt, fog_cnt);
	memcpy(diagnosisValue, buffer, strlen(buffer) + 1);
	ble_interface.gattServer().write(
		diagnosisCharacteristic.getValueHandle(),
		diagnosisValue,
		strlen(buffer) + 1
	);
	printf("[BLE DIAGNOSIS] %s\n", buffer);
}

// Main sensor processing with diagnosis tracking
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
            
            // Track as non-detection for diagnosis
            if (!diagnosis.diagnosis_complete) {
                diagnosis.update(false, false, false);
            }
            
        } else {
            float freq = tremorDetector.dominant_freq;
            float fog_index = fogDetector.current_index;
            float tremor_power = tremorDetector.tremor_power;
            float dysk_power = tremorDetector.dyskinesia_power;
            
            bool fog_detected = false;
            bool tremor_detected = false;
            bool dysk_detected = false;
            
            // Priority 1: FOG detection (5-8 Hz)
            if (fogDetector.is_frozen && freq >= 5.0f && freq <= 8.0f) {
                fog_detected = true;
                printf("[FOG]    >>> FREEZE DETECTED! <<<\r\n");
                printf("         Freeze Index: %.3f\r\n", fog_index);
                printf("         Frequency: %.2f Hz\r\n", freq);
            } else {
                printf("[FOG]    Walking normally (Index: %.3f)\r\n", fog_index);
            }
            
            // Priority 2: Dyskinesia (4-6 Hz)
            if (!fog_detected && tremorDetector.is_dyskinesia && 
                freq >= 4.0f && freq <= 6.0f && dysk_power > tremor_power) {
                dysk_detected = true;
                printf("[DYSK]   >>> DYSKINESIA DETECTED! <<<\r\n");
                printf("         Frequency: %.2f Hz\r\n", freq);
                printf("         Power: %.4f\r\n", dysk_power);
            } else {
                printf("[DYSK]   Not detected (Power: %.4f)\r\n", dysk_power);
            }
            
            // Priority 3: Tremor (3-4 Hz)
            if (!fog_detected && !dysk_detected && tremorDetector.is_tremor && 
                freq >= 3.0f && freq < 4.0f) {
                tremor_detected = true;
                printf("[TREMOR] >>> TREMOR DETECTED! <<<\r\n");
                printf("         Frequency: %.2f Hz\r\n", freq);
                printf("         Power: %.4f\r\n", tremor_power);
            } else {
                printf("[TREMOR] Not detected (Freq: %.2f Hz)\r\n", freq);
            }
            
            // Update diagnosis tracking
            if (!diagnosis.diagnosis_complete) {
                diagnosis.update(tremor_detected, dysk_detected, fog_detected);
                
                // Show progress
                int elapsed_sec = diagnosisTimer.elapsed_time().count() / 1000000;
                printf("[DIAGNOSIS] Progress: %d/%d sec | T:%d D:%d F:%d\r\n", 
                       elapsed_sec, DIAGNOSIS_WINDOW_MS/1000,
                       diagnosis.tremor_count, diagnosis.dyskinesia_count, diagnosis.fog_count);
            }
            
            // Update all BLE characteristics
            updateFOGCharacteristic(fog_detected, fog_index, freq);
            updateTremorCharacteristic(tremor_detected, freq, tremor_power);
            updateDyskCharacteristic(dysk_detected, freq, dysk_power);
            
            // LED blink if any condition detected
            if (fog_detected || tremor_detected || dysk_detected) {
                led = !led;
            }
        }
        
        // Check if 3 minutes elapsed
        if (!diagnosis.diagnosis_complete && 
            diagnosisTimer.elapsed_time().count() >= DIAGNOSIS_WINDOW_MS * 1000) {
            diagnosis.evaluate();
            updateDiagnosisCharacteristic(diagnosis.has_parkinsons, 
                                         diagnosis.tremor_count,
                                         diagnosis.dyskinesia_count,
                                         diagnosis.fog_count);
        }
        
        printf("=====================================\r\n");
        fflush(stdout);
    }
}

void on_ble_init_complete(BLE::InitializationCompleteCallbackContext *params){
	if (params->error != BLE_ERROR_NONE){
		printf("BLE Initialization failed.\n");
		return;
	}
	
	// Initialize all characteristics
	strcpy((char*)fogValue, "FOG:0,0.00,0.00");
	strcpy((char*)tremorValue, "TREMOR:0,0.00,0.00");
	strcpy((char*)dyskValue, "DYSK:0,0.00,0.00");
	strcpy((char*)diagnosisValue, "PD:2,0,0,0");  // Status 2 = analyzing
	
	ble_interface.gattServer().addService(pdService);
	
	uint8_t adv_buffer[LEGACY_ADVERTISING_MAX_SIZE];
	AdvertisingDataBuilder adv_data(adv_buffer);
	adv_data.setFlags();
	adv_data.setName("PD-Monitor");
	
	ble_interface.gap().setAdvertisingParameters(
		LEGACY_ADVERTISING_HANDLE,
		AdvertisingParameters(advertising_type_t::CONNECTABLE_UNDIRECTED,adv_interval_t(160))
	);
	
	ble_interface.gap().setAdvertisingPayload(LEGACY_ADVERTISING_HANDLE, adv_data.getAdvertisingData());
	ble_interface.gap().startAdvertising(LEGACY_ADVERTISING_HANDLE);
	ble_interface.gap().setEventHandler(&gapHandler);

    printf("BLE advertising started as 'PD-Monitor'\n");
    printf("Service UUID:     A0E1B2C3-D4E5-F6A7-B8C9-D0E1F2A3B4C5\n");
    printf("FOG Char:         A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C6\n");
    printf("Tremor Char:      A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C7\n");
    printf("Dysk Char:        A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C8\n");
    printf("Diagnosis Char:   A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C9\n");
}

void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context){
	event_queue.call(callback(&ble_interface, &BLE::processEvents));
}

int main(){
	pc.set_baud(115200);
	printf("\r\n");
    printf("===========================================\r\n");
    printf("  Parkinson's Disease Monitoring System   \r\n");
    printf("     with 3-Minute Diagnosis Feature      \r\n");
    printf("===========================================\r\n");
    printf("  FOG Detection:    ENABLED\r\n");
    printf("  Tremor Detection: ENABLED\r\n");
    printf("  Dyskinesia:       ENABLED\r\n");
    printf("  Sample Rate:      52 Hz\r\n");
    printf("  Diagnosis Time:   3 minutes\r\n");
    printf("===========================================\r\n\r\n");
	
	initSensor();
	printf("LSM6DSL Sensor Initialized\r\n");
	
	fogDetector.init();
	printf("FOG Service Initialized\r\n");

	tremorDetector.init();
	printf("Tremor Service Initialized\r\n");

	ble_interface.onEventsToProcess(schedule_ble_events);
	ble_interface.init(on_ble_init_complete);
	printf("BLE Service Initialized\r\n\r\n");
	
	// Initialize diagnosis tracker
	diagnosis.reset();
	diagnosisTimer.start();
	printf("Starting 3-minute diagnosis monitoring...\r\n\r\n");
	
	timer.start();
	
	while(true){
		auto start_t = timer.elapsed_time();
		
		processSensors();
		
		// Maintain 52Hz timing
		while((timer.elapsed_time()-start_t).count() < PERIOD_US);
		
		event_queue.dispatch(0);
	}
}