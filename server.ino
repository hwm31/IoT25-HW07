/*
 * ESP32 BLE Client for Distance Measurement
 * This device scans for BLE signals, calculates distance based on RSSI,
 * and sends the distance information back to the server
 */

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

// Define target device
#define TARGET_DEVICE_NAME "11_server"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Define measurement parameters
#define SCAN_TIME  3         // Scan time in seconds
#define N_SAMPLES  5         // Number of RSSI measurements to average
#define PATH_LOSS_EXPONENT 2.7  // Path loss exponent (adjust for your environment)
                              // Free space: 2.0, Indoor: 2.7-4.3

// LED pin for proximity alert
#define LED_PIN 2            // Built-in LED on ESP32 is typically pin 2

// Variables for RSSI measurements
int rssiValues[N_SAMPLES];
int sampleIndex = 0;
int txPowerAt1m = -59;       // Transmission power at 1m (needs calibration)

// Scanning and client objects
BLEScan* pBLEScan;
BLEClient* pClient = NULL;
BLERemoteCharacteristic* pRemoteCharacteristic = NULL;

// Server info storage variables
BLEAddress* serverAddress = NULL;
bool doConnect = false;
bool connected = false;

// Variable to store last distance measurement
float lastDistance = -1;

// BLE connection and communication function
bool connectToServer() {
  Serial.print("Connecting to: ");
  Serial.println(serverAddress->toString().c_str());
  
  // Connect to server
  pClient = BLEDevice::createClient();
  pClient->connect(*serverAddress);

  // Find service
  BLERemoteService* pRemoteService = pClient->getService(BLEUUID(SERVICE_UUID));
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find service: ");
    Serial.println(SERVICE_UUID);
    pClient->disconnect();
    return false;
  }

  // Find characteristic
  pRemoteCharacteristic = pRemoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find characteristic: ");
    Serial.println(CHARACTERISTIC_UUID);
    pClient->disconnect();
    return false;
  }

  // Read txPower value
  if(pRemoteCharacteristic->canRead()) {
    String value = pRemoteCharacteristic->readValue();
    if(value.length() > 0) {
      txPowerAt1m = atoi(value.c_str());
      Serial.print("Server txPower: ");
      Serial.println(txPowerAt1m);
    }
  }

  connected = true;
  return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() != TARGET_DEVICE_NAME)
      return;

    int rssi = advertisedDevice.getRSSI();
    rssiValues[sampleIndex % N_SAMPLES] = rssi;
    sampleIndex++;

    // N 샘플 모이면…  
    if (sampleIndex >= N_SAMPLES) {
      // (1) 거리 계산  
      int sum = 0;
      for (int i = 0; i < N_SAMPLES; i++) sum += rssiValues[i];
      float avgRssi = (float)sum / N_SAMPLES;
      float distance = calculateDistance(avgRssi, txPowerAt1m);
      Serial.printf("Estimated distance: %.2f m\n", distance);

      // (2) 서버에 연결 → 쓰기 → 끊기  
      if (connectToServer()) {
        char buf[20];
        sprintf(buf, "DIST:%.2f", distance);
        pRemoteCharacteristic->writeValue(buf);
        Serial.println("Sent & disconnecting");
        pClient->disconnect();
      }

      // (3) 다음 사이클을 위해 리셋  
      sampleIndex = 0;
      for (int i = 0; i < N_SAMPLES; i++) rssiValues[i] = 0;
      doConnect = false;
    }
  }
};

// Function to calculate distance from RSSI
float calculateDistance(float rssi, float txPower) {
  if (rssi == 0) {
    return -1.0; // Error case
  }
  
  // Distance calculation using the path loss model
  // distance = 10 ^ ((txPower - RSSI) / (10 * n))
  // where n is the path loss exponent
  float ratio = (txPower - rssi) / (10 * PATH_LOSS_EXPONENT);
  return pow(10, ratio);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Client...");
  
  // Setup LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize BLE
  BLEDevice::init("");
  
  // Create scanner
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); // Active scan uses more power but gets results faster
  pBLEScan->setInterval(100);    // Set scan interval
  pBLEScan->setWindow(99);       // Less than interval for power saving
  
  // Initialize RSSI values array
  for (int i = 0; i < N_SAMPLES; i++) {
    rssiValues[i] = 0;
  }
  
  Serial.println("BLE Client ready. Scanning for target device...");
  Serial.printf("Target device: %s\n", TARGET_DEVICE_NAME);
  Serial.printf("txPower at 1m (default): %d dBm\n", txPowerAt1m);
  Serial.printf("Path loss exponent: %.1f\n", PATH_LOSS_EXPONENT);
}

void loop() {
  // 연결 플래그 사용 안 함 → 스캔만 반복
  Serial.println("Scanning for RSSI samples…");
  pBLEScan->start(SCAN_TIME, false);
  pBLEScan->clearResults();
  delay(1000);
}