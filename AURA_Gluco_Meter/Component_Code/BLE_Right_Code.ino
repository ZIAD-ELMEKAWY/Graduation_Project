/* ESP32C3 BLE Monitor 
 * @auther : Ziad-Elmekawy
 * @file   : Hardware/BLE
 * @date   : 26, April, 2025
 * @proj   : AURA Health Monitor
 * @desc   : Send JSON data through BLE 
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h> // REQUIRED LIBRARY FOR JSON SERIALIZATION

// --- BLE Globals (Unchanged) ---
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// See the following for generating UUIDs:
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// --- Health Data Variables (New) ---
// Note: In a real application, these values would be updated by sensor readings.
int heartRate = 85.0;
int spO2 = 98;
int temp = 36.8;
int steps = 1024;
float gps_lat = 34.0522; // Placeholder for Latitude
float gps_lon = -118.2437; // Placeholder for Longitude
int posture = 1;         // 1: Standing, 0: Sitting
int sos = 1;             // 1: SOS Alert Active
int shake = 1;           // 1: Shake/Fall Detected

// Define the size of the JSON document buffer (256 bytes is enough for this data)
const size_t JSON_DOC_SIZE = 256;

// --- Callback Classes (Unchanged) ---
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
};

    void onDisconnect(BLEServer *pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected");
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        Serial.println(rxValue);
        Serial.println("*********");
      }
    }
};

// --- Setup Function (Unchanged) ---
void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("Health Monitor"); // Changed name for context
  // Set MTU to 200 bytes to accommodate your JSON payload. 
  // The maximum is typically 247, but 200 is a safe, high value.
  // Write to MTU in nmobile app equal to 200 as write here (Important)
  BLEDevice::setMTU(200);
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic (TX - NOTIFY)
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  // Create a BLE Characteristic (RX - WRITE)
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service & advertising
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

// --- Loop Function (Modified) ---
void loop() {
  if (deviceConnected) {
    // 1. Create a dynamic JSON document
    DynamicJsonDocument doc(JSON_DOC_SIZE);
    
    // 2. Populate the document with all data points
    // Using abbreviations to save bytes over BLE
    doc["HR"] = heartRate;
    doc["O2"] = spO2;
    doc["T"] = temp;
    doc["ST"] = steps;

    // Create a nested GPS object
    JsonObject gps = doc.createNestedObject("GPS");
    gps["lat"] = gps_lat;
    gps["lon"] = gps_lon;

    // Add status/event flags
    doc["POS"] = posture; 
    doc["SOS"] = sos;
    doc["SHK"] = shake;

    // 3. Serialize JSON to a String
    String jsonString;
    serializeJson(doc, jsonString);
    
    // 4. Update placeholder data (for demonstration purposes)
    // To show the value changing over time
    steps++; 
    heartRate = 85.0 + (sin(millis() / 5000.0) * 5.0); // Simple HR fluctuation

    Serial.print("Notifying JSON: ");
    Serial.println(jsonString);
    
    // 5. Send the JSON string via BLE (Notify)
    pTxCharacteristic->setValue((uint8_t*)jsonString.c_str(), jsonString.length());
    pTxCharacteristic->notify();
    
    // 6. Wait for 5 seconds (5000 milliseconds)
    delay(5000);  
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   
    pServer->startAdvertising();  
    Serial.println("Started advertising again...");
    oldDeviceConnected = false;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = true;
  }
}
