#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h> 

#include <string.h>

#define eeprom 0x50

BLEServer *pServer = NULL;
BLECharacteristic * pNotifyCharacteristic;
BLECharacteristic * pReadCharacteristic;

bool deviceConnected = false;
bool oldDeviceConnected = false;

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

#define SERVICE_UUID                "fdc59e94-27d1-4576-94c6-404b459c11ff" // UART service UUID
#define CHARACTERISTIC_UUID_READ    "fdc59e94-27d2-4576-94c6-404b459c11ff"
#define CHARACTERISTIC_UUID_NOTIFY  "fdc59e94-27d3-4576-94c6-404b459c11ff"

bool dataDump = false;
uint16_t dataSize = 0;

int pIndex = 0;
int address = 0;

class MainServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class ReadCallback: public BLECharacteristicCallbacks {
	void onRead(BLECharacteristic *pCharacteristic) {
    Serial.println("Got read ");
	}
};

class NotifyCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      std::string dumpStr ("d");
      std::string sleepStr ("s");
      
      if (rxValue.length() > 0) {
        Serial.print("Received Value: ");
        Serial.println(rxValue.c_str());

        if (rxValue.compare(dumpStr) == 0){
          Serial.println("Dump data command");
          dataDump = true;
        }
        else if (rxValue.compare(sleepStr) == 0){
             Serial.println("Night night");
             esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
             esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
             esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
             esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
             esp_deep_sleep_start();
        }
      }
    }
};

void writeEEPROMPage(int deviceaddress, unsigned int eeaddress, int *data ) {
  Wire.beginTransmission(deviceaddress);
  Wire.write((uint8_t)(eeaddress >> 8));    // MSB 
  Wire.write((uint8_t)(eeaddress & 0xFF)); // LSB 

  for (int i=0;i<64;i++){
    Wire.write(data[i]);
  }
  Wire.endTransmission();
  delay(10);
}

void writeEEPROM(int deviceaddress, unsigned int eeaddress, int data ) 
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((uint8_t)(eeaddress >> 8));    // MSB 
  Wire.write((uint8_t)(eeaddress & 0xFF)); // LSB 
  Wire.write(data);
  Wire.endTransmission();
  delay(10);
}
 
uint8_t readEEPROM(int deviceaddress, unsigned int  eeaddress ) 
{
  uint8_t rdata = 0x00;
 
  Wire.beginTransmission(deviceaddress);
  Wire.write((uint8_t)(eeaddress >> 8));    // MSB 
  Wire.write((uint8_t)(eeaddress & 0xFF)); // LSB 
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress, 1);
 
  while (Wire.available()){
    rdata = Wire.read();
  }
 
  return rdata;
}

void writeDummyData(){
  Serial.println("Starting write");

  int charStart = 48;
  int data[64];
  for (int x=0;x<64;x++){
    data[x] = charStart++;
    if (charStart >= 122) charStart = 48;
  }

  for (int x=0;x<512;x++){
    writeEEPROMPage(eeprom, address+(x*64), data);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Starting up");

  Wire.begin(); //creates a Wire object
  //writeDummyData();

  // Create the BLE Device
  BLEDevice::init("BLEDU");

  dataSize = 32768 ; // Full size of external eeprom

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MainServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pReadCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_READ, BLECharacteristic::PROPERTY_READ);
  pReadCharacteristic->setCallbacks(new ReadCallback());
  pReadCharacteristic->setValue(dataSize); // This should changed when new data is saved.

  pNotifyCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_NOTIFY, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pNotifyCharacteristic->setCallbacks(new NotifyCallback());

  // Start the service
  pService->start();

  // Start advertising the service too.
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID); 
  pServer->getAdvertising()->start();

  Serial.println("Waiting a client connection to notify...");
}

/* Reads bytes from disk and into buffer */
void bytesFromDisk(uint8_t *buffer, int bufferSize){
  for (int x=0;x<bufferSize;x++){
    byte b = readEEPROM(eeprom, address + pIndex);
    buffer[x] = b;
    pIndex++;
  }
}

void loop() {
  if (dataDump == true){
    // Default 20 byte buffer
    int buffer_size = 20;
    if (address + pIndex + buffer_size >= dataSize){
      // Don't have a full buffer left.
      int diff = (address + pIndex + buffer_size) - dataSize;
      buffer_size -= diff;
    }

    uint8_t buffer[buffer_size];

    // Measure how long the read takes so we can alter the delay we use
    unsigned long StartTime = millis();

    bytesFromDisk(buffer, buffer_size);

    unsigned long CurrentTime = millis();
    unsigned long ElapsedTime = CurrentTime - StartTime;

    if (pIndex >= dataSize){
      Serial.println("All data sent.");
      pIndex = 0;
      dataDump = false;
    }
      
    if (deviceConnected) {
        pNotifyCharacteristic->setValue(buffer, buffer_size);
        pNotifyCharacteristic->notify();
    }

    // BLE stack needs a break
    int tdiff = 10 - ElapsedTime;
    if (tdiff > 0) delay(tdiff);
  }
    
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
    }

    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
}
