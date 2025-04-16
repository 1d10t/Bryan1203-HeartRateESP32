#include <ArduinoBLE.h>
#include <algorithm>
#include <iostream>

#define DATA_PIN 12

/* Device name which can be seen in BLE scanning software. */
#define BLE_DEVICE_NAME "ESP32 BLE Sense"
/* Local name which should pop up when scanning for BLE devices. */
#define BLE_LOCAL_NAME "ESP32 HR monitor"

#define MIN_HR 45
#define MAX_HR 210

#define FAN_PWM_PIN 0
#define PLUS_PIN 1
#define MINUS_PIN 2
#define AUTO_PIN 3
#define MANUAL_PIN 4

/* Define global variables */
volatile uint8_t duty_cycle;
volatile int offset = 170;
// Determine if using HR to control the fan; default to true (AUTO mode)

volatile bool automatic = true;
volatile bool neutral = false;

BLEService HeartRateService("180D");
BLECharacteristic HeartRateMeasurement("2A37", BLENotify, 6);
BLEDevice peripheral;

// Variables to keep track of the timing of recent interrupts
unsigned long button_time = 0;
unsigned long last_button_time = 0;

void IRAM_ATTR plusISR() {
  button_time = millis();
  if (button_time - last_button_time > 250) {
    if (offset + 10 > 255) {
      offset = 255;
    } else {
      offset += 10;
    }
    Serial.println("plus offset");
    last_button_time = button_time;
  }
}

void IRAM_ATTR minusISR() {
  button_time = millis();
  if (button_time - last_button_time > 250) {
    if (offset - 10 < 0) {
      offset = 0;
    } else {
      offset -= 10;
    }
    Serial.println("minus offset");
    last_button_time = button_time;
  }
}

void IRAM_ATTR autoISR() {
  button_time = millis();
  if (button_time - last_button_time > 250) {
    Serial.println("Entered Auto Mode");
    automatic = true;
    neutral = false;
    last_button_time = button_time;
  }
}

void IRAM_ATTR manualISR() {
  button_time = millis();
  if (button_time - last_button_time > 250) {
    Serial.println("Entered Manual Mode");
    automatic = false;
    neutral = false;
    last_button_time = button_time;
  }
}

void IRAM_ATTR neutralISR() {
  button_time = millis();
  if (button_time - last_button_time > 250) {
    Serial.println("Entered Neutral Mode");
    automatic = false;
    neutral = true;
    last_button_time = button_time;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  Serial.println("BLE Central for Garmin HRM");

  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setLocalName(BLE_LOCAL_NAME);
  BLE.setAdvertisedService(HeartRateService);
  HeartRateService.addCharacteristic(HeartRateMeasurement);
  BLE.addService(HeartRateService);
  BLE.advertise();
  Serial.println("Center established");
  BLE.scanForUuid("180D");

  pinMode(FAN_PWM_PIN, OUTPUT);
  pinMode(PLUS_PIN, INPUT_PULLUP);
  pinMode(MINUS_PIN, INPUT_PULLUP);
  pinMode(AUTO_PIN, INPUT_PULLUP);
  pinMode(MANUAL_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PLUS_PIN), plusISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MINUS_PIN), minusISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(AUTO_PIN), autoISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(MANUAL_PIN), manualISR, FALLING);
}

void loop() {
  peripheral = BLE.available();
  BLEDevice central = BLE.central();

  if (automatic) {
    Serial.println("Auto Heart Rate Mode: ");
    analogWrite(FAN_PWM_PIN, 0);
    if (peripheral) {
      Serial.print("Found ");
      Serial.print(peripheral.address());
      Serial.print(" '");
      Serial.print(peripheral.localName());
      Serial.print("' ");
      Serial.print(peripheral.advertisedServiceUuid());
      Serial.println();
      if (peripheral.localName() == "HUAWEI WATCH 4-271") {  // name of BLE HRM device
        BLE.stopScan();
        if (peripheral.connect()) Serial.print("Connected to Garmin HRM ");
        Serial.println("Discovering attributes ...");
        if (peripheral.discoverAttributes()) {  // find services and characteristics
          Serial.println("Attributes discovered");
          BLEService service = peripheral.service("180d");
          BLECharacteristic characteristic = service.characteristic("2a37");
          characteristic.subscribe();
          Serial.println("subscribed to 2a37");
        } else {
          Serial.println("Attribute discovery failed!");
          peripheral.disconnect();
          setup();
          return;
        }
      } else {
        Serial.println("Failed to connect!");
        return;
      }

      while (peripheral.connected() && automatic) {
        BLEService service = peripheral.service("180d");
        BLECharacteristic characteristic = service.characteristic("2a37");

        if (characteristic.valueUpdated()) {
          uint8_t value[6];
          characteristic.readValue(value, 6);
          Serial.println(value[1]);
          // Change the fan speed to the corresponding HR value
          // Map the HR to 0 - 255 that corresponds to the duty cycle (PWM) fan speed
          Serial.print("Fan PWM 0-255: ");
          duty_cycle = min((int)(((double)(value[1] - MIN_HR) / (MAX_HR - MIN_HR)) * 255) + offset, 255);
          Serial.println(duty_cycle);
          analogWrite(FAN_PWM_PIN, duty_cycle);
        }
        delay(1000);
      }
      Serial.println("disconnected to HRM");
      peripheral.disconnect();
      setup();
    }
  } else if (!automatic && !neutral) {
    Serial.println("Manual Mode:");
    delay(1000);
    Serial.print("Fan PWM 0-255: ");
    duty_cycle = offset;
    Serial.println(duty_cycle);
    analogWrite(FAN_PWM_PIN, duty_cycle);
  } else if (neutral) {
    Serial.println("Neutral Mode:");
    analogWrite(FAN_PWM_PIN, 0);
    delay(1000);
  }
}
