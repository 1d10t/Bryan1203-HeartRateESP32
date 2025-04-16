#include <ArduinoBLE.h>
#include <algorithm>

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

struct BLEDeviceWithRSSI {
  BLEDevice device;
  int rssi;
};

bool compareDevicesByRSSI(const BLEDeviceWithRSSI& a, const BLEDeviceWithRSSI& b) {
  return a.rssi > b.rssi;
}

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
  BLEDevice central = BLE.central();

  if (automatic) {
    Serial.println("Auto Heart Rate Mode: ");
    analogWrite(FAN_PWM_PIN, 0);
    BLE.scanForUuid("180D");

    BLEDeviceWithRSSI devices[10]; // Buffer to store up to 10 devices
    int deviceCount = 0;

    unsigned long scanStartTime = millis();
    const unsigned long scanDuration = 5000; // Scan duration in milliseconds

    while (millis() - scanStartTime < scanDuration) {
      BLEDevice dev = BLE.available();
      if (dev) {
        int rssi = dev.rssi();
        Serial.print("Found ");
        Serial.print(dev.address());
        Serial.print(" '");
        Serial.print(dev.localName());
        Serial.print("' ");
        Serial.print(dev.advertisedServiceUuid());
        Serial.print(" RSSI: ");
        Serial.println(rssi);

        if (deviceCount < 10) {
          devices[deviceCount].device = dev;
          devices[deviceCount].rssi = rssi;
          deviceCount++;
        }
      }
      delay(100);
    }

    // Sort devices by RSSI in descending order
    std::sort(devices, devices + deviceCount, compareDevicesByRSSI);

    for (int i = 0; i < deviceCount; i++) {
      BLEDevice dev = devices[i].device;
      Serial.print("Connecting to ");
      Serial.print(dev.address());
      Serial.print(" '");
      Serial.print(dev.localName());
      Serial.print("' ");
      Serial.print(dev.advertisedServiceUuid());
      Serial.print(" RSSI: ");
      Serial.println(dev.rssi());

      BLE.stopScan();
      if (dev.connect()) {
        Serial.println("Connected to HRM device");
        if (dev.discoverAttributes()) {
          Serial.println("Attributes discovered");
          BLEService service = dev.service("180d");
          BLECharacteristic characteristic = service.characteristic("2a37");
          if (characteristic.canSubscribe()) {
            characteristic.subscribe();
            Serial.println("Subscribed to 2a37");
            peripheral = dev;
            break;
          } else {
            Serial.println("Cannot subscribe to 2a37");
            dev.disconnect();
          }
        } else {
          Serial.println("Attribute discovery failed!");
          dev.disconnect();
        }
      } else {
        Serial.println("Failed to connect!");
      }
    }

    if (!peripheral) {
      Serial.println("No suitable device found");
    }

    while (peripheral.connected() && automatic) {
      BLEService service = peripheral.service("180d");
      BLECharacteristic characteristic = service.characteristic("2a37");

      if (characteristic.valueUpdated()) {
        uint8_t value[6];
        characteristic.readValue(value, 6);
        uint8_t heartRate = value[1];
        Serial.println(heartRate);
        // Change the fan speed to the corresponding HR value
        // Map the HR to 0 - 255 that corresponds to the duty cycle (PWM) fan speed
        Serial.print("Fan PWM 0-255: ");
        duty_cycle = min((int)(((double)(heartRate - MIN_HR) / (MAX_HR - MIN_HR)) * 255) + offset, 255);
        Serial.println(duty_cycle);
        analogWrite(FAN_PWM_PIN, duty_cycle);

        // Prepare the HRM data to send to the connected central device
        uint8_t hrmData[6] = {0x06, heartRate}; // Flags + Heart Rate Value
        HeartRateMeasurement.writeValue(hrmData, 6);
      }
      delay(1000);
    }

    Serial.println("Disconnected from HRM device");
    peripheral.disconnect();
    peripheral = BLEDevice();
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
