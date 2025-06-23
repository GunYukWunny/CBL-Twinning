#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

const char* targetName = "GuideBeacon";
const int scanTime = 3; // seconds
int txPower = -59;
float alpha = 0.2;
float lastRSSI = -59;

BLEScan* pBLEScan;

float estimateDistance(float rssi) {
  return pow(10, (txPower - rssi) / (10.0 * 2.0)); // path loss exponent n = 2
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == targetName) {
      float rssi = advertisedDevice.getRSSI();
      rssi = alpha * rssi + (1 - alpha) * lastRSSI;
      lastRSSI = rssi;

      float dist = estimateDistance(rssi);
      Serial.printf("%.2f\n", dist);
    }
  }
};

void setup() {
  Serial.begin(115200);
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), false);
  pBLEScan->setActiveScan(true);
}

void loop() {
  pBLEScan->start(scanTime, false);
  pBLEScan->clearResults();
  delay(500);
}
