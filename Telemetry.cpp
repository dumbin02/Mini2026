//BLE
#include "Setup.h"

NimBLEServer* server;
NimBLECharacteristic* telemetryChar;

void setupBLE() {

  NimBLEDevice::init("MMouse");

  server = NimBLEDevice::createServer();

  NimBLEService* service = server->createService("1234");

  telemetryChar = service->createCharacteristic(
        "6678",
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  telemetryChar->createDescriptor("2902");
  telemetryChar->setValue("0,0,0");

  service->start();

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID("1234");
  advertising->setName("MMouse");
  advertising->start();

  while(server->getConnectedCount() == 0){
    delay(10);
  }
  
}

void sendTelemetry(float x, float y, float theta) {

  static char packet[32];

  snprintf(packet, sizeof(packet), "%.3f,%.3f,%.3f", x, y, theta);

  telemetryChar->setValue(packet);
  telemetryChar->notify();
}




