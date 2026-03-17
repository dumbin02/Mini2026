#ifndef TELEMETRY_HEADER
#define TELEMETRY_HEADER
#include <NimBLEDevice.h>

//-----------------------------------------------Declaracion Funciones-----------------------------------------------
void setupBLE();
void sendTelemetry(float x, float y, float theta);


//-----------------------------------------------Extern variables-----------------------------------------------
extern NimBLEServer* server;
extern NimBLECharacteristic* telemetryChar;

#endif