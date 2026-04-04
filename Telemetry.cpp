#include <sys/_stdint.h>
#include "Setup.h"

int i = 0;

int   telemetriaPWMA[1500];
int   telemetriaPWMB[1500];
float telemetriaVelErr[1500];
float telemetriaVelPTerm[1500];
float telemetriaVelDTerm[1500];
float telemetriaVelITerm[1500];
float telemetriaAngErr[1500];
float telemetriaAngPTerm[1500];
float telemetriaAngDTerm[1500];
float telemetriaAngITerm[1500];
float telemetriaDT[1500];

NimBLEServer* server;
NimBLECharacteristic* telemetryChar;

// Instancias
TelemetriaGeneral   datosGeneral;
TelemetriaVelocidad datosVelocidad;
ValoresPID          valoresPID;

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
}
void sendBinaryTelemetry(void* data, size_t size) {
    // NimBLE::setValue y notify son muy eficientes con punteros directos
    telemetryChar->setValue((uint8_t*)data, size);
    telemetryChar->notify(); 
}

void sendDatosGenerales(float tiempo, float posX, float posY, float angulo, float velA, float velB){
  sendBinaryTelemetry(&datosGeneral, sizeof(datosGeneral));
}

void sendDatosVelocidadEOR(uint16_t errIR, float angVelErrorIR, float errAngVel, float angVel, float targetAngVel, float velA, float velB,float targetVelA, float targetVelB,float dt){
  datosVelocidad = {1,0,2,errIR,angVelErrorIR,errAngVel,angVel,targetAngVel,velA,velB,targetVelA,targetVelB,dt};
  sendBinaryTelemetry(&datosVelocidad, sizeof(datosVelocidad));
}

void sendValoresPIDEOR(int cuenta){
  valoresPID = {2,cuenta,2, telemetriaPWMA[cuenta],telemetriaPWMB[cuenta],
                                      telemetriaVelErr[cuenta],telemetriaVelPTerm[cuenta],telemetriaVelDTerm[cuenta],telemetriaVelITerm[cuenta],
                                      telemetriaAngErr[cuenta],telemetriaAngPTerm[cuenta],telemetriaAngDTerm[cuenta],telemetriaAngITerm[cuenta],
                                      telemetriaDT[cuenta]};
  sendBinaryTelemetry(&valoresPID, sizeof(valoresPID));
}


void telemetryEndOfRun(){
  if(endOfRunFlag){
    
    sendValoresPIDEOR(i);
    delay(20);                 //pequeño delay para no saturar ble
    i++;
    
    if(i >= 1500) {
      i = 0;
      endOfRunFlag = false;
      setPWM(300, 300);
      delay(200);
      setPWM(0, 0);
      telemetryLoopFlag = false;
    }
  }
}



