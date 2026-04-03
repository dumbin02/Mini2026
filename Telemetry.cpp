#include <sys/_stdint.h>
#include "Setup.h"

int i = 0;

int   listaTelemetria0[1500];
int   listaTelemetria1[1500];
float listaTelemetria2[1500];
float listaTelemetria3[1500];
float listaTelemetria4[1500];
float listaTelemetria5[1500];
float listaTelemetria6[1500];
float listaTelemetria7[1500];
float listaTelemetria8[1500];
float listaTelemetria9[1500];
float listaTelemetria10[1500];

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
  valoresPID = {2,cuenta,2, listaTelemetria0[cuenta],listaTelemetria1[cuenta],
                                      listaTelemetria2[cuenta],listaTelemetria3[cuenta],listaTelemetria4[cuenta],listaTelemetria5[cuenta],
                                      listaTelemetria6[cuenta],listaTelemetria7[cuenta],listaTelemetria8[cuenta],listaTelemetria9[cuenta],
                                      listaTelemetria10[cuenta]};
  sendBinaryTelemetry(&valoresPID, sizeof(valoresPID));
}


void telemetryEndOfRun(){
  telemetryLoopFlag = false;
  
  if(endOfRunFlag){
    
    sendValoresPIDEOR(i);
    delay(40);                 //pequeño delay para no saturar ble
    i++;
    
    if(i >= 1500) {
      i = 0;
      endOfRunFlag = false;
      setPWM(300, 300);
      delay(200);
      setPWM(0, 0);
    }
  }
}



