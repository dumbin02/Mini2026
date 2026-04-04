#ifndef TELEMETRY_HEADER
#define TELEMETRY_HEADER
#include <NimBLEDevice.h>
#pragma pack(push, 1) // Asegura que no haya bytes de relleno

struct TelemetriaGeneral {
    uint8_t     tipo = 0; // ID General
    uint16_t    tick;
    uint8_t     intervalo;
    uint8_t     posX, posY;
    float       vel, velAng, distanciaRecorrida;
    float       dt_funcion; //4 floats
};

struct TelemetriaVelocidad {
    uint8_t     tipo = 1; // ID PID
    uint16_t    tick;
    uint8_t     intervalo; // 25, 50, 100, 200...
    uint16_t    errIR;    //
    float       angVelErrorIR, errAngVel, angVel, targetAngVel;
    float       velA, velB, targetVelA, targetVelB;
    float       dt_funcion; // 9 floats
};

struct ValoresPID {
    uint8_t     tipo = 2; // ID General
    uint16_t    tick;
    uint8_t     intervalo;
    //0-1
    int         pwmA, pwmB; 
    //2-5
    float       velocidadError, velocidadPterm, velocidadDterm, velocidadIterm;
    //6-9
    float       angVelError, angVelPterm, angVelDterm, angVelIterm;
    float       dt_funcion; //9 floats
};
#pragma pack(pop)
//-----------------------------------------------Declaracion Funciones-----------------------------------------------
void setupBLE();
void sendBinaryTelemetry(void* data, size_t size);

void sendDatosGenerales(float tiempo, float posX, float posY, float angulo, float velA, float velB);
void sendDatosVelocidadEOR(uint16_t errIR, float angVelErrorIR, float errAngVel, float angVel, float targetAngVel, float velA, float velB,float targetVelA, float targetVelB,float dt);
void sendValoresPIDEOR(int cuenta);
void telemetryEndOfRun();
//-----------------------------------------------Extern variables-----------------------------------------------
extern NimBLEServer* server;
extern NimBLECharacteristic* telemetryChar;
extern TelemetriaGeneral   datosGeneral;
extern TelemetriaVelocidad datosVelocidad;
extern ValoresPID          valoresPID;

extern int   telemetriaPWMA[1500];
extern int   telemetriaPWMB[1500];
extern float telemetriaVelErr[1500];
extern float telemetriaVelPTerm[1500];
extern float telemetriaVelDTerm[1500];
extern float telemetriaVelITerm[1500];
extern float telemetriaAngErr[1500];
extern float telemetriaAngPTerm[1500];
extern float telemetriaAngDTerm[1500];
extern float telemetriaAngITerm[1500];
extern float telemetriaDT[1500];



#endif