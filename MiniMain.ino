#include "Setup.h"
#define   SETUP_BLE       1
#define   SETUP_ENCODER   1
#define   SETUP_MPU       1
#define   SETUP_BOTON     0

void setup()
{
  setupBLE();
  delay(500);
  iniciarPines();
  avanzar();
  sendTelemetry(contadorA,contadorB,0.0);
  setPWM(100, 100);
}

void loop()
{
  getRevoluciones_Llanta();
  sendTelemetry(revolucionesA,contadorA,0.0);
  delay(100);
}
