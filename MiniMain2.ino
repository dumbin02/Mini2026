#include "Setup.h"

Acciones estadoActual = ESPERAR_A_BOTON;

void setup()
{
  mmSetup();
  delay(500);
  setupTimer();
  PIDvelocidad_init(p_velPID);
  avanzar();
  setPWM(0,0);
  resetearContadoresDeEncoders();
}

void loop()
{ 
  if (controlLoopFlag) {
    controlLoopFlag = false;
    actualizarOdometriaSensores();
    // 2. Máquina de Estados de Movimiento
    switch (estadoActual) {      
      case DETENERSE:
        setPWM(0, 0);
        break;
      case LINEA_RECTA_PID:
        tiempoInicio = esp_timer_get_time();
        targetVelocity1(); 
        tiempoFinal = esp_timer_get_time();
        telemetriaDT[contadorTest] = tiempoFinal - tiempoInicio;
        break;
      case VUELTA_90_DER:
        noGirarConGyro();
        break;
      case VUELTA_90_IZQ:
        break;
      case VUELTA_180:
      setPWM(75,85);
        break;
      case ESPERAR_A_BOTON:
        setPWM(0, 0);
        if(esperarABoton()){
          delay(500);
          resetFlag = true;
          estadoActual = LINEA_RECTA_PID;
        }
        break;
    }
    }
  
  if (telemetryLoopFlag){
    telemetryEndOfRun();
  }
  
}
