#include "Arduino.h"
#include "esp_timer.h"
#include <sys/_stdint.h>
#include "Setup.h"

//Var temp
  int contadorTest = 0;
  int contadorPulsoPrevioA = 0;
  int contadorPulsoPrevioB = 0;
  int pulsoPrevioA = 0;
  int pulsoPrevioB = 0;
//Flags
  bool vueltaFlag = false;
  bool endOfRunFlag = false;
  bool resetFlag = true;


//--------------------------------Variables Timing--------------------------------
  #define DELTA_T_MIN   300
          int64_t     tiempoActual = 0;
          int64_t     tiempoPrevio = 0;
          int64_t     tiempoInicio;
          int64_t     tiempoFinal;
          int64_t     tiempoActualTelemetria = 0;
          int64_t     tiempoPrevioTelemetria = 0;
  volatile  int64_t   tiempoActualContadorA = 0;
          int64_t     tiempoPrevioContadorA = 0;
  volatile  int64_t   tiempoActualContadorB = 0;
          int64_t     tiempoPrevioContadorB = 0;
  volatile  float     dtContadorA,dtContadorB;
            float     dt = 0.001;


void calcular_dt(){
  //tiempoActual = esp_timer_get_time();
  dt = 0.001; //(tiempoActual-tiempoPrevio)*1e-6;
  //tiempoPrevio = tiempoActual;
}

void calcular_dtContadorA(){
  tiempoActualContadorA = esp_timer_get_time();
  uint64_t delta = tiempoActualContadorA - tiempoPrevioContadorA;

  // Solo procesar y actualizar si el tiempo es válido (> 270 us)
  if(delta > DELTA_T_MIN) {
    dtContadorA = delta * 1e-6;
  }
  tiempoPrevioContadorA = tiempoActualContadorA;
}

void calcular_dtContadorB(){
  tiempoActualContadorB = esp_timer_get_time();
  uint64_t delta = tiempoActualContadorB - tiempoPrevioContadorB;

  if(delta > DELTA_T_MIN) {
    dtContadorB = delta * 1e-6;
  }
    tiempoPrevioContadorB = tiempoActualContadorB;
}

void actualizarOdometriaSensores(){
  if(resetFlag){
    resetFlag = false;
    p_MM->velocidadMotorA = 0;
    p_MM->velocidadMotorB = 0;
    tiempoPrevioContadorA = esp_timer_get_time();
    tiempoPrevioContadorB = esp_timer_get_time();
    PIDvelocidad_init(p_velPID);
  }

  getVelocidades();

  //Forzar velocidades a 0 si llevamos cierto tiempo sin recibir un nuevo pulso
  if(MM.contadorMotorA == pulsoPrevioA) contadorPulsoPrevioA++;
  else contadorPulsoPrevioA = 0;
  if(MM.contadorMotorB == pulsoPrevioB) contadorPulsoPrevioB++;
  else contadorPulsoPrevioB = 0;
  if(contadorPulsoPrevioA >= CONTADOR_PULSO_VEL_MAX) MM.velocidadMotorA = 0;
  if(contadorPulsoPrevioA >= CONTADOR_PULSO_VEL_MAX) MM.velocidadMotorA = 0;
  
  actualizar_sensoresIR();
  getDistanciaRecorrida();
  getVelocidadAng(); 
}



/* --------------------------------------Tests----------------------------------------- */
void noGirarConGyro(){
  float velllA = 270 ; //+ velOutA
  float velllB = 270 ; //+ velOutB 
  PIDvelocidad_update(velllA,velllB,p_velPID,p_MM);
  if(contadorTest >= 1499) {
    setPWM(0, 0);
    contadorTest = 0;
    endOfRunFlag = true;
    telemetryLoopFlag = true;
    estadoActual = DETENERSE;
  }
  contadorTest++;
}

void targetVelocity2() {
  float targetVel = 0;

  if (contadorTest < 500) {
    targetVel = 300.0 - (float)(contadorTest) * (200.0 / 500.0);
  } 
  else if (contadorTest < 1000) {
    targetVel = 100.0;
  } 
  else if (contadorTest < 1500) {
    // Calculamos cuánto hemos avanzado en esta fase (0 a 500)
    targetVel = 100 + (float)(contadorTest - 1000) * (200.0 / 500.0);
  } 
  PIDvelocidad_update(targetVel,targetVel,p_velPID,p_MM);
  telemetriaVelErr[contadorTest] = targetVel;
 // 4. Parada Total
  if(contadorTest >= 1499) {
    setPWM(0, 0);
    targetVel = 0;
    contadorTest = 0;
    endOfRunFlag = true;
    telemetryLoopFlag = true;
    delay(200);
    estadoActual = DETENERSE;
    contadorTest++;
  }
  
}

void targetVelocity1() {

  float targetVel = 0;

  
  if (contadorTest < 100) {
    targetVel = (float)(contadorTest) * (300.0f / 100.0f);
  } 
  else if (contadorTest < 1400) {
    targetVel = 300.0;
  } 
  else if (contadorTest < 1500) {
    // Calculamos cuánto hemos avanzado en esta fase (0 a 500)
    targetVel = 300.0 - ((float)(contadorTest - 1400) * (300.0 / 100.0));
  } 

  PIDvelocidad_update(targetVel,targetVel,p_velPID,p_MM);
  
  // 4. Parada Total
  if(contadorTest >= 1499) {
    setPWM(0, 0);
    targetVel = 0;
    endOfRunFlag = true;
    telemetryLoopFlag = true;
    delay(100);
    estadoActual = DETENERSE;
  }
  if(contadorTest<=1499){
  telemetriaVelErr[contadorTest] = targetVel;
  telemetriaPWMA[contadorTest] = p_velPID->pwmA;
  telemetriaPWMB[contadorTest] = p_velPID->pwmB;
  telemetriaVelITerm[contadorTest] = p_velPID->integratorA;
  telemetriaAngErr[contadorTest] = p_MM->velocidadMotorA;
  telemetriaAngPTerm[contadorTest] = p_MM->velocidadMotorB;
  contadorTest++;}
}

void targetVelocityConst(float vel_A, float vel_B){ 
  
  PIDvelocidad_update(vel_A,vel_B,p_velPID,p_MM);


  if(contadorTest >= 1499) {
    setPWM(0, 0);
    contadorTest = 0;
    endOfRunFlag = true;
    telemetryLoopFlag = true;
    delay(200);
    estadoActual = DETENERSE;
  }
  contadorTest++;
}








