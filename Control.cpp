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
  bool resetFlag = false;


//--------------------------------Variables Timing--------------------------------
  #define DELTA_T_MIN   300
          int64_t     tiempoActual = 0;
          int64_t     tiempoPrevio = 0;
          int64_t     tiempoActualTelemetria = 0;
          int64_t     tiempoPrevioTelemetria = 0;
  volatile  int64_t   tiempoActualContadorA = 0;
          int64_t     tiempoPrevioContadorA = 0;
  volatile  int64_t   tiempoActualContadorB = 0;
          int64_t   tiempoPrevioContadorB = 0;
  volatile  float     dtContadorA,dtContadorB;
            float     dt = 0.001;

//--------------------------------Variables PID Velocidad (Se usa dt [1kHz control loop = 0.001 s como dt]) (const)--------------------------------
  const float  velKP  = 10;
  const float  velKA  = 142;
  const float  velKFF = 3.2;        //3.6 le cambiaste pto
  const float  velKD  = 1.4;       //0.27
  const float  velKI  = 300;
  float  alphaKd = 0.002;
  float I_A;
  float I_B ;              
  //--------------------------------------P Term-----------------------------------
  //--------------------------------------I Term-----------------------------------
  float velSumaGananciaIntegralA = 0;      // Acumulador rueda A
  float velSumaGananciaIntegralB = 0;      // Acumulador rueda B
  const float velGananciaMaxIntegral = 30/velKI; // Anti-Windup: Límite de la suma
  //--------------------------------------D Term-----------------------------------
  float  kAcc_VelPreviaA = 0;
  float  kAcc_VelPreviaB = 0;
  float  velErrorPrevioA = 0;
  float  velErrorPrevioB = 0;
  float  previoD_A = 0;
  float  previoD_B = 0;
  //PWMs de salida
  int pwmOutA;
  int pwmOutB;

//--------------------------------Variables PID Vel Angular (Se usa dt [1kHz control loop = 0.001 s como dt]) (const)--------------------------------
  const float  omegaKFF = DISTANCIA_ENTRE_RUEDAS;
  const float  omegaKP  = 25;
  const float  omegaKD  = 0.02;     
  //--------------------------------------P Term-----------------------------------
  //--------------------------------------I Term-----------------------------------
  float omegaSumaGananciaIntegralA = 0;      // Acumulador 
  const float omegaGananciaMaxIntegral = 50/velKI; // Anti-Windup: Límite de la suma
  //--------------------------------------D Term-----------------------------------
  float  omegaErrorPrevio = 0;
  //Velocidades de salida
  float velOutA;
  float velOutB;

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
  getVelocidades();

  //Forzar velocidades a 0 si llevamos cierto tiempo sin recibir un nuevo pulso
  if(MM.contadorMotorA == pulsoPrevioA) contadorPulsoPrevioA++;
  else contadorPulsoPrevioA = 0;
  if(MM.contadorMotorB == pulsoPrevioB) contadorPulsoPrevioB++;
  else contadorPulsoPrevioB = 0;
  if(contadorPulsoPrevioA >= CONTADOR_PULSO_VEL_MAX) MM.velocidadMotorA = 0;
  if(contadorPulsoPrevioA >= CONTADOR_PULSO_VEL_MAX) MM.velocidadMotorA = 0;
  
  //calcular_dt();
  if(resetFlag){
    resetFlag = false;
    resetearVariablesPID();
    resetearContadoresDeEncoders();
    //dt = 0.001;
  }
  actualizar_sensoresIR();
  getDistanciaRecorrida();
  getVelocidadAng(); 
  
}

void resetearVariablesPID(){
  velSumaGananciaIntegralA = 0;
  velSumaGananciaIntegralB = 0;
  I_A = 0;
  I_B = 0;
}

void calculoPIDVelocidad(float targetVelA, float targetVelB) {
  // 1. Cálculo de errores
  float errorA = targetVelA - MM.velocidadMotorA;
  float errorB = targetVelB - MM.velocidadMotorB;

  telemetriaAngDTerm[contadorTest] = errorA;
  telemetriaAngITerm[contadorTest] = errorB;

  float AccA = targetVelA - kAcc_VelPreviaA;
  float AccB = targetVelB - kAcc_VelPreviaB;

  kAcc_VelPreviaA = targetVelA;
  kAcc_VelPreviaB = targetVelB;

  // 2. Ganancia Proporcional (P)
  float kP_A = (velKP * errorA);
  float kP_B = (velKP * errorB);

  float kP_AccA = velKA * AccA;
  float kP_AccB = velKA * AccB;

  float P_A = kP_A + (targetVelA * velKFF) + 17 + kP_AccA;//15 de offset
  float P_B = kP_B + (targetVelB * velKFF) + 31 + kP_AccB;

  P_A = constrain(P_A, -1023, 1023);
  P_B = constrain(P_B, -1023, 1023);  

  // 3. Ganancia Integral (I) con Anti-Windup
  velSumaGananciaIntegralA += (errorA) * dt;
  velSumaGananciaIntegralB += (errorB) * dt;
  
  velSumaGananciaIntegralA = constrain(velSumaGananciaIntegralA, -velGananciaMaxIntegral, velGananciaMaxIntegral);
  velSumaGananciaIntegralB = constrain(velSumaGananciaIntegralB, -velGananciaMaxIntegral, velGananciaMaxIntegral);

  // Limitar la suma para evitar saturación (Anti-Windup)
  I_A = velKI * velSumaGananciaIntegralA;
  I_B = velKI * velSumaGananciaIntegralB;

  // 4. Ganancia Derivativa (D)
  float errorD_A = (errorA - velErrorPrevioA);
  float errorD_B = (errorB - velErrorPrevioB);

  float D_A = -velKD * errorD_A / dt;
  float D_B = -velKD * errorD_B / dt;
  
  D_A = (D_A * alphaKd) + (1.0 - alphaKd) * (previoD_A);
  D_B = (D_B * alphaKd) + (1.0 - alphaKd) * (previoD_B);
  D_A = constrain(D_A,-500,500);
  D_B = constrain(D_B,-500,500);
  previoD_A = D_A;
  previoD_B = D_B;
  if((fabsf(errorD_A)) < 2)D_A = 0;
  if((fabsf(errorD_B)) < 2)D_B = 0;
  velErrorPrevioA = errorA; // Actualizar para el próximo ciclo
  velErrorPrevioB = errorB;

  // 5. Salida de Control
  pwmOutA = P_A + I_A + D_A;
  pwmOutB = P_B + I_B + D_B;

  // Aplicamos constrain de PWM final
  pwmOutA = constrain(pwmOutA, -1023, 1023);
  pwmOutB = constrain(pwmOutB, -1023, 1023);
  telemetriaVelDTerm[contadorTest] = D_A;
  telemetriaVelITerm[contadorTest] = errorD_A;
  telemetriaVelPTerm[contadorTest] = errorD_B;
  telemetriaPWMA[contadorTest] = pwmOutA;
  telemetriaPWMB[contadorTest] = pwmOutB;
  telemetriaAngErr[contadorTest]   = MM.velocidadMotorA;
  telemetriaAngPTerm[contadorTest] = MM.velocidadMotorB;

  setPWM(pwmOutA, pwmOutB);
}

void calculoPIDVelocidadAngular(){
  //Tomamos como error el negativo de la Velocidad Angular del Giroscopio
  float omegaError = 0;
    
  //Ganancia Proporciona;
  float omegaP  = omegaKP * omegaError;

  //Ganancia Derivativa
  float omegaD = omegaKD * (omegaError - omegaErrorPrevio) / dt;
  omegaErrorPrevio = omegaError;

  float omegaVelOut = omegaP + omegaD;

  if(omegaError > 0.003){
    velOutA = (-omegaVelOut);
    velOutB = 0;
  }else if(omegaError < -0.003){
    velOutA = 0;
    velOutB = (omegaVelOut);
  }
}

void noGirarConGyro(){
  calculoPIDVelocidadAngular();
  float velllA = 270 ; //+ velOutA
  float velllB = 270 ; //+ velOutB 
  calculoPIDVelocidad(velllA, velllB);
  if(contadorTest >= 1499) {
    setPWM(0, 0);
    contadorTest = 0;
    endOfRunFlag = true;
    telemetryLoopFlag = true;
    estadoActual = DETENERSE;
  }
  contadorTest++;
}

void targetVelocity1() {
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
  calculoPIDVelocidad(targetVel, targetVel);
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
  }
  contadorTest++;
}

void targetVelocity2() {
  
  float targetVel = 0;
    targetVel = (float)(contadorTest) * (300.0 / 500.0);

  if (contadorTest < 500) {
  } 
  else if (contadorTest < 1000) {
    targetVel = 300.0;
  } 
  else if (contadorTest < 1500) {
    // Calculamos cuánto hemos avanzado en esta fase (0 a 500)
    targetVel = 300.0 - (float)(contadorTest - 1000) * (300.0 / 500.0);
  } 
  calculoPIDVelocidad(targetVel, targetVel);
  
  // 4. Parada Total
  if(contadorTest >= 1499) {
    setPWM(0, 0);
    targetVel = 0;
    contadorTest = 0;
    endOfRunFlag = true;
    telemetryLoopFlag = true;
    delay(100);
    estadoActual = DETENERSE;
  }
  telemetriaVelErr[contadorTest] = targetVel;
  contadorTest++;
}

void targetVelocityConst(float vel_A, float vel_B){ 
  calculoPIDVelocidad(vel_A, vel_B);

  // 4. Parada Total
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

void vuelta90Grados(){
  //
}














