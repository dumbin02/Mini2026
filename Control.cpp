#include "Arduino.h"
#include "esp_timer.h"
#include <sys/_stdint.h>
#include "Setup.h"

//Var temp
  int contadorTargetVel = 0;
  bool flagVuelta = false;
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
  volatile  float     dt,dtContadorA,dtContadorB;

//--------------------------------Variables PID Velocidad (Se usa dt [1kHz control loop = 0.001 s como dt]) (const)--------------------------------
  const float  velKP  = 10;
  const float  velKA  = 142;
  const float  velKFF = 3.2;        //3.6 le cambiaste pto
  const float  velKD  = 0.0027;       //0.27
  const float  velKI  = 300;
  float  alphaKd = 1;
  float I_A;
  float I_B ;              
  //--------------------------------------P Term-----------------------------------
  //--------------------------------------I Term-----------------------------------
  float velSumaGananciaIntegralA = 0;      // Acumulador rueda A
  float velSumaGananciaIntegralB = 0;      // Acumulador rueda B
  const float velGananciaMaxIntegral = 100/velKI; // Anti-Windup: Límite de la suma
  //--------------------------------------D Term-----------------------------------
  float  kAcc_VelPreviaA = 0;
  float  kAcc_VelPreviaB = 0;
  float  velErrorPrevioA = 0;
  float  velErrorPrevioB = 0;
  float  kdPrevioA = 0;
  float  kdPrevioB = 0;
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
  tiempoActual = esp_timer_get_time();
  dt = (tiempoActual-tiempoPrevio)*1e-6;
  tiempoPrevio = tiempoActual;
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
  calcular_dt();
  if(resetFlag){
    resetFlag = false;
    resetearVelocidadYPWM();
    resetearVariablesPID();
    resetearContadoresDeEncoders();
    dt = 0.001;
  }
  actualizar_sensoresIR();
  leerMPU();
  getDistanciaRecorrida();
  getVelocidadAng(); 
}

void resetearVariablesPID(){
  velSumaGananciaIntegralA = 0;
  velSumaGananciaIntegralB = 0;
  I_A = 0;
  I_B = 0;
}

void resetearVelocidadYPWM(){
  pwmOutA = 0;
  pwmOutB = 0;
  velocidadA = 0;
  velocidadB = 0;
}

void calculoPIDVelocidad(float targetVelA, float targetVelB) {
  // 1. Cálculo de errores
  float errorA = targetVelA - velocidadA;
  float errorB = targetVelB - velocidadB;

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
  float D_A = velKD * (errorA - velErrorPrevioA) / dt;
  float D_B = velKD * (errorB - velErrorPrevioB) / dt;

  D_A = (D_A * alphaKd) + (1.0 - alphaKd) * (kdPrevioA);
  D_B = (D_B * alphaKd) + (1.0 - alphaKd) * (kdPrevioB);

  velErrorPrevioA = errorA; // Actualizar para el próximo ciclo
  velErrorPrevioB = errorB;

  // 5. Salida de Control
  pwmOutA = P_A + I_A + D_A;
  pwmOutB = P_B + I_B + D_B;

  // Aplicamos constrain de PWM final
  pwmOutA = constrain(pwmOutA, -1023, 1023);
  pwmOutB = constrain(pwmOutB, -1023, 1023);
  
  listaTelemetria10[contadorTargetVel] = dt;
  
  setPWM(pwmOutA, pwmOutB);
}

void calculoPIDVelocidadAngular(){
  //Tomamos como error el negativo de la Velocidad Angular del Giroscopio
  float omegaError = velocidadAngular;
    
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
  float velllA = 270 + velOutA;
  float velllB = 270 + velOutB; 
  calculoPIDVelocidad(velllA, velllB);
  if(contadorTargetVel >= 1499) {
    setPWM(0, 0);
    contadorTargetVel = 0;
    endOfRunFlag = true;
    estadoActual = DETENERSE;
  }
  contadorTargetVel++;
}


void targetVelocity1() {
  float targetVel = 0;

  if (contadorTargetVel < 500) {
    targetVel = 300.0 - (float)(contadorTargetVel) * (200.0 / 500.0);
  } 
  else if (contadorTargetVel < 1000) {
    targetVel = 100.0;
  } 
  else if (contadorTargetVel < 1500) {
    // Calculamos cuánto hemos avanzado en esta fase (0 a 500)
    targetVel = 100 + (float)(contadorTargetVel - 1000) * (200.0 / 500.0);
  } 
  calculoPIDVelocidad(targetVel, targetVel);
   // Error diferencial para análisis
  
  
  

 // 4. Parada Total
  if(contadorTargetVel >= 1499) {
    setPWM(0, 0);
    targetVel = 0;
    contadorTargetVel = 0;
    endOfRunFlag = true;
    delay(200);
    estadoActual = DETENERSE;
  }
  contadorTargetVel++;
}

void targetVelocity2() {
  
  float targetVel = 0;
    targetVel = (float)(contadorTargetVel) * (300.0 / 500.0);

  if (contadorTargetVel < 500) {
  } 
  else if (contadorTargetVel < 1000) {
    targetVel = 300.0;
  } 
  else if (contadorTargetVel < 1500) {
    // Calculamos cuánto hemos avanzado en esta fase (0 a 500)
    targetVel = 300.0 - (float)(contadorTargetVel - 1000) * (300.0 / 500.0);
  } 
  calculoPIDVelocidad(targetVel, targetVel);
   // Error diferencial para análisis
  
  
  

  // 4. Parada Total
  if(contadorTargetVel >= 1499) {
    setPWM(0, 0);
    targetVel = 0;
    contadorTargetVel = 0;
    endOfRunFlag = true;
    delay(100);
    estadoActual = DETENERSE;
  }
  contadorTargetVel++;
}

void targetVelocityConst(float vel_A, float vel_B){ 
  calculoPIDVelocidad(vel_A, vel_B);

  
  
  

  // 4. Parada Total
  if(contadorTargetVel >= 1499) {
    setPWM(0, 0);
    contadorTargetVel = 0;
    endOfRunFlag = true;
    delay(200);
    estadoActual = DETENERSE;
  }
  contadorTargetVel++;
}

void vuelta90Grados(){
  if(flagVuelta){
    resetearContadoresDeEncoders();
    flagVuelta = false;
  }
  if(distanciaRecorridaMM < 70.68){
    giro_diferencial_derecha();
    calculoPIDVelocidad(100, 100);
  }else{
  setPWM(0,0);
  delay(500);
  flagVuelta = true;
  }

}














