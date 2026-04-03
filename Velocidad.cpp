#include <sys/_stdint.h>
#include "Arduino.h"
#include "esp_timer.h"
#include <sys/_intsup.h>
#include "Setup.h"

const float constCont_REV = (PI*DIAMETRO_LLANTA)/(CPR*REDUCCION_ENGRANAJE);
const float constDistancia_Ruedas = 1.00/(DISTANCIA_ENTRE_RUEDAS);
const float alpha = 0.1;

volatile int      contadorA = 0, contadorB = 0;
volatile float   velocidadA_interrupt, velocidadB_interrupt;
float             velocidadA,velocidadB;
float             distanciaRecorridaMM = 0;
float             velocidadAngular=0;

volatile uint64_t tiempoFinalA,tiempoFinalB;

void contadorEncoderA(){
  volatile uint64_t tiempoInicioA = esp_timer_get_time();
  contadorA++;
  calcular_dtContadorA();
  velocidadA_interrupt = constCont_REV/dtContadorA;
  tiempoFinalA = esp_timer_get_time() - tiempoInicioA;
}
uint64_t getTiempoEncoderAINT(){
  return tiempoFinalA;
}

void contadorEncoderB(){
  volatile uint64_t tiempoInicioB = esp_timer_get_time();
  contadorB++;
  calcular_dtContadorB();
  velocidadB_interrupt = constCont_REV/dtContadorB;
  tiempoFinalB = esp_timer_get_time() - tiempoInicioB;
}

uint64_t getTiempoEncoderBINT(){
  return tiempoFinalB;
}

void getDistanciaRecorrida(){
  // Distancia total acumulada desde el inicio 
  float distanciaRecorridaA = (float)contadorA * constCont_REV;
  float distanciaRecorridaB = (float)contadorB * constCont_REV;

  // Promedio de ambas ruedas para el centro del robot
  distanciaRecorridaMM = (distanciaRecorridaA + distanciaRecorridaB) * 0.5f;
}

void getVelocidades(){
  //Extraemos velocidades de los interrupts
  float velocidadA_raw = velocidadA_interrupt;
  float velocidadB_raw = velocidadB_interrupt;
  //Dependiendo de la direccion del motor cambiamos el signo de nuestra velocidad
  if(!direccionMotorB) velocidadB_raw = -velocidadB_raw;
  if(!direccionMotorA) velocidadA_raw = -velocidadA_raw;
  //Filtrado de velocidad, sirve en ambas direcciones
  velocidadA =(alpha * velocidadA_raw) + ((1.0 - alpha) * velocidadA);
  velocidadB =(alpha * velocidadB_raw) + ((1.0 - alpha) * velocidadB);
}

void resetearContadoresDeEncoders(){
  contadorA = 0;
  contadorB = 0;
  
  // Promedio de ambas ruedas para el centro del robot
  distanciaRecorridaMM = 0;
}

//Obtenemos Velocidad Angular con la diff de velocidades lineales entre las 2 ruedas, divididas por la distancia entre las ruedas
//Unidades en rad/s, 
void getVelocidadAng(){
  velocidadAngular = (-velocidadA + velocidadB) * constDistancia_Ruedas;
}




