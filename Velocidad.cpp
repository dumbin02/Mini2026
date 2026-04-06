#include <sys/_stdint.h>
#include "Arduino.h"
#include "esp_timer.h"
#include <sys/_intsup.h>
#include "Setup.h"

const float constCont_REV = (PI*DIAMETRO_LLANTA)/(CPR*REDUCCION_ENGRANAJE);
const float constDistancia_Ruedas = 1.00/(DISTANCIA_ENTRE_RUEDAS);
const float alpha = 0.15;

volatile float   velocidadA_interrupt, velocidadB_interrupt;
float             distanciaRecorridaMM = 0;
float             velocidadAngular=0;

volatile uint64_t tiempoFinalA,tiempoFinalB;

void contadorEncoderA(){
  p_MM->contadorMotorA++;
  calcular_dtContadorA();
  velocidadA_interrupt = constCont_REV/dtContadorA;
}


void contadorEncoderB(){
  p_MM->contadorMotorB++;
  calcular_dtContadorB();
  velocidadB_interrupt = constCont_REV/dtContadorB;
}


void getDistanciaRecorrida(){
  // Promedio de ambas ruedas para el centro del robot
  p_MM->distanciaRecorrida = (p_MM->contadorMotorA + p_MM->contadorMotorB) * 0.5f * constCont_REV;
}

void getVelocidades(){
  //Extraemos velocidades de los interrupts
  float velocidadA_raw = velocidadA_interrupt;
  float velocidadB_raw = velocidadB_interrupt;
  //Dependiendo de la direccion del motor cambiamos el signo de nuestra velocidad
  if(!direccionMotorB) velocidadB_raw = -velocidadB_raw;
  if(!direccionMotorA) velocidadA_raw = -velocidadA_raw;
  //Filtrado de velocidad[1], sirve en ambas direcciones
  p_MM->velocidadMotorA =(alpha * velocidadA_raw) + ((1.0 - alpha) * p_MM->velocidadMotorA);
  p_MM->velocidadMotorB =(alpha * velocidadB_raw) + ((1.0 - alpha) * p_MM->velocidadMotorB);

}

void resetearContadoresDeEncoders(){
  p_MM->contadorMotorA = 0;
  p_MM->contadorMotorB = 0;
  // Promedio de ambas ruedas para el centro del robot
  p_MM->distanciaRuedaA = 0;
  p_MM->distanciaRuedaB = 0;
  p_MM->distanciaRecorrida = 0;
}

//Obtenemos Velocidad Angular con la diff de velocidades lineales entre las 2 ruedas, divididas por la distancia entre las ruedas
//Unidades en rad/s, 
void getVelocidadAng(){
  p_MM->velocidadAngular = (-p_MM->velocidadMotorA + p_MM->velocidadMotorB) * constDistancia_Ruedas;
}




