#include <sys/_stdint.h>
#include "Arduino.h"
#include "esp_timer.h"
#include <sys/_intsup.h>
#include "Setup.h"

const float constCont_REV = (PI*DIAMETRO_LLANTA)/(CPR*REDUCCION_ENGRANAJE);
const float constDistancia_Ruedas = 1.00/(DISTANCIA_ENTRE_RUEDAS);
const float alpha = 0.23;

volatile float   velocidadA_interrupt, velocidadB_interrupt;
float             distanciaRecorridaMM = 0;
float             velocidadAngular=0;

volatile uint64_t tiempoFinalA,tiempoFinalB;

void contadorEncoderA(){
  MM.contadorMotorA++;
  calcular_dtContadorA();
  velocidadA_interrupt = constCont_REV/dtContadorA;
}


void contadorEncoderB(){
  MM.contadorMotorB++;
  calcular_dtContadorB();
  velocidadB_interrupt = constCont_REV/dtContadorB;
}


void getDistanciaRecorrida(){
  // Promedio de ambas ruedas para el centro del robot
  MM.distanciaRecorrida = (MM.contadorMotorA + MM.contadorMotorB) * 0.5f * constCont_REV;
}

void getVelocidades(){
  //Extraemos velocidades de los interrupts
  float velocidadA_raw = velocidadA_interrupt;
  float velocidadB_raw = velocidadB_interrupt;
  //Dependiendo de la direccion del motor cambiamos el signo de nuestra velocidad
  if(!direccionMotorB) velocidadB_raw = -velocidadB_raw;
  if(!direccionMotorA) velocidadA_raw = -velocidadA_raw;
  //Filtrado de velocidad[1], sirve en ambas direcciones
  MM.velocidadMotorA =(alpha * velocidadA_raw) + ((1.0 - alpha) * MM.velocidadMotorA);
  MM.velocidadMotorB =(alpha * velocidadB_raw) + ((1.0 - alpha) * MM.velocidadMotorB);

}

void resetearContadoresDeEncoders(){
  MM.contadorMotorA = 0;
  MM.contadorMotorB = 0;
  // Promedio de ambas ruedas para el centro del robot
  MM.distanciaRuedaA = 0;
  MM.distanciaRuedaB = 0;
  MM.distanciaRecorrida = 0;
}

//Obtenemos Velocidad Angular con la diff de velocidades lineales entre las 2 ruedas, divididas por la distancia entre las ruedas
//Unidades en rad/s, 
void getVelocidadAng(){
  MM.velocidadAngular = (-MM.velocidadMotorA + MM.velocidadMotorB) * constDistancia_Ruedas;
}




