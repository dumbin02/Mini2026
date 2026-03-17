#include <sys/_intsup.h>
#include "Setup.h"

const float constCont_REV = PI/(CPR*REDUCCION_ENGRANAJE);

float distanciaRecorrida = 0;

int contadorA = 0, contadorB = 0;
float revolucionesA,revolucionesB,velocidadA,velocidadB;

void contadorEncoderA(){
  contadorA++;
}
void contadorEncoderB(){
  contadorB++;
}

void getDistanciaRecorrida(){
  distanciaRecorridaA = contadorA*constCont_REV;
  distanciaRecorridaB = contadorB*constCont_REV;
  distanciaRecorrida = ((distanciaRecorridaA+distanciaRecorridaB)<<2)*constCont_REV;
}

void getVelocidad(){
  ve
}

void getVelocidadAng(){

}


