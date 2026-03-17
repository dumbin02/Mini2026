#ifndef SENSORES_HEADER
#define SENSORES_HEADER

//-----------------------------------------------Declaracion Funciones-----------------------------------------------

void leerMPU();
void printMPU();
void leerIRs();
void printIRs();

//-----------------------------------------------Extern variables-----------------------------------------------

//Int(65,536)(-32768,32767), units  A(16384) == (1 g)   G(131) == ()
extern int16_t ax, ay, az, gx, gy, gz;
//ValoresSensoresIR
extern int16_t valorIRDerecho,valorIRIzquierdo,valorIRFrontal;

#endif