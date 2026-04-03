#ifndef SENSORES_HEADER
#define SENSORES_HEADER
#include "esp_adc/adc_continuous.h"
#define GYRO_Z_OUT_H  0x47
#define READ_LEN 256    // Tamaño buffer rafaga

//-----------------------------------------------Declaracion Funciones-----------------------------------------------

void leerMPU();
void leerIRs();
void setup_adc_dma();
void actualizar_sensoresIR();
#if SETUP_SERIAL
void printMPU();
void printIRs();
#endif

//-----------------------------------------------Extern variables-----------------------------------------------

//Int(65,536)(-32768,32767), units  A(16384) == (1 g)   G(131) == ()
extern int16_t gz;
extern float   gyroZ;
//ValoresSensoresIR
extern float valorIRDerecho,valorIRIzquierdo,valorIRFrontal;
extern adc_continuous_handle_t handle;


#endif