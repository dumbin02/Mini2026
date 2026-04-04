#ifndef SENSORES_HEADER
#define SENSORES_HEADER
#include "esp_adc/adc_continuous.h"
#define READ_LEN 256    // Tamaño buffer rafaga

//-----------------------------------------------Declaracion Funciones-----------------------------------------------

void actualizar_sensoresIR();
#if SETUP_SERIAL
void printIRs();
#endif

//-----------------------------------------------Extern variables-----------------------------------------------

//ValoresSensoresIR
extern adc_continuous_handle_t handle;


#endif