#include "Setup.h"

float valorIRDerecho,valorIRIzquierdo,valorIRFrontal;

uint8_t result[READ_LEN];
uint32_t ret_num = 0;

const float alphaIR = 0.9;

#if SETUP_SERIAL
void printIRs(){
  Serial.print(valorIRIzquierdo); Serial.print(" ");
  Serial.print(valorIRFrontal); Serial.print(" ");
  Serial.println(valorIRDerecho);
}
#endif

void actualizar_sensoresIR() {
    uint8_t result[READ_LEN];
    uint32_t ret_num = 0;
    
    // 1. Declarar variables temporales para los RAW
    static int16_t rawDer = 0, rawFront = 0, rawIzq = 0; 

    esp_err_t ret = adc_continuous_read(handle, result, READ_LEN, &ret_num, 0);

    if (ret == ESP_OK) {
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
            // Extraer canal y datos (Lógica para C3/S3)
            uint32_t chan = (uint32_t)((result[i + 1] >> 4) & 0xF); 
            uint32_t data = (uint32_t)((result[i] | (result[i + 1] & 0xF) << 8));

            // ASIGNACIÓN CORRECTA (Sin redeclarar la variable)
            if      (chan == ADC_CHANNEL_0) rawDer = data;
            else if (chan == ADC_CHANNEL_2) rawFront = data;
            else if (chan == ADC_CHANNEL_4) rawIzq = data;
        }
    }

    // 2. Filtrado Alpha (Ahora sí usan los valores actualizados del loop)
    MM.sensorDerecho   = (alphaIR * rawDer)   + ((1.0 - alphaIR) * valorIRDerecho);
    MM.sensorIzquierdo = (alphaIR * rawIzq)   + ((1.0 - alphaIR) * valorIRIzquierdo);
    MM.sensorFrontal   = (alphaIR * rawFront) + ((1.0 - alphaIR) * valorIRFrontal);
}

