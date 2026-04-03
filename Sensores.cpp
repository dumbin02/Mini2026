#include "Setup.h"

float valorIRDerecho,valorIRIzquierdo,valorIRFrontal;
int16_t gz;
float   gyroZ;

uint8_t result[READ_LEN];
uint32_t ret_num = 0;

const float const_degS_radS = PI/(180*65.5); 
const float alpha = 0.9;

void leerMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_Z_OUT_H); // Apunta directamente al registro 0x47
  if (Wire.endTransmission(false) != 0) return; // Error de bus

  // Solicitamos únicamente los 2 bytes del eje Z
  if (Wire.requestFrom(MPU_ADDR, 2, true) == 2) {
    // Combinamos High y Low byte en un entero de 16 bits con signo
    gz = (int16_t)(Wire.read() << 8 | Wire.read());

    // Convertimos a la unidad de control (rad/s) para el PID
    gyroZ = gz * const_degS_radS;
  }
}
#if SETUP_SERIAL
void printMPU(){
  Serial.print(ax/16384.00); Serial.print(" ");
  Serial.print(ay/16384.00); Serial.print(" ");
  Serial.print(az/16384.00); Serial.print(" ");

  Serial.print(gx); Serial.print(" ");
  Serial.print(gy); Serial.print(" ");
  Serial.println(gz);
}

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
            if (chan == ADC_CHANNEL_0)      rawDer = data;
            else if (chan == ADC_CHANNEL_2) rawFront = data;
            else if (chan == ADC_CHANNEL_4) rawIzq = data;
        }
    }

    // 2. Filtrado Alpha (Ahora sí usan los valores actualizados del loop)
    valorIRDerecho   = (alpha * rawDer)   + ((1.0 - alpha) * valorIRDerecho);
    valorIRIzquierdo = (alpha * rawIzq)   + ((1.0 - alpha) * valorIRIzquierdo);
    valorIRFrontal   = (alpha * rawFront) + ((1.0 - alpha) * valorIRFrontal);

    // Telemetría
    listaTelemetria2[contadorTargetVel] = valorIRDerecho;
    listaTelemetria3[contadorTargetVel] = valorIRFrontal;
    listaTelemetria4[contadorTargetVel] = valorIRIzquierdo;
}

