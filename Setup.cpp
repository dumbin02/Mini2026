#include "Arduino.h"
#include "FunctionalInterrupt.h"
#include "esp32-hal-ledc.h"
#include "Setup.h"


void iniciarPines(){
  pinMode(MOTOR_A1,         OUTPUT);
  pinMode(MOTOR_A2,         OUTPUT);
  pinMode(MOTOR_B1,         OUTPUT);
  pinMode(MOTOR_B2,         OUTPUT);

  ledcAttach(PWM_A, PWM_FREQ, PWM_RESOLUCION);
  ledcAttach(PWM_B, PWM_FREQ, PWM_RESOLUCION);

  pinMode(SENSORDERECHO,    INPUT_PULLDOWN);
  pinMode(SENSORFRONTAL,    INPUT_PULLDOWN);
  pinMode(SENSORIZQUIERDO,  INPUT_PULLDOWN);
#if SETUP_ENCODER
  attachInterrupt(ENCODER_A, contadorEncoderA, RISING);
  attachInterrupt(ENCODER_B, contadorEncoderB, RISING);
#endif
}



void initMPU() {
  Wire.begin(SDA_PIN, SCL_PIN);   // custom I2C pins

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);               // power management register
  Wire.write(0);                  // wake up MPU6050
  Wire.endTransmission(true);
}

void mmSetup(){
  iniciarPines();
#if SETUP_MPU
  initMPU();
#endif
  avanzar();
#if SETUP_BLE
  setupBLE();
  sendTelemetry(0.0,0.0,0.0);
  delay(600);
#endif

}