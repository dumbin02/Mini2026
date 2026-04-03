#ifndef SETUP_HEADER
#define SETUP_HEADER
//---------------------------------------CONFIGURACION DE COMPILACION---------------------------------------
#define   SETUP_SERIAL    0
#define   SETUP_BOTON     1
#define   SETUP_BLE       1
#define   SETUP_ENCODER   1
#define   SETUP_MPU       1
//-----------------------------------------------LIBRERIAS-----------------------------------------------
//Acceso a pines
#include "driver/gpio.h"
//I2C comm
#include <Wire.h>
//-----------------------------------------------PINES-----------------------------------------------
//Pines Motor  [Motor Derecho A - Motor Izquierdo B]
#define   MOTOR_A1          5
#define   MOTOR_A2          4
#define   MOTOR_B1          6
#define   MOTOR_B2          7
#define   PWM_A             3
#define   PWM_B             10
//Pines Encoders
#define   ENCODER_A         21
#define   ENCODER_B         20
//Pines Sensores IR
#define   SENSORDERECHO     0
#define   SENSORFRONTAL     1
#define   SENSORIZQUIERDO   2
//MPU6050 Pines,Address
#define   SDA_PIN           8
#define   SCL_PIN           9
#define   MPU_ADDR          0x68
//PIN BOTON GEI
#define   BOTONGEI          4
//PWM Resolucion y frecuencia
#define   PWM_FREQ          12000
#define   PWM_RESOLUCION    10
// Hz
#define FRECUENCIA_CONTROL_LOOP 1000

//-----------------------------------------------Declaracion Funciones-----------------------------------------------
void initMPU();
void setupADC();
void setup_adc_dma();
void iniciarPines();
void mmSetup();
#if SETUP_BOTON
bool esperarABoton();
#endif
void timerCallback(void* arg);
void telemetryCallback(void* arg);
void setupTimer();
//-----------------------------------------------Extern variables-----------------------------------------------
extern const float constFrecuencia_Control; 
extern volatile bool controlLoopFlag;
extern volatile bool telemetryLoopFlag;
extern esp_timer_handle_t  timer_control;
#include "ControlMotores.h"
#include "Sensores.h"
#include "Telemetry.h"
#include "Velocidad.h"
#include "Control.h"

#endif