#ifndef SETUP_HEADER
#define SETUP_HEADER

//-----------------------------------------------LIBRERIAS-----------------------------------------------
//Acceso a pines
#include "driver/gpio.h"
//I2C comm
#include <Wire.h>
//-----------------------------------------------PINES-----------------------------------------------
//Pines Motor  [Motor Izquierdo A - Motor Derecho B]
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
#define SDA_PIN 8
#define SCL_PIN 9
#define MPU_ADDR 0x68
//PWM Resolucion y frecuencia
#define   PWM_FREQ          12000
#define   PWM_RESOLUCION    10

//-----------------------------------------------Declaracion Funciones-----------------------------------------------
void initMPU();
void iniciarPines();
void mmSetup();
//-----------------------------------------------Extern variables-----------------------------------------------


#include "ControlMotores.h"
#include "Sensores.h"
#include "Telemetry.h"
#include "Velocidad.h"

#endif