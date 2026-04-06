/*
  04-2026
  Este controlador de volocidad esta pensado para un micro-mouse
  con 2 motores n20 con encoder. Se le da un target velocity como Setpoint
  y la velocidad actual de cada motor y te entrega una salida PWM entre -1023 a 1023
  la cual en conjunto con el controlador de motores puede controlar la velocidad en 
  ambas direcciones.
*/
#include "Setup.h"

PIDvelocidad velPID;
PIDvelocidad* p_velPID = &velPID;

void PIDvelocidad_init(PIDvelocidad *pid){
  
  /* Resetear memoria del PID */

  pid->integratorA       = 0;
  pid->prevErrorA        = 0;
  pid->prevTargetVelA    = 0;
  pid->integratorB       = 0;
  pid->prevErrorB        = 0;
  pid->prevTargetVelB    = 0;
  pid->integratorMax     = 50.0f;
  pid->integratorMin     = -50.0f;

  pid->pwmA = 0;
  pid->pwmB = 0;

}

void PIDvelocidad_update(float targetVelA,float targetVelB ,PIDvelocidad *pid, Mouse *m){
  //Calculo de error
  float errorA = targetVelA - m->velocidadMotorA;
  float errorB = targetVelB - m->velocidadMotorB;
  
  //telemetriaOpcional
  telemetriaAngDTerm[contadorTest] = errorB;
  telemetriaAngITerm[contadorTest] = pid->prevErrorB;

  //Predecimos el pwm en base a la velocidad y aceleracion del robot
  float feedForward_A  = pid->k_FF * targetVelA + pid->k_Acc * (targetVelA - pid->prevTargetVelA); 
  float feedForward_B  = pid->k_FF * targetVelB + pid->k_Acc * (targetVelB - pid->prevTargetVelB);
  
  //Actualizamos la velocidad anterior para el calculo de aceleracion instantanea
  pid->prevTargetVelA  = targetVelA;
  pid->prevTargetVelB  = targetVelB;

  //Calculo de ganancia proporcional
  float proportional_A = pid->k_P * errorA;
  float proportional_B = pid->k_P * errorB;

  //telemetriaOpcional
  telemetriaVelPTerm[contadorTest] = proportional_A;

  //Calculo de ganancia integral con metodo de trapecio
  pid->integratorA = pid->integratorA + 0.5f * pid->k_I * pid->T * (errorA + pid->prevErrorA); 
  pid->integratorB = pid->integratorB + 0.5f * pid->k_I * pid->T * (errorB + pid->prevErrorB);

  //Limitar la ganacia integral a un intervalo predefinido
  pid->integratorA = constrain(pid->integratorA, pid->integratorMin, pid->integratorMax);
  pid->integratorB = constrain(pid->integratorB, pid->integratorMin, pid->integratorMax);   

  //Actualizar errores previos y medidas previas
  pid->prevErrorA = errorA;
  pid->prevErrorB = errorB;

  //Sumar y limitar ganancias de PWM
  pid->pwmA = feedForward_A + proportional_A + pid->integratorA;
  pid->pwmB = feedForward_B + proportional_B + pid->integratorB;
  pid->pwmA = constrain(pid->pwmA,-1023,1023);
  pid->pwmB = constrain(pid->pwmB,-1023,1023);

  //Actualizar PWMs en los motores
  setPWM(pid->pwmA,pid->pwmB);

}
