#include "Setup.h"
//Motor B -> Derecho, Motor A -> Izquierdo

bool direccionMotorA = 1;
bool direccionMotorB = 1;

void avanzar(){
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B2, LOW);
}

void detenerse(){
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B2, LOW);
}

void giro_diferencial_derecha(){
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B2, HIGH);
}

void giro_diferencial_izquierda(){
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B2, LOW);
}

void retroceder(){
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B2, HIGH);
}

void setPWM(int pwmA, int pwmB){
  if(pwmA >= 0){
    ledcWrite(PWM_A, pwmA);
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, HIGH); 
    if(MM.velocidadMotorA >= -10) direccionMotorA = 1;   
  } else{
    pwmA = -(pwmA);
    ledcWrite(PWM_A, pwmA);
    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
    if(MM.velocidadMotorA <= 10) direccionMotorA = 0;
  }
  if(pwmB >= 0){
    ledcWrite(PWM_B, pwmB);
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
    if(MM.velocidadMotorB >= -10) direccionMotorB = 1;
  }else{
    pwmB = -(pwmB);
    ledcWrite(PWM_B, pwmB);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, HIGH);
    if(MM.velocidadMotorB <= 10) direccionMotorB = 0;
  }
  if(pwmA == 0 && pwmB == 0) detenerse();
}



