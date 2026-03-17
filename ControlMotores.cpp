#include "Setup.h"
//Motor B -> Derecho, Motor A -> Izquierdo

void avanzar(){
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B2, HIGH);
}

void retroceder(){
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B2, LOW);
}

void setPWM(int pwmA, int pwmB){
  pwmA = constrain(pwmA, DEADZONE_MIN_PWM, DEADZONE_MAX_PWM);
  pwmB = constrain(pwmB, DEADZONE_MIN_PWM, DEADZONE_MAX_PWM);

  ledcWrite(PWM_A, pwmA);
  ledcWrite(PWM_B, pwmB);
}
