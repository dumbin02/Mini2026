#ifndef CONTROLMOTORES_HEADER
#define CONTROLMOTORES_HEADER

#define DEADZONE_MIN_PWM 0
#define DEADZONE_MAX_PWM 1023
// --------Declaracion Funciones--------
void avanzar();
void retroceder();
void setPWM(int pwmA, int pwmB);


#endif