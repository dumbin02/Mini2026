#ifndef CONTROLMOTORES_HEADER
#define CONTROLMOTORES_HEADER


// --------Declaracion Funciones--------
void avanzar();
void detenerse();
void retroceder();
void setPWM(int pwmA, int pwmB);
void giro_diferencial_derecha();
void giro_diferencial_izquierda();

extern bool direccionMotorA,direccionMotorB;

#endif