#ifndef VELOCIDAD_PID_HEADER
#define VELOCIDAD_PID_HEADER

typedef struct {
//Gains

  float k_FF = 3.2f;
  float k_Acc = 142.0f;
  float k_P = 10.0f;       //10
  float k_I = 200.0f;     //300

//Limites y deadzoneMin para los motores

  int16_t pwmMax =  1023;
  int16_t pwmMin = -1023;
  float   integratorMax =  50.0f;
  float   integratorMin = -50.0f;

//Tiempo de muestreo en (s)

  float T = 0.001;

//Memoria del controlador

  float integratorA;            //---Motor A---
  float prevErrorA;                 //Memoria parte Integral
  float prevTargetVelA;             //Memoria de aceleracion
  float integratorB;            //---Motor B---
  float prevErrorB;             
  float prevTargetVelB;                  

//Salidas de PWM

  int16_t pwmA = 0;
  int16_t pwmB = 0;

} PIDvelocidad;

extern PIDvelocidad   velPID;
extern PIDvelocidad*  p_velPID;

//Funciones
void PIDvelocidad_init(PIDvelocidad *pid);
void PIDvelocidad_update(float targetVelA,float targetVelB ,PIDvelocidad *pid, Mouse *m);

#endif