#ifndef VELOCIDAD_HEADER
#define VELOCIDAD_HEADER
//Unidades en mm y rad/s
#define CPR                     7
#define REDUCCION_ENGRANAJE     100
#define DIAMETRO_LLANTA         34
#define RADIO_DE_ROTACION       45
//-----------------------------------------------Declaracion Funciones-----------------------------------------------
void contadorEncoderA();
void contadorEncoderB();
void getRevoluciones_Eje();
void getRevoluciones_Llanta();
void getRPM();


//-----------------------------------------------Extern variables-----------------------------------------------
extern int contadorA;
extern int contadorB;
extern float revolucionesA;
extern float revolucionesB;
extern float rpmA;
extern float rpmB;

#endif