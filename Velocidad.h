#ifndef VELOCIDAD_HEADER
#define VELOCIDAD_HEADER
//Unidades en mm y rad/s
#define CPR                     7
#define REDUCCION_ENGRANAJE     100
#define DIAMETRO_LLANTA         32
#define RADIO_DE_ROTACION       45
#define DISTANCIA_ENTRE_RUEDAS  83
//-----------------------------------------------Declaracion Funciones-----------------------------------------------
void contadorEncoderA();
void contadorEncoderB();
uint64_t getTiempoEncoderAINT();
uint64_t getTiempoEncoderBINT();
void getDistanciaRecorrida();
void getVelocidades();
void getVelocidadAng();
void resetearContadoresDeEncoders();

#endif