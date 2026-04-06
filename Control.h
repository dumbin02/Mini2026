#ifndef CONTROL_HEADER
#define CONTROL_HEADER
//Unidades en mm y rad/s
#define RADIO_DE_ROTACION   45
#define DISTANCIA_ENTRE_RUEDAS  83
#define CONTADOR_PULSO_VEL_MAX 2
//-------------------------------------------------------------------------------------------------------------------
enum Acciones : uint8_t {
  DETENERSE = 0,
  LINEA_RECTA_PID,
  VUELTA_90_DER,
  VUELTA_90_IZQ,
  VUELTA_180,
  MANTENER_ANGULO,
  ESPERAR_A_BOTON,
  ENVIAR_TELEMETRIA
};
// Variable para el estado actual
extern Acciones estadoActual;
//-----------------------------------------------Declaracion Funciones-----------------------------------------------
void calcular_dt();
void calcular_dtContadorA();
void calcular_dtContadorB();
void actualizarOdometriaSensores();
//Tests
void targetVelocity1();
void targetVelocity2();
void targetVelocityConst(float vel_A, float vel_B);
void noGirarConGyro();

//-----------------------------------------------Extern variables-----------------------------------------------
extern volatile float dtContadorA,dtContadorB;
extern int64_t tiempoPrevio,tiempoActualTelemetria,tiempoPrevioTelemetria,tiempoInicio,tiempoFinal;
extern bool endOfRunFlag,resetFlag;
extern int contadorTest;
#endif