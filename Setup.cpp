#include "soc/gpio_num.h"
#include "driver/gpio.h"
#include "esp32-hal-gpio.h"
#include "esp_adc/adc_continuous.h"
#include "esp_timer.h"
#include "Arduino.h"
#include "FunctionalInterrupt.h"
#include "esp32-hal-ledc.h"
#include "Setup.h"

//Flag para el uso del boton
bool botonFlag = 0;

adc_continuous_handle_t handle = NULL;

esp_timer_handle_t  timer_control;
esp_timer_handle_t  timer_telemetry;
volatile bool       controlLoopFlag = false;
volatile bool       telemetryLoopFlag = false;

void setup_adc_dma() {
    gpio_reset_pin(GPIO_NUM_1);
    
    // Configuración del Handle
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = 256,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    // Configuración Continua (CORREGIDO para 3.3.7)
    adc_continuous_config_t config = {
        .pattern_num = 3,
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1, // Solo usamos ADC1
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    // Definimos los patrones de los pines 0, 1 y 2
    static adc_digi_pattern_config_t adc_pattern[3];
    adc_pattern[0] = {.atten = ADC_ATTEN_DB_12, .channel = ADC_CHANNEL_0, .unit = ADC_UNIT_1, .bit_width = ADC_BITWIDTH_12};
    adc_pattern[1] = {.atten = ADC_ATTEN_DB_12, .channel = ADC_CHANNEL_1, .unit = ADC_UNIT_1, .bit_width = ADC_BITWIDTH_12};
    adc_pattern[2] = {.atten = ADC_ATTEN_DB_12, .channel = ADC_CHANNEL_2, .unit = ADC_UNIT_1, .bit_width = ADC_BITWIDTH_12};
    
    config.adc_pattern = adc_pattern;

    ESP_ERROR_CHECK(adc_continuous_config(handle, &config));
    ESP_ERROR_CHECK(adc_continuous_start(handle));
}

void iniciarPines(){
  
  pinMode(MOTOR_A1,         OUTPUT);
  pinMode(MOTOR_B1,         OUTPUT);
  pinMode(MOTOR_B2,         OUTPUT);

#if SETUP_BOTON
  pinMode(MOTOR_A2, INPUT_PULLDOWN);
#endif

  ledcAttach(PWM_A, PWM_FREQ, PWM_RESOLUCION);
  ledcAttach(PWM_B, PWM_FREQ, PWM_RESOLUCION);

#if SETUP_ENCODER
  attachInterrupt(ENCODER_A, contadorEncoderA, RISING);
  attachInterrupt(ENCODER_B, contadorEncoderB, RISING);
#endif
}



void initMPU() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(1000000);           // ¡Vital para la velocidad!

  // 1. Wake up (Registro 0x6B)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0x01);                // Usar el reloj del Giroscopio X (más estable que el interno)
  Wire.endTransmission(true);

  // 2. Configurar Filtro Digital Low Pass (DLPF) - Registro 0x1A
  // Un valor de 3 o 4 (aprox 42Hz) suele ser ideal para filtrar motores
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); 
  Wire.write(0x05);                // DLPF_CFG = 3
  Wire.endTransmission(true);

  // 3. Configurar Sensibilidad del Giroscopio - Registro 0x1B
  // 0x00 = 250deg/s | 0x08 = 500deg/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);                // 500 deg/s
  Wire.endTransmission(true);
}

void timerCallback(void* arg){
  controlLoopFlag = true;
}

void telemetryCallback(void* arg){
  telemetryLoopFlag = true;
}

void setupTimer(){

  const esp_timer_create_args_t timer_args = {
    .callback = &timerCallback,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "control_timer"
  };

  esp_timer_create(&timer_args, &timer_control);

  esp_timer_start_periodic(timer_control, 1000); // 1000 µs = 1 kHz

  const esp_timer_create_args_t timer_argsTelemetry = {
    .callback = &telemetryCallback,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "telemetry_timer"
  };

  esp_timer_create(&timer_argsTelemetry, &timer_telemetry);

  esp_timer_start_periodic(timer_telemetry, 10000); // 1000 µs = 20 Hz
}

#if SETUP_BOTON
bool esperarABoton(){
  delay(20);
  botonFlag = digitalRead(MOTOR_A2);
  if (botonFlag == true){
    botonFlag = false;
    pinMode(MOTOR_A2, OUTPUT);
    avanzar();
    return true;
  } else {
    return false;
    }
}
#endif

void mmSetup(){
#if SETUP_SERIAL
  Serial.begin(115200);
  Serial.println("Serial Conectado");
#endif

#if SETUP_BLE
  setupBLE();
#endif

#if SETUP_MPU
  initMPU();
#endif

  iniciarPines();
  setup_adc_dma();
}

