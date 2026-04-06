#include "soc/gpio_num.h"
#include "driver/gpio.h"
#include "esp32-hal-gpio.h"
#include "esp_adc/adc_continuous.h"
#include "esp_timer.h"
#include "Arduino.h"
#include "FunctionalInterrupt.h"
#include "esp32-hal-ledc.h"
#include "Setup.h"

Mouse MM = {0,0,NORTE,0,0,1,1,0,0,0,0,0,0,0,0,0};
Mouse* p_MM = &MM;

//Flag para el uso del boton
            bool botonFlag = 0;
  volatile  bool controlLoopFlag = false;
  volatile  bool telemetryLoopFlag = false;

adc_continuous_handle_t handle = NULL;

esp_timer_handle_t  timer_control;

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

void timerCallback(void* arg){
  controlLoopFlag = true;
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

  iniciarPines();
  setup_adc_dma();
}

