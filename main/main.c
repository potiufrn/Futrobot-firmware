/**
 *  Programa teste para o controle local FeedForward
 *  O programa ira identificar a relacao entre PWM e Velocidade de cada motor para cada
 *  sentido de rotacao
 **/

 //Doc util:
// https://www.pjrc.com/teensy/td_libs_Encoder.html#optimize
// https://github.com/igorantolic/ai-esp32-rotary-encoder/tree/master/src

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/mcpwm.h"

#include "nvs.h"
#include "nvs_flash.h"

//Bluetooth
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "esp_attr.h"

#include "common.h"

#include <stdio.h>
#include "esp_log.h"

// #define DEVICE_NAME "ESP_ROBO_TEST"
// #define DEVICE_NAME "ESP_ROBO_1"
#define DEVICE_NAME "ESP_ROBO_2"

#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_STBY)      | \
                             (1ULL << GPIO_A1N1_LEFT) | \
                             (1ULL << GPIO_A1N2_LEFT) | \
                             (1ULL << GPIO_B1N1_RIGHT)| \
                             (1ULL << GPIO_B1N2_RIGHT))

#define GPIO_INPUT_PIN_SEL ( (1ULL << GPIO_OUTA_LEFT)  | \
                             (1ULL << GPIO_OUTB_LEFT)  | \
                             (1ULL << GPIO_OUTA_RIGHT) | \
                             (1ULL << GPIO_OUTB_RIGHT))

/*************************************************************************************/
/********************************* CABEÇALHOS ****************************************/
/*************************************************************************************/
static void config_bluetooth();
static void config_gpio();

//interruptions and callbacks
static void IRAM_ATTR isr_EncoderLeft();
static void IRAM_ATTR isr_EncoderRight();
static void esp_spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
static void periodic_controller();
//my functions
static void func_controlSignal(const float pwmL,const float pwmR);
static void func_calibration();
//options = 0xXY, sendo X 1 bit para indicar o motor, e os demais bits 7bits da parte Y indicando opcoes para o controlador utilizado,
//por enquanto Y serão:
// 0 => controlador padrao
// 1 => bypass controlador
// a coleta as velocidades medidas pelos sensores em intervalos de step_time até
// um tempo total de timeout, retorna um vetor de omegas medidos por referencias e
// o tamanho do vetor por retorno de funcao
static void func_identify(const uint8_t options,
                          const float setpoint);  // -1.0 a 1.0
/*************************************************************************************/
/****************************** VARIAVEIS GLOBAIS ************************************/
/*************************************************************************************/
static xQueueHandle bt_queue = NULL;
static xQueueHandle encoder_queue[2] = {NULL,NULL};

static bool bypass_controller = true;

struct CoefLine coef[4];

static float omega_max = 0.0;   //modulo da velocidade maxima do robo
static float omega_ref[2]       = {0.0, 0.0}; //-1.0 a 1.0
static float omega_current[2]   = {0.0, 0.0}; //-1.0 a 1.0

static float kp[2] = {0.00009652702911, 0.0001116636528}; // kp para testes
static float ki[2] = {0.004, 0.004}; // kp para testes

//Identificador do Bluetooth
static uint32_t bt_handle = 0;
static TaskHandle_t controller_xHandle = 0;

/*************************************************************************************/
/****************************** ROTINAS PRINCIPAIS ***********************************/
/*************************************************************************************/

void app_main()
{
  bt_data btdata;
  bt_queue   = xQueueCreate(1, sizeof(bt_data));
  encoder_queue[LEFT]  = xQueueCreate(1, sizeof(struct Encoder_data));
  encoder_queue[RIGHT] = xQueueCreate(1, sizeof(struct Encoder_data));

  omega_max = 2403.318359;
  coef[LEFT  << 1 | FRONT].ang = 0.000281;
  coef[LEFT  << 1 | FRONT].lin = 0.089219;

  coef[LEFT  << 1 | BACK].ang  = -0.000306;
  coef[LEFT  << 1 | BACK].lin  =  0.088609;

  coef[RIGHT << 1 | FRONT].ang = 0.000275;
  coef[RIGHT << 1 | FRONT].lin = 0.078843;

  coef[RIGHT << 1 | BACK].ang  = -0.000332;
  coef[RIGHT << 1 | BACK].lin  = -0.078233;
  //As 4 retas com os mesmos coeficentes, esses valores foram obtidos por meio de medias
  //de algumas repetições de identificação
  // for(uint8_t motor = 0; motor < 2; motor++)
  // {
  //   for(uint8_t sense = 0; sense < 2; sense++)
  //   {
  //     coef[MS2i(motor, sense)].ang = 0.000272*(1.0+sense*-2.0);
  //     coef[MS2i(motor, sense)].lin = 0.089525*(1.0+sense*-2.0);
  //   }
  // }

  /** configuracoes iniciais **/
  //interrupcoes no nucleo 0
  xTaskCreatePinnedToCore(config_gpio, "config_gpio", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(config_bluetooth, "config_bluetooth", 2048, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(periodic_controller, "periodic_controller", 2048, NULL, 4, &controller_xHandle, 1);

  //controlador no nucleo 1
  uint8_t  head, cmd;
  float    *vec_float;
  while(1){
      xQueueReceive(bt_queue, &btdata, portMAX_DELAY);
      head = btdata.data[0] & 0xF0;
      if(head != CMD_HEAD)
        continue;
      cmd = btdata.data[0] & 0x0F;

      switch (cmd) {
      case CMD_REF:
        decodeFloat(btdata.data, &omega_ref[LEFT], &omega_ref[RIGHT]);
        if(bypass_controller)
          bypass_controller = false;
        break;
      case CMD_CONTROL_SIGNAL:
        bypass_controller = true;
        decodeFloat(btdata.data, &omega_ref[LEFT], &omega_ref[RIGHT]);
        func_controlSignal(omega_ref[LEFT], omega_ref[RIGHT]);
        break;
      case CMD_PING:
        esp_spp_write(bt_handle, btdata.len, btdata.data);
        break;
      case CMD_IDENTIFY:
        func_identify((uint8_t)btdata.data[1],                       //options
                      *(float*)&btdata.data[2+0*sizeof(float)]);      //setpoint
        break;
      case CMD_SET_KP:
        kp[LEFT] = *(float*)&btdata.data[1 + 0*sizeof(float)];
        kp[RIGHT]= *(float*)&btdata.data[1 + 1*sizeof(float)];
        break;
      case CMD_CALIBRATION:
        func_calibration();
        break;
      case CMD_REQ_OMEGA:
        esp_spp_write(bt_handle, 2*sizeof(float), (uint8_t*)omega_current);
        break;
      case CMD_REQ_CAL:
        vec_float = (float*)malloc((9 + 2)*sizeof(float));//9 coef + 2 Kp

        vec_float[0] = omega_max;
        memcpy(vec_float+1, coef, 8*sizeof(float));

        vec_float[9]  = kp[LEFT];
        vec_float[10] = kp[RIGHT];

        esp_spp_write(bt_handle, (9+2)*sizeof(float), (uint8_t*)vec_float);
        free(vec_float);

        break;
      default:
        break;
      }
  }
}
//***************************************************************************************
static void IRAM_ATTR isr_EncoderLeft()
{
  static int8_t  lookup_table[] = { 0,1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
  static uint8_t enc_v = 0;
  static struct Encoder_data my_data = {0, 0.0};
  static int8_t last = 0;
  static double lastTime = 0, newTime;

  newTime = get_time_sec();
  my_data.dt = newTime - lastTime;
  lastTime = newTime;

  enc_v <<= 2;
  enc_v |= (REG_READ(GPIO_IN1_REG) >> (GPIO_OUTB_LEFT - 32)) & 0b0011;

  // my_data.pulse_counter += lookup_table[enc_v & 0b111];
  //utiliza a ultima decodificação quando há falha(decode = 0)
  my_data.pulse_counter += (lookup_table[enc_v & 0b111] == 0)?last:lookup_table[enc_v & 0b111];
  if(lookup_table[enc_v & 0b111] != 0)
    last = lookup_table[enc_v & 0b111];

  /*atualiza buffer*/
  xQueueSendFromISR(encoder_queue[LEFT], &my_data, NULL);
}
static void IRAM_ATTR isr_EncoderRight()
{
  static int8_t  lookup_table[] = { 0,-1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  static uint8_t  enc_v = 0;
  static uint32_t reg_read;
  static struct Encoder_data my_data = {0, 0.0};
  static int8_t last = 0;
  static double lastTime = 0, newTime;

  newTime = get_time_sec();
  my_data.dt = newTime - lastTime;
  lastTime = newTime;

  enc_v <<= 2;
  reg_read = REG_READ(GPIO_IN_REG);
  enc_v |= (reg_read & LS(GPIO_OUTA_RIGHT)) >> (GPIO_OUTA_RIGHT-1);
  enc_v |= (reg_read & LS(GPIO_OUTB_RIGHT)) >> GPIO_OUTB_RIGHT;

  my_data.pulse_counter += (lookup_table[enc_v & 0b111] == 0)?last:lookup_table[enc_v & 0b111];
  if(lookup_table[enc_v & 0b111] != 0)
    last = lookup_table[enc_v & 0b111];

  xQueueSendFromISR(encoder_queue[RIGHT], &my_data, NULL);
}

static void
periodic_controller()
{
  //Sabendo que uma volta completa no eixo do encoder gera 12 interrupcoes
  //ou seja, 2pi => 12 interrupcoes
  const double k1 = (M_PI/6.0); //para dt variavel
  const double k2 = k1/(TIME_CONTROLLER/1000.0); //constante pra converter de pulsos/s para rad/s, para dt fixo
  float pwm[2];
  struct Encoder_data enc_datas[2];
  int64_t old_pulse_counter[2] = {0, 0};

  double erro[2];
  double cumErro[2]= {0.0, 0.0};
  double ref_rad[2];

  uint16_t countVelZero[2] = {TIME_TEST_OMEGA_ZERO/TIME_CONTROLLER, TIME_TEST_OMEGA_ZERO/TIME_CONTROLLER}; //numero de contagem equivalente a 500ms no ciclo do controle

  TickType_t xLastWakeTime = xTaskGetTickCount();
  int i;
  while(1)
  {
    vTaskDelayUntil( &xLastWakeTime, TIME_CONTROLLER/portTICK_PERIOD_MS);

    //Calculando os omegas atuais
    for(i = 0; i < 2; i++)
    {
      if(xQueueReceive(encoder_queue[i], &enc_datas[i], 0) == 0) //buffer vazio
      {
        countVelZero[i]--;
        if(countVelZero[i] <= 0)
        {
          omega_current[i] = 0;
        }
      }else{
        omega_current[i]  = (enc_datas[i].pulse_counter - old_pulse_counter[i])*k2;//*0.5 + omega_lin[LEFT]*0.5;
        old_pulse_counter[i] = enc_datas[i].pulse_counter;
        // if(enc_datas[i].dt != 0)
        //   omega_current[i]  = (1 - 2*F_IS_NEG(enc_datas[i].pulse_counter))*(1.0/enc_datas[i].dt)*k1;
        // if(i == 0)ESP_LOGI("LOG!!!","Omega:%lf Dt:%lf", omega_current[i], enc_datas[i].dt);
        countVelZero[i]   = TIME_TEST_OMEGA_ZERO/TIME_CONTROLLER;
      }
    }

    if(bypass_controller)
    {
      func_controlSignal(omega_ref[LEFT], omega_ref[RIGHT]);
      continue;
    }

    //Controlador
    memset(pwm, 0, 2*sizeof(float));
    for(i = 0; i < 2; i++)
    {
      ref_rad[i]  = omega_ref[i]*omega_max;
      erro[i]     = ref_rad[i] - omega_current[i];    // rad/s [-omegaMaximo, omegaMaximo]
      cumErro[i] += erro[i]*(TIME_CONTROLLER/1000.0);

      pwm[i]  = (erro[i]*kp[i] + cumErro[i]*ki[i]*(ABS_F(pwm[i]) <= 1.0))*!!(ref_rad[i]); // PU [-1.0, 1.0]

      // pwm[i] += coef[MS2i(i, F_IS_NEG(omega_ref[i]))].ang*ref_rad[i] +
      //           coef[MS2i(i, F_IS_NEG(omega_ref[i]))].lin*(!!omega_ref[i]);

      //INFO: !!omega_ref eh para zerar a contribuicao do lin quando omega_Ref for zero e para nao influenciar no valor quando for diferente de 0, pois com "!!" esse valor ou e 0 ou 1.
    }
    // ESP_LOGI("DEBUG", "Motor:%d PWM:%f Omega:%f", 1, pwm[1], omega_current[1]);
    func_controlSignal(pwm[LEFT], pwm[RIGHT]);
  }
}

//WARNING usar o acesso direto aos registradores para efetuar as operacoes de
//mudanca de nivel dos gpios. Obs.: tambem eh possivel alterar o dutycicle por registradores
static void func_controlSignal(const float pwmL,const float pwmR)
{
  #define SAT(x) (((x) > 1.0)?1.0:(x))
  static bool front[2]  = {false, false};
  front[LEFT]  = !F_IS_NEG(pwmL);
  front[RIGHT] = !F_IS_NEG(pwmR);

  REG_WRITE(GPIO_OUT_REG, (1 << GPIO_STBY) |
                          (!front[LEFT]  << GPIO_A1N1_LEFT)  | (front[LEFT]  << GPIO_A1N2_LEFT) |
                          (!front[RIGHT] << GPIO_B1N1_RIGHT)  | (front[RIGHT] << GPIO_B1N2_RIGHT));
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, SAT(ABS_F(pwmL))*100.0);  //set PWM motor esquerdo
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0B, SAT(ABS_F(pwmR))*100.0);//set PWM motor direito
}
//Funcao de tratamento de eventos do bluetooth
static void
esp_spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  bt_data btdata;
    switch (event){
    case ESP_SPP_INIT_EVT:
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "ESP32_SPP_SERVER");
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        break;
    case ESP_SPP_OPEN_EVT:
        bt_handle = param->open.handle;
        break;
    case ESP_SPP_CLOSE_EVT:
        bt_handle = 0;
        memset(omega_ref, 0, 2*sizeof(float));
        bypass_controller = true;
        break;
    case ESP_SPP_START_EVT:
        vTaskResume(controller_xHandle);
        break;
    case ESP_SPP_CL_INIT_EVT:
        break;
    case ESP_SPP_DATA_IND_EVT:
        btdata.data = param->data_ind.data;
        btdata.len  = param->data_ind.len;
        xQueueSend(bt_queue, &btdata, NULL);
        break;
    case ESP_SPP_CONG_EVT:
        break;
    case ESP_SPP_WRITE_EVT:
        break;
    case ESP_SPP_SRV_OPEN_EVT:
      bt_handle = param->open.handle;
        break;
    default:
        break;
    }
}
//Refazer
static void func_calibration()
{
  memset(omega_ref, 0, 2*sizeof(float));
  bypass_controller = true;

  #define TIME_MS_WAIT           500
  #define MIN_INIT               0.2
  #define TIME_MS_WAIT_MIN       150
  #define TIME_MS_WAIT_VEL_MAX  1000
  const float STEP = 5.0/32767.0;    //por causa da resolucao de 15 bits na transmissao da ref

  float omega_max_tmp[4];
  float pwmMin[4];
  float minOmegaMax = 0.0;
  float refmin;

  for(int motor = 0; motor < 2; motor++)
  {
    memset(omega_ref, 0, 2*sizeof(float));
    for(int sense = 0; sense < 2; sense++)
    {
      //medir velocidade maxima
      //esse trecho seginte eh para evitar uma subida abrupta de 0 ah 100
      omega_ref[motor] = 0.7 - 2*0.7*(sense);
      vTaskDelay(TIME_MS_WAIT/portTICK_PERIOD_MS);
      //equacao para dar 1 quando sense == 0, e -1 para sense == 1
      omega_ref[motor] = 1.0 - 2.0*(sense);
      vTaskDelay(TIME_MS_WAIT_VEL_MAX/portTICK_PERIOD_MS);
      omega_max_tmp[MS2i(motor, sense)] = omega_current[motor];
      minOmegaMax = (omega_max_tmp[MS2i(motor, sense)] < minOmegaMax)?omega_max_tmp[MS2i(motor, sense)]:minOmegaMax;

      //medir pwm minimo
      //Ideia base: reduzir ref(pwm) ate descobrir o minimo necessario para fazer o motor rotacionar.
      refmin = MIN_INIT - 2.0*MIN_INIT*sense;
      while(omega_current[motor] != 0.0)
      {
        refmin += 2*STEP*sense - STEP;
        omega_ref[motor] = refmin;
        vTaskDelay(TIME_MS_WAIT_MIN/portTICK_PERIOD_MS);
      }

      refmin += STEP - 2*STEP*sense;  //volta um incremento/decremento para deixar no pwm antes do pwm que fez parar o motor
      pwmMin[MS2i(motor, sense)] = refmin;

      //Calculo dos coef.
      coef[MS2i(motor, sense)].ang  = (1.0 - pwmMin[MS2i(motor, sense)])/(omega_max_tmp[MS2i(motor, sense)]);
      coef[MS2i(motor, sense)].lin  = pwmMin[MS2i(motor, sense)];
    }
  }
  memset(omega_ref, 0, 2*sizeof(float));

  //Maior velocidade considerada
  omega_max = 0.90*ABS_F(minOmegaMax);
}
/**WARNING:
** Fixar o steptime e o timeout = 2s
**/
static void func_identify(const uint8_t options,const float setpoint)
{
  bypass_controller  = !!(options & 0x7F);  //bypass ou nao o controlador
  memset(omega_ref, 0, 2*sizeof(float));

  int motor_identify     = ((options & 0x80) >> 7) & 0x01;
  int size = 2.0/(TIME_CONTROLLER/1000.0); //captura durante 2 segundos a cada ciclo de controle vai gerar 'size' floats de dados lidos
  float *vec_omegas_identify = (float*)malloc(size*sizeof(float));
  memset(vec_omegas_identify, 0, size*sizeof(float));

  int steptime = 2000/size;
  omega_ref[motor_identify]  = setpoint;
  for(int i = 0; i < size; i++)
  {
    vTaskDelay(steptime/portTICK_PERIOD_MS);  //aguarda os 2s
    vec_omegas_identify[i] = omega_current[motor_identify];
  }

  memset(omega_ref, 0, 2*sizeof(float));

  //envia a informacao de quantas medidas foram realizadas
  esp_spp_write(bt_handle, sizeof(int), &size);
  //enviar em blocos de 200, observei que o maximo de bytes transmitidos esta sendo de 990 bytes
  //que equivale a 247.5 floats
  //transmissao dos dados em bloco de 200
  for(int i = 0; i < size; i += 200)
    esp_spp_write(bt_handle, (200)*sizeof(float), (uint8_t*)vec_omegas_identify+i);

  free(vec_omegas_identify);
}
/*************************************************************************************/
/****************************** CONFIGURACOES ****************************************/
/*************************************************************************************/
static void config_bluetooth()
{
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      nvs_flash_erase();
      nvs_flash_init();
  }
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bt_controller_init(&bt_cfg);
  esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
  esp_bluedroid_init();
  esp_bluedroid_enable();
  esp_spp_register_callback(esp_spp_callback);
  esp_spp_init(ESP_SPP_MODE_CB);
  esp_bt_dev_set_device_name(DEVICE_NAME);

  vTaskDelete(NULL);
}
static void config_gpio(){
  //##### SET GPIO CONFIG ########
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);

  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 1;
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  gpio_config(&io_conf);

  gpio_install_isr_service(0);
  gpio_isr_handler_add(GPIO_OUTA_LEFT, isr_EncoderLeft, NULL);
  gpio_isr_handler_add(GPIO_OUTB_LEFT, isr_EncoderLeft, NULL);

  gpio_isr_handler_add(GPIO_OUTA_RIGHT,isr_EncoderRight, NULL);
  gpio_isr_handler_add(GPIO_OUTB_RIGHT,isr_EncoderRight, NULL);

  mcpwm_pin_config_t pin_configLeft = {
      .mcpwm0a_out_num = GPIO_PWM_LEFT,
      .mcpwm0b_out_num = GPIO_PWM_RIGHT,
  };
  mcpwm_set_pin(MCPWM_UNIT_0, &pin_configLeft);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = 10000;    //frequency = 10kHz
  pwm_config.cmpr_a = 0.0;       //duty cycle of PWMxA = 0.0%
  pwm_config.cmpr_b = 0.0;       //duty cycle of PWMxA = 0.0%
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings

  vTaskDelete(NULL);
}
