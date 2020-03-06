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
// #define DEVICE_NAME "ESP_ROBO_4"

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
static void _setup();
//interruptions and callbacks
static void IRAM_ATTR isr_EncoderLeft(const void*  arg);
static void IRAM_ATTR isr_EncoderRight(const void* arg);
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

static float reference[2]    = {0.0, 0.0}; //-1.0 a 1.0
static double omegaCurrent[2] = {0.0, 0.0}; //-1.0 a 1.0

static struct Parameters parameters;

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

  /** configuracoes iniciais **/
  //interrupcoes no nucleo 0, dos encoders e do bluetooth
  xTaskCreatePinnedToCore(_setup, "setup_BT_GPIO", 4096, NULL, 1, NULL, 0);
  //loop do controlador no nucleo 1
  xTaskCreatePinnedToCore(periodic_controller, "periodic_controller", 2048, NULL, 4, &controller_xHandle, 1);

  //controlador no nucleo 1
  uint8_t  head, cmd;
  double    *vec_double;
  while(1){
      xQueueReceive(bt_queue, &btdata, portMAX_DELAY);
      head = btdata.data[0] & 0xF0;
      if(head != CMD_HEAD)
        continue;
      cmd = btdata.data[0] & 0x0F;
      switch (cmd) {
      case CMD_REF:
        decodeFloat(btdata.data, &reference[LEFT], &reference[RIGHT]);
        if(bypass_controller)
          bypass_controller = false;
        break;
      case CMD_CONTROL_SIGNAL:
        bypass_controller = true;
        decodeFloat(btdata.data, &reference[LEFT], &reference[RIGHT]);
        func_controlSignal(reference[LEFT], reference[RIGHT]);
        break;
      case CMD_PING:
        esp_spp_write(bt_handle, btdata.len, btdata.data);
        break;
      case CMD_IDENTIFY:
        func_identify((uint8_t)btdata.data[1],                       //options
                      *(float*)&btdata.data[2+0*sizeof(float)]);      //setpoint
        break;
      case CMD_SET_KP:
        parameters.Kp[LEFT] = *(double*)&btdata.data[1 + 0*sizeof(double)];
        parameters.Kp[RIGHT]= *(double*)&btdata.data[1 + 1*sizeof(double)];
        save_parameters(&parameters);
        break;
      case CMD_CALIBRATION:
        func_calibration();
        break;
      case CMD_REQ_OMEGA:
        esp_spp_write(bt_handle, 2*sizeof(double), (uint8_t*)omegaCurrent);
        break;
      case CMD_REQ_CAL:
        vec_double = (double*)malloc((9 + 2)*sizeof(double));//9 coef + 2 Kp

        vec_double[0] = parameters.omegaMax;
        memcpy(vec_double+1, parameters.coef, 8*sizeof(double));

        vec_double[9]  = parameters.Kp[LEFT];
        vec_double[10] = parameters.Kp[RIGHT];

        esp_spp_write(bt_handle, (9+2)*sizeof(double), (uint8_t*)vec_double);
        free(vec_double);

        break;
      default:
        break;
      }
  }
}
//***************************************************************************************
//this function costs about 50us
static void IRAM_ATTR isr_EncoderLeft(const void* arg)
{
  static struct Encoder_data my_data = {0, 0.0};
  static double k = 2.0*M_PI/3.0;
  static int8_t lookup_table[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
  static uint8_t enc_v = 0;

  static bool    aux = true;
  static double  time[2] = {0.0, 0.0};

  enc_v <<= 2;
  enc_v |= (REG_READ(GPIO_IN1_REG) >> (GPIO_OUTB_LEFT - 32)) & 0b0011;

  //WARNING: A logica atual se baseia em utilizar apenas as bordas de subida do canal A para calcular a velocidade
  //mas a interrupcao como um todo faz uso de todas as bordas dos dois canais para a maxima precisao em estimar a orientacao
  //estude uma possibilidade de melhorar a precisao utilizando mais do sensor
  if(arg)
  {
    if(aux)
    {
      time[1] = esp_timer_get_time()/1000000.0;
      my_data.cumOmega += (double)lookup_table[enc_v & 0b1111]*(k/(time[1] - time[0]))*(!!time[0]);//rad/s
      time[0] = time[1];
      my_data.nOmegas++;
    }
    aux = !aux;
  }

  xQueueSendFromISR(encoder_queue[LEFT], &my_data, NULL);
}
//this function costs about 50us
static void IRAM_ATTR isr_EncoderRight(const void* arg)
{
  static int8_t  lookup_table[] = { 0,-1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  static uint8_t  enc_v = 0;
  static uint32_t reg_read;
  static struct Encoder_data my_data = {0, 0.0};
  static double time[2] = {0.0, 0.0};

  static bool   aux = true;
  static double k = 2.0*M_PI/3.0;

  enc_v <<= 2;
  reg_read = REG_READ(GPIO_IN_REG);
  enc_v |= (reg_read & LS(GPIO_OUTA_RIGHT)) >> (GPIO_OUTA_RIGHT-1);
  enc_v |= (reg_read & LS(GPIO_OUTB_RIGHT)) >> GPIO_OUTB_RIGHT;

  //WARNING: A logica atual se baseia em utilizar apenas as bordas de subida do canal A para calcular a velocidade
  //mas a interrupcao como um todo faz uso de todas as bordas dos dois canais para a maxima precisao em estimar a orientacao
  //estude uma possibilidade de melhorar a precisao utilizando mais do sensor
  if(arg)
  {
    if(aux)
    {
      time[1] = esp_timer_get_time()/1000000.0;
      my_data.cumOmega += (double)lookup_table[enc_v & 0b1111]*(k/(time[1] - time[0]))*(!!time[0]);//rad/s
      time[0] = time[1];
      my_data.nOmegas++;
    }
    aux = !aux;
  }

  xQueueSendFromISR(encoder_queue[RIGHT], &my_data, NULL);
}

static void
periodic_controller()
{
  struct Encoder_data enc_datas[2];
  float    pwm[2] = {0.0, 0.0};

  double   prevCumOmega[2]= {0.0, 0.0};
  uint64_t prevNOmegas[2] = {0, 0};

  double erro[2];
  double omegaRef[2];

  uint16_t countVelZero[2] = {1, 1}; //numero de contagem equivalente a 500ms no ciclo do controle
  uint8_t motor;
  double diff;
  while(1)
  {
    vTaskDelay(TIME_CONTROLLER/portTICK_PERIOD_MS);

    //Calculando os omegas atuais
    for(motor = 0; motor < 2; motor++)
    {
      if(xQueueReceive(encoder_queue[motor], &enc_datas[motor], 0) == 0) //buffer vazio
      {
        countVelZero[motor]++;
        if(countVelZero[motor] > TIME_TEST_OMEGA_ZERO/TIME_CONTROLLER)
          omegaCurrent[motor] = 0;
      }else{

        diff = enc_datas[motor].nOmegas - prevNOmegas[motor];
        if(diff != 0)omegaCurrent[motor] = (enc_datas[motor].cumOmega - prevCumOmega[motor])/diff;

        prevCumOmega[motor] = enc_datas[motor].cumOmega;
        prevNOmegas[motor]  = enc_datas[motor].nOmegas;
        countVelZero[motor] = 1;
      }
    }


    if(bypass_controller)
    {
      func_controlSignal(reference[LEFT], reference[RIGHT]);
      continue;
    }

    //Controlador
    for(motor = 0; motor < 2; motor++)
    {
      omegaRef[motor] = reference[motor]*parameters.omegaMax;
      // erro[motor]    = omegaRef[motor] - omegaCurrent[motor];    // rad/s [-omegaMaximo, omegaMaximo]
      // pwm[i] = erro[i]*parameters.Kp[i];
      pwm[motor]  = parameters.coef[MS2i(motor, F_IS_NEG(reference[motor]))].ang*omegaRef[motor] +
                    parameters.coef[MS2i(motor, F_IS_NEG(reference[motor]))].lin*(!!reference[motor]);
    }
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
        memset(reference, 0, 2*sizeof(float));
        bypass_controller = true;
        break;
    case ESP_SPP_START_EVT:
        // vTaskResume(controller_xHandle);
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
  memset(reference, 0, 2*sizeof(float));
  bypass_controller = true;

  #define TIME_MS_WAIT          1000
  #define N_POINTS                20
  #define MIN_PWM                0.1
  #define STEP                  (1.0 - MIN_PWM)/((float)N_POINTS)

  double omegaMax_tmp;
  double minOmegaMax = 999999.9;

  double v_omega[N_POINTS];
  double v_pwm[N_POINTS];
  double K;
  for(int motor = 0; motor < 2; motor++)
  {
    memset(reference, 0, 2*sizeof(float));
    for(int sense = 0; sense < 2; sense++)
    {
      reference[motor] = 0.0;
      vTaskDelay(TIME_MS_WAIT/portTICK_PERIOD_MS);

      for(int i = 0; i < N_POINTS; i++)
      {
        v_pwm[i] = (sense == 0)?MIN_PWM + STEP*i: -MIN_PWM - STEP*i;
        reference[motor] = v_pwm[i];
        vTaskDelay(TIME_MS_WAIT/portTICK_PERIOD_MS);
        v_omega[i]= omegaCurrent[motor];
      }

      linearReg(v_omega, v_pwm, N_POINTS,
                &parameters.coef[MS2i(motor, sense)].ang,
                &parameters.coef[MS2i(motor, sense)].lin);

      minOmegaMax = (ABS_F(v_omega[N_POINTS-1]) < minOmegaMax)?ABS_F(v_omega[N_POINTS-1]):minOmegaMax;

      ESP_LOGI("POTI_INFO", "Motor:%d Sense:%d a:%f b:%f",
               motor, sense, parameters.coef[MS2i(motor, sense)].ang,
                             parameters.coef[MS2i(motor, sense)].lin);
    }
    //calculando tau
    // K = 1.0/parameters.coef[MS2i(motor, sense)].ang;
    // calcular Tau, para 63% da velocidade maxima
    
  }
  memset(reference, 0, 2*sizeof(float));

  //Maior velocidade considerada
  parameters.omegaMax = 0.90*ABS_F(minOmegaMax);
  esp_err_t err = save_parameters(&parameters);
  if(err != ESP_OK) ESP_LOGE("POTI_ERRO","Falha ao salvar os parametros, erro:%s", esp_err_to_name(err));
}
/**WARNING:
** Fixar o steptime e o timeout = 2s
**/
static void func_identify(const uint8_t options,const float setpoint)
{
  bypass_controller  = !!(options & 0x7F);  //bypass ou nao o controlador
  float timeout = 2.0;

  int motor_identify     = ((options & 0x80) >> 7) & 0x01;
  int size = timeout/(TIME_CONTROLLER/1000.0); //captura durante 2 segundos a cada ciclo de controle vai gerar 'size' floats de dados lidos
  double *vec_omegas_identify = (double*)malloc(size*sizeof(double));
  memset(vec_omegas_identify, 0, size*sizeof(double));

  reference[motor_identify]  = setpoint;
  vec_omegas_identify[0] = setpoint*parameters.omegaMax; //aqui eu uso o primeiro float do vetor para guardar a ref. em rad/s
  for(int i = 1; i < size; i++)
  {
    vTaskDelay(TIME_CONTROLLER/portTICK_PERIOD_MS);  //aguarda os 2s
    vec_omegas_identify[i] = omegaCurrent[motor_identify];
  }
  memset(reference, 0, 2*sizeof(float));

  //envia a informacao de quantas medidas foram realizadas
  esp_spp_write(bt_handle, sizeof(int), &size);
  //enviar em blocos de 200, observei que o maximo de bytes transmitidos esta sendo de 990 bytes
  //que equivale a 247.5 floats
  //transmissao dos dados em bloco de 200
  for(int i = 0; i < size; i += 100)
    esp_spp_write(bt_handle, (100)*sizeof(double), (uint8_t*)(vec_omegas_identify+i));

  free(vec_omegas_identify);
}
/*************************************************************************************/
/****************************** CONFIGURACOES ****************************************/
/*************************************************************************************/
static void _setup()
{
  //Config. Non-volatile storage (NVS)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }
  //Carrega os ultimos parametros salvos na memoria Flash
  esp_err_t err = load_parameters(&parameters);
  if(err != ESP_OK)
  {
    ESP_LOGE("POTI_ERRO","Falha ao carregar os parametros, erro:%s", esp_err_to_name(err));
    // if(err == ESP_ERR_NVS_NOT_FOUND)//este esp ainda nao possui dados salvos na memoria
  }

  //Config. Bluetooth
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bt_controller_init(&bt_cfg);
  esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
  esp_bluedroid_init();
  esp_bluedroid_enable();
  esp_spp_register_callback(esp_spp_callback);
  esp_spp_init(ESP_SPP_MODE_CB);
  esp_bt_dev_set_device_name(DEVICE_NAME);

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
  gpio_isr_handler_add(GPIO_OUTA_LEFT, isr_EncoderLeft, 0);
  gpio_isr_handler_add(GPIO_OUTB_LEFT, isr_EncoderLeft, 1);

  gpio_isr_handler_add(GPIO_OUTA_RIGHT,isr_EncoderRight, 0);
  gpio_isr_handler_add(GPIO_OUTB_RIGHT,isr_EncoderRight, 1);

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
