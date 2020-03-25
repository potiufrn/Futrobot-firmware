#include "common.h"
/****************************** VARIAVEIS GLOBAIS ************************************/
static xQueueHandle bt_queue = NULL;
static xQueueHandle encoder_queue[2] = {NULL,NULL};
static bool bypass_controller = true;
static float  reference[2]    = {0.0, 0.0}; //-1.0 a 1.0
static double omegaCurrent[2] = {0.0, 0.0}; //-1.0 a 1.0
static parameters_t parameters;
//Identificador do Bluetooth
static uint32_t bt_handle = 0;
/********************************* CABEÇALHOS ****************************************/
static void _setup();
//interruptions and callbacks
static void IRAM_ATTR isr_EncoderLeft(void *arg);
static void IRAM_ATTR isr_EncoderRight(void *arg);
static void esp_spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
static void periodic_controller();
static void func_calibration();
static void func_identify(const uint8_t options,//options => motor | bypass_controller
                          const float setpoint);  // -1.0 a 1.0
/****************************** ROTINAS PRINCIPAIS ***********************************/

void app_main()
{
  bt_data_t btdata;
  bt_queue   = xQueueCreate(1, sizeof(bt_data_t));
  encoder_queue[LEFT]  = xQueueCreate(1, sizeof(encoder_data_t));
  encoder_queue[RIGHT] = xQueueCreate(1, sizeof(encoder_data_t));

  /** configuracoes iniciais **/
  _setup();//interrupcoes no nucleo 0, dos encoders e do bluetooth
  //loop do controlador no nucleo 1
  xTaskCreatePinnedToCore(periodic_controller, "periodic_controller", 2048, NULL, 4, NULL, 1);

  //controlador no nucleo 1
  uint8_t  head, cmd;
  double   *vec_double;
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
        vec_double = (double*)malloc((9 + 4)*sizeof(double));//9 coef + 4 Kp

        vec_double[0] = parameters.omegaMax;
        memcpy(vec_double+1, parameters.coef, 8*sizeof(double));

        vec_double[9]  = parameters.Kp[LEFT << 1 | FRONT];
        vec_double[10] = parameters.Kp[LEFT << 1 | BACK];
        vec_double[11] = parameters.Kp[RIGHT<< 1 | FRONT];
        vec_double[12] = parameters.Kp[RIGHT<< 1 | BACK];

        esp_spp_write(bt_handle, (9+4)*sizeof(double), (uint8_t*)vec_double);
        free(vec_double);

        break;
      default:
        break;
      }
  }
}

//***************************************************************************************
//this function costs about ?us
static void IRAM_ATTR isr_EncoderLeft(void *arg)
{
  static int8_t lookup_table[] = {0, 1, -1, 0, 0, 0, 0, 1, 0, 0, 0, -1, 0, 1, -1, 0};
  static encoder_data_t my_data = {0, 0.0, 0.0, 0.0};
  static uint8_t enc_v = 0;
  static double k = 2.0*M_PI/3.0;
  static uint32_t prevTime[2] = {0, 0};
  static uint32_t currentTime[2] = {0, 0};
  static double dt = 0.0;
  static uint8_t ch = 0;

  enc_v <<= 2;
  enc_v |= (REG_READ(GPIO_IN1_REG) >> (GPIO_OUTB_LEFT - 32)) & 0b0011;

  ch = ((uint32_t)arg)&0b1;
  currentTime[ch] = esp_timer_get_time();
  dt = (currentTime[ch] - prevTime[ch])/1000000.0;
  prevTime[ch] = currentTime[ch];

  my_data.rawOmega  = k*(double)lookup_table[enc_v & 0b1111]/dt;
  my_data.cumOmega += my_data.rawOmega;//rad/s
  my_data.nOmegas++;

  xQueueSendFromISR(encoder_queue[LEFT], &my_data, NULL);
}
//this function costs about ?us
static void IRAM_ATTR isr_EncoderRight(void* arg)
{
  static int8_t lookup_table[] = {0, -1, 1, 0, 0, 0, 0, -1, 0, 0, 0, 1, 0, -1, 1, 0};
  static encoder_data_t my_data = {0, 0.0, 0.0, 0.0};
  static uint8_t enc_v = 0;
  static double k = 2.0*M_PI/3.0;
  static uint32_t reg_read;
  static uint32_t prevTime[2] = {0, 0};
  static uint32_t currentTime[2] = {0, 0};
  static double dt = 0.0;
  static uint8_t ch = 0;

  enc_v <<= 2;
  reg_read = REG_READ(GPIO_IN_REG);
  enc_v |= (reg_read & LS(GPIO_OUTA_RIGHT)) >> (GPIO_OUTA_RIGHT-1);
  enc_v |= (reg_read & LS(GPIO_OUTB_RIGHT)) >> GPIO_OUTB_RIGHT;

  ch = ((uint32_t)arg)&0b1;
  currentTime[ch] = esp_timer_get_time();
  dt = (currentTime[ch] - prevTime[ch])/1000000.0;
  prevTime[ch] = currentTime[ch];

  my_data.rawOmega  = k*(double)lookup_table[enc_v & 0b1111]/dt;
  my_data.cumOmega += my_data.rawOmega;//rad/s
  my_data.nOmegas++;

  xQueueSendFromISR(encoder_queue[RIGHT], &my_data, NULL);
}
static void
periodic_controller()
{
  encoder_data_t enc_datas[2];
  float    pwm[2] = {0.0, 0.0};

  // double   prevCumOmega[2]= {0.0, 0.0};
  // uint64_t prevNOmegas[2] = {0, 0};
  // double diff;

  double erro[2];
  double omegaRef[2];

  uint16_t countVelZero[2] = {1, 1}; //numero de contagem equivalente a 500ms no ciclo do controle
  uint8_t motor;
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
        // diff = enc_datas[motor].nOmegas - prevNOmegas[motor];
        // if(diff != 0)omegaCurrent[motor] = (enc_datas[motor].cumOmega - prevCumOmega[motor])/diff;
        // prevCumOmega[motor] = enc_datas[motor].cumOmega;
        // prevNOmegas[motor]  = enc_datas[motor].nOmegas;
        omegaCurrent[motor] = enc_datas[motor].rawOmega;
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
      erro[motor]= omegaRef[motor] - omegaCurrent[motor];    // rad/s [-omegaMaximo, omegaMaximo]

      //Ação proporcional
      pwm[motor] = erro[motor]*parameters.Kp[MS2i(motor, F_IS_NEG(reference[motor]))];

      //Forward
      pwm[motor] += parameters.coef[MS2i(motor, F_IS_NEG(reference[motor]))].ang*omegaRef[motor] +
                    parameters.coef[MS2i(motor, F_IS_NEG(reference[motor]))].lin*(!!reference[motor]);
    }
    func_controlSignal(pwm[LEFT], pwm[RIGHT]);
  }
}
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
  esp_spp_write(bt_handle, sizeof(int), (void*)&size);
  //enviar em blocos de 200, observei que o maximo de bytes transmitidos esta sendo de 990 bytes
  //que equivale a 247.5 floats
  //transmissao dos dados em bloco de 200
  for(int i = 0; i < size; i += 100)
    esp_spp_write(bt_handle, (100)*sizeof(double), (uint8_t*)(vec_omegas_identify+i));

  free(vec_omegas_identify);
}
static void func_calibration()
{
 memset(reference, 0, 2*sizeof(float));
 bypass_controller = true;

 #define N_POINTS                20
 #define TIMEOUT                 50   //ms
 #define STEP_TIME                5   //ms
 #define N_IT                   (TIMEOUT/STEP_TIME)

 #define TIME_MS_WAIT          1000
 #define MIN_PWM               0.15
 #define STEP                  (1.0 - MIN_PWM)/((float)N_POINTS)

 double minOmegaMax = 999999.9;
 double *x = NULL;
 double *y = NULL;
 double K;
 double tau = 0.0;
 uint32_t prevTime = 0;

 for(int motor = 0; motor < 2; motor++)
 {
   memset(reference, 0, 2*sizeof(float));
   for(int sense = 0; sense < 2; sense++)
   {
     /*Estimando a zona morta e o ganho do sistema*/
     x = (double*)malloc(N_POINTS*sizeof(double));
     y = (double*)malloc(N_POINTS*sizeof(double));

     reference[motor] = 0.0;
     vTaskDelay(TIME_MS_WAIT/portTICK_PERIOD_MS);

     for(uint32_t i = 0; i < N_POINTS; i++)
     {
       reference[motor] = (sense == 0)?MIN_PWM + STEP*i: -(MIN_PWM + STEP*i);
       vTaskDelay(TIME_MS_WAIT/portTICK_PERIOD_MS);
       x[i] = omegaCurrent[motor];
       y[i] = reference[motor];
     }
     reference[motor] = 0.0;
     vTaskDelay(TIME_MS_WAIT/portTICK_PERIOD_MS);

     linearReg(x, y, N_POINTS,
                     &parameters.coef[MS2i(motor, sense)].ang,
                     &parameters.coef[MS2i(motor, sense)].lin);
     minOmegaMax = (ABS_F(x[N_POINTS-1]) < minOmegaMax)?ABS_F(x[N_POINTS-1]):minOmegaMax;
     K = 1.0/parameters.coef[MS2i(motor, sense)].ang;

     ESP_LOGI("POTI_INFO", "Motor:%d Sense:%d a:%f b:%f",
              motor, sense, parameters.coef[MS2i(motor, sense)].ang,
                            parameters.coef[MS2i(motor, sense)].lin);
     /*Estimando a constante de tempo*/
     free(x);
     free(y);
     x = (double*)malloc(N_IT*sizeof(double));
     y = (double*)malloc(N_IT*sizeof(double));

     /*Regressao*/
     prevTime = esp_timer_get_time();
     reference[motor] = 1.0*(!sense?1.0:-1.0);
     for(uint32_t i = 0; i < N_IT; i++)
     {
       x[i] = (esp_timer_get_time() - prevTime)/1000000.0;
       y[i] = omegaCurrent[motor];
       vTaskDelay(STEP_TIME/portTICK_PERIOD_MS);
     }
     tau = _calcTau(x, y, N_IT, reference[motor]*K);
     reference[motor] = 0.0;
     vTaskDelay(TIME_MS_WAIT/portTICK_PERIOD_MS);

     // double wss = 1.0*K;
     // double tau2;
     // reference[motor] = 1.0*(!sense?1.0:-1.0);
     // prevTime = esp_timer_get_time();
     // while(1)
     // {
     //   if(abs(omegaCurrent[motor]) >= 0.63*wss)
     //   {
     //     tau2 = (esp_timer_get_time() - prevTime)/1000000.0;
     //     break;
     //   }
     // }
     // reference[motor] = 0.0;
     // vTaskDelay(TIME_MS_WAIT/portTICK_PERIOD_MS);

     /*Calcula kp para o polo do sistema ficar em Sd*/
     parameters.Kp[MS2i(motor, sense)] = (Sd*tau - 1.0)/K;
     free(x);
     free(y);

     ESP_LOGI("POTI_INFO", "tau_reg:%lf K:%lf Kp:%lf",
                            tau, K, parameters.Kp[MS2i(motor, sense)]);
   }
 }
 memset(reference, 0, 2*sizeof(float));
 //Maior velocidade considerada
 parameters.omegaMax = 0.90*ABS_F(minOmegaMax);
 esp_err_t err = save_parameters(&parameters);
 if(err != ESP_OK) ESP_LOGE("POTI_ERRO","Falha ao salvar os parametros, erro:%s", esp_err_to_name(err));
}
//Funcao de tratamento de eventos do bluetooth
static void
esp_spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  bt_data_t btdata;
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
        break;
    case ESP_SPP_CL_INIT_EVT:
        break;
    case ESP_SPP_DATA_IND_EVT:
        btdata.data = param->data_ind.data;
        btdata.len  = param->data_ind.len;
        xQueueSend(bt_queue, (void*)&btdata, 1);
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
/****************************** CONFIGURACOES ****************************************/
static void _setup()
{
  #define ESP_INTR_FLAG_DEFAULT 0
  #define CHANNEL_A 0
  #define CHANNEL_B 1
  //Config. Non-volatile storage (NVS)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  //Carrega os ultimos parametros salvos na memoria Flash
  esp_err_t err = load_parameters(&parameters);
  if(err != ESP_OK)ESP_LOGE("POTI_ERRO","Falha ao carregar os parametros, erro:%s", esp_err_to_name(err));

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

  io_conf.intr_type = GPIO_INTR_POSEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 1;
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  gpio_config(&io_conf);

  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(GPIO_OUTA_LEFT, isr_EncoderLeft, (void*)CHANNEL_A);
  gpio_isr_handler_add(GPIO_OUTB_LEFT, isr_EncoderLeft, (void*)CHANNEL_B);

  gpio_isr_handler_add(GPIO_OUTA_RIGHT,isr_EncoderRight, (void*)CHANNEL_A);
  gpio_isr_handler_add(GPIO_OUTB_RIGHT,isr_EncoderRight, (void*)CHANNEL_B);

  mcpwm_pin_config_t pin_configLeft = {
      .mcpwm0a_out_num = GPIO_PWM_LEFT,
      .mcpwm0b_out_num = GPIO_PWM_RIGHT,
  };
  mcpwm_set_pin(MCPWM_UNIT_0, &pin_configLeft);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = 10000;  //frequency = 10kHz
  pwm_config.cmpr_a = 0.0;       //duty cycle of PWMxA = 0.0%
  pwm_config.cmpr_b = 0.0;       //duty cycle of PWMxA = 0.0%
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
}
