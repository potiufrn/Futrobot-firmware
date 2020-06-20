#include "common.h"
/****************************** VARIAVEIS GLOBAIS ************************************/
// typedef struct
// {
//   double  rawOmega;  //ultimo omega medido, sem filtro
//   double  omega;     //omega filtrado
//   double  pOmega;    //predicted omega
//   double  kGain;     //kalman gain
//   double  p;         //preditec variance, incerteza da estimativa
//   double  r;         //measure variance, incerteza da medição
// }encoder_data_t;
static encoder_data_t  enc_datas[2] = {{
    .rawOmega = 0.0,
    .omega = 0.0,
    .pOmega = 0.0,
    .kGain = 0.0,
    .p = 0.0,
    .r = 0.0},
    {
    .rawOmega = 0.0,
    .omega = 0.0,
    .pOmega = 0.0,
    .kGain = 0.0,
    .p = 0.0,
    .r = 0.0}};
static xQueueHandle bt_queue = NULL;
static xQueueHandle from_encoder_queue[2] = {NULL,NULL};
static xQueueHandle to_encoder_queue[2] = {NULL,NULL};
static bool bypass_controller = true;
static float  reference[2]    = {0.0, 0.0}; //-1.0 a 1.0
static memory_data_t mem;
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
static void func_identify(const uint8_t motor, const uint8_t controller,
                          const float setpoint, const float stopTime);
/****************************** ROTINAS PRINCIPAIS ***********************************/

void app_main()
{
  bt_data_t btdata;
  bt_queue   = xQueueCreate(1, sizeof(bt_data_t));
  from_encoder_queue[LEFT]  = xQueueCreate(1, sizeof(encoder_data_t));
  from_encoder_queue[RIGHT] = xQueueCreate(1, sizeof(encoder_data_t));

  to_encoder_queue[LEFT]  = xQueueCreate(1, sizeof(input_encoder_t));
  to_encoder_queue[RIGHT] = xQueueCreate(1, sizeof(input_encoder_t));

  /** configuracoes iniciais **/
  _setup();//interrupcoes no nucleo 0, dos encoders e do bluetooth
  //controlador no nucleo 1

  xTaskCreatePinnedToCore(periodic_controller, "periodic_controller", 2048, NULL, 4, NULL, 1);
  // xTaskCreatePinnedToCore(periodic_controller, "periodic_controller", 2048, NULL, 5, LEFT, 1);
  // xTaskCreatePinnedToCore(periodic_controller, "periodic_controller", 2048, NULL, 5, RIGHT, 1);

  uint8_t  head, cmd;
  while(1){
      xQueueReceive(bt_queue, &btdata, portMAX_DELAY);
      head = btdata.data[0] & 0xF0;
      if(head != CMD_HEAD)
        continue;
      cmd = btdata.data[0] & 0x0F;

      // Ações
      switch (cmd)
      {
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
        {
          uint8_t motor = btdata.data[1];
          uint8_t controller = btdata.data[2];
          float sp = *(float*)&btdata.data[3 + 0*sizeof(float)];
          float st = *(float*)&btdata.data[3 + 1*sizeof(float)];
          func_identify(motor, controller, sp, st);
          break;
        }
        case CMD_CALIBRATION:
          func_calibration();
          break;
        case CMD_REQ_OMEGA:
        {
          double omegas[2] = {enc_datas[LEFT].omega, enc_datas[RIGHT].omega};
          esp_spp_write(bt_handle, 2*sizeof(double), (uint8_t*)omegas);
          break;
        }
        case CMD_REQ_CAL:
          esp_spp_write(bt_handle, 2*sizeof(parameters_t), (uint8_t*)&mem.params);
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
  static const int8_t lookup_table[] = {0, 1, -1, 0, 0, 0, 0, 1, 0, 0, 0, -1, 0, 1, -1, 0};
  static const double k = 2.0*M_PI/3.0, q = 50.0;
  static input_encoder_t input = {0.0, 0.0}; //Wss, tau
  static uint8_t  enc_v = 0, ch = 0;
  static uint32_t prevTime[2] = {0.0, 0.0}, currentTime[2] = {0, 0};
  static double   dt = 0.0;
  static encoder_data_t mydata = {.rawOmega = 0.0,
                                  .omega = 0.0,
                                  .pOmega = 0.0,
                                  .kGain = 0.0,
                                  .p = 2000.0,
                                  .r = 2000.0/5.0};//raw, filtered, predicted, gain, p
  enc_v <<= 2;
  enc_v |= (REG_READ(GPIO_IN1_REG) >> (GPIO_OUTB_LEFT - 32)) & 0b0011;

  ch = ((uint32_t)arg)&0b1;
  currentTime[ch] = esp_timer_get_time();
  if(prevTime[ch] == 0)
  {
    prevTime[ch] = currentTime[ch];
    return;
  }
  dt = (currentTime[ch] - prevTime[ch])/1000000.0;
  xQueueReceiveFromISR(to_encoder_queue[LEFT], &input, 0);

  //medicao
  mydata.rawOmega  = k*(double)lookup_table[enc_v & 0b1111]/dt;
  //atualiza ganho do filtro
  mydata.kGain = mydata.p/(mydata.p + mydata.r);
  //predição de omega
  mydata.pOmega = mydata.omega + (input.wss - mydata.omega)*(1.0 - exp(-dt/input.tau));
  //omega filtrado
  mydata.omega = mydata.pOmega + mydata.kGain*(mydata.rawOmega - mydata.pOmega);
  //atualiza p
  mydata.p = (1.0 - mydata.kGain)*mydata.p + q;

  xQueueSendFromISR(from_encoder_queue[LEFT], &mydata, 0);
  prevTime[ch] = currentTime[ch];
}
//this function costs about ?us
static void IRAM_ATTR isr_EncoderRight(void* arg)
{
  static encoder_data_t mydata = {.rawOmega = 0.0,
                                  .omega = 0.0,
                                  .pOmega = 0.0,
                                  .kGain = 0.0,
                                  .p = 2000.0,
                                  .r = 2000.0/5.0};//raw, filtered, predicted, gain, p
  static const int8_t lookup_table[] = {0, -1, 1, 0, 0, 0, 0, -1, 0, 0, 0, 1, 0, -1, 1, 0};
  static const double k = 2.0*M_PI/3.0, q = 50.0;
  static input_encoder_t input = {0.0, 0.0}; //Wss, tau
  static uint8_t  enc_v = 0, ch = 0;
  static uint32_t prevTime[2] = {0.0, 0.0}, currentTime[2] = {0, 0}, reg_read;
  static double   dt = 0.0;

  enc_v <<= 2;
  reg_read = REG_READ(GPIO_IN_REG);
  enc_v |= (reg_read & LS(GPIO_OUTA_RIGHT)) >> (GPIO_OUTA_RIGHT-1);
  enc_v |= (reg_read & LS(GPIO_OUTB_RIGHT)) >> GPIO_OUTB_RIGHT;

  ch = ((uint32_t)arg)&0b1;
  currentTime[ch] = esp_timer_get_time();
  if(prevTime[ch] == 0)
  {
    prevTime[ch] = currentTime[ch];
    return;
  }
  dt = (currentTime[ch] - prevTime[ch])/1000000.0;
  xQueueReceiveFromISR(to_encoder_queue[RIGHT], &input, 0);

  //medicao
  mydata.rawOmega  = k*(double)lookup_table[enc_v & 0b1111]/dt;
  //atualiza ganho do filtro
  mydata.kGain = mydata.p/(mydata.p + mydata.r);
  //predição de omega
  mydata.pOmega = mydata.omega + (input.wss - mydata.omega)*(1.0 - exp(-dt/input.tau));
  //omega filtrado
  mydata.omega = mydata.pOmega + mydata.kGain*(mydata.rawOmega - mydata.pOmega);
  //atualiza p
  mydata.p = (1.0 - mydata.kGain)*mydata.p + q;

  xQueueSendFromISR(from_encoder_queue[RIGHT], &mydata, 0);
  prevTime[ch] = currentTime[ch];
}

static void
periodic_controller()
{
  input_encoder_t datas_to_enc[2] = {{0.0, 0.2}, {0.0, 0.2}};
  double erro[2];
  double omegaRef[2];
  float  pwm[2] = {0.0, 0.0};
  uint16_t countVelZero[2] = {1, 1}; //numero de contagem equivalente a 500ms no ciclo do controle
  uint8_t motor;

  while(1)
  {
    vTaskDelay(TIME_CONTROLLER/portTICK_PERIOD_MS);
    //Calculando os omegas atuais
    for(motor = 0; motor < 2; motor++)
    {
      if(xQueueReceive(from_encoder_queue[motor], &enc_datas[motor], 0) == 0) //buffer vazio
      {
        countVelZero[motor]++;
        if(countVelZero[motor] > TIME_TEST_OMEGA_ZERO/TIME_CONTROLLER)
        {
          enc_datas[motor].rawOmega = 0.0;
          enc_datas[motor].omega = 0;
        }
      }else{
        countVelZero[motor] = 1;
      }
    }
    #define ANG_COEF  mem.params[motor].coef[SENSE(reference[motor])].ang
    #define DEAD_ZONE mem.params[motor].coef[SENSE(reference[motor])].lin*(!!reference[motor])
    //Controlador
    for(motor = 0; motor < 2; motor++)
    {
      if(bypass_controller == false){
        omegaRef[motor] = reference[motor]*mem.omegaMax;
        // erro[motor]= omegaRef[motor] - enc_datas[motor].omega;    // rad/s [-omegaMaximo, omegaMaximo]
        erro[motor]= omegaRef[motor] - enc_datas[motor].rawOmega;    // rad/s [-omegaMaximo, omegaMaximo]
        //Ação proporcional
        pwm[motor] = erro[motor]*mem.params[motor].Kp[SENSE(reference[motor])];
        //Forward
        pwm[motor] += ANG_COEF * omegaRef[motor] + DEAD_ZONE;
        //saturator
        pwm[motor] = saturator(pwm[motor]);
      }else{
        pwm[motor] = saturator(reference[motor]);
      }
      datas_to_enc[motor].wss = pwm[motor]*mem.params[motor].K;
      datas_to_enc[motor].tau = mem.params[motor].tau;
      xQueueSend(to_encoder_queue[motor], &datas_to_enc[motor], 0);
    }
    func_controlSignal(pwm[LEFT], pwm[RIGHT]);
  }
}

static void
func_identify(const uint8_t motor, const uint8_t controller,
              const float setpoint, const float stopTime)
{
  memset(reference, 0, 2*sizeof(float));
  bypass_controller = !controller;

  uint16_t size = stopTime/(TIME_CONTROLLER/1000.0);
  export_data_t *out = malloc(sizeof(export_data_t) * size);

  // Coleta dados
  // reference[!motor] = setpoint;
  reference[motor] = setpoint;
  uint32_t startTime = esp_timer_get_time();
  for(int i = 0; i < size; i++)
  {
    out[i].dt = (esp_timer_get_time() - startTime)/1000000.0;
    out[i].encoder= enc_datas[motor];
    vTaskDelay(TIME_CONTROLLER/portTICK_PERIOD_MS);
  }
  memset(reference, 0, 2*sizeof(float));
  // send motor parameters
  esp_spp_write(bt_handle, sizeof(parameters_t), (void*)&mem.params[motor]);
  // send omegaMax
  esp_spp_write(bt_handle, sizeof(double), (void*)&mem.omegaMax);
  // send size
  esp_spp_write(bt_handle, sizeof(uint16_t), (void*)&size);
  // send sensor datas
  for(int i = 0; i < size; i++){
    vTaskDelay(10/portTICK_PERIOD_MS);
    esp_spp_write(bt_handle, sizeof(export_data_t), (void*)&out[i]);
  }
  free(out);
}
static void func_calibration()
{
 memset(reference, 0, 2*sizeof(float));
 bypass_controller = true;

 #define N_POINTS                20  //numero de pontos para regressao que estimara o ganho do sistema e os parametros do forward
 #define TIMEOUT                 50   //ms
 #define STEP_TIME                5   //ms
 #define N_IT                   (TIMEOUT/STEP_TIME)  //numero de pontos para regresso que estimara a constante de tempo

 #define TIME_MS_WAIT          1000
 #define MIN_PWM               0.15
 #define STEP                  (1.0 - MIN_PWM)/((float)N_POINTS)

 double minOmegaMax = 999999.9;
 double *x = NULL;
 double *y = NULL;
 double tau_tmp = 0.0;
 double K_tmp = 0.0;
 uint32_t prevTime = 0;

 memset(&mem, 0, sizeof(memory_data_t));
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
       reference[motor] = (sense == 0)?1.0 - STEP*i: -(1.0 - STEP*i);
       vTaskDelay(TIME_MS_WAIT/portTICK_PERIOD_MS);
       x[i] = enc_datas[motor].rawOmega;
       y[i] = reference[motor];
     }
     reference[motor] = 0.0;
     vTaskDelay(TIME_MS_WAIT/portTICK_PERIOD_MS);

     linearReg(x, y, N_POINTS,
                     &mem.params[motor].coef[sense].ang,
                     &mem.params[motor].coef[sense].lin);
     minOmegaMax = (fabs(x[0]) < minOmegaMax)?fabs(x[0]):minOmegaMax;

     ESP_LOGI("POTI_INFO", "Motor:%d Sense:%d a:%f b:%f",
              motor, sense, mem.params[motor].coef[sense].ang,
                            mem.params[motor].coef[sense].lin);
     /*Estimando a constante de tempo*/
     free(x);
     free(y);
     x = (double*)malloc(N_IT*sizeof(double));
     y = (double*)malloc(N_IT*sizeof(double));

     /*Regressao*/
     prevTime = esp_timer_get_time();
     reference[motor] = (sense == 0?1.0:-1.0);
     for(uint32_t i = 0; i < N_IT; i++)
     {
       x[i] = (esp_timer_get_time() - prevTime)/1000000.0;
       y[i] = enc_datas[motor].rawOmega;
       vTaskDelay(STEP_TIME/portTICK_PERIOD_MS);
     }
     K_tmp      = 1.0/mem.params[motor].coef[sense].ang;
     double wss = K_tmp*reference[motor];
     tau_tmp    = _calcTau(x, y, N_IT, wss);
     reference[motor] = 0.0;
     vTaskDelay(TIME_MS_WAIT/portTICK_PERIOD_MS);

     /*Calcula kp para o polo do sistema ficar em Sd*/
     mem.params[motor].Kp[sense] = -(Sd*tau_tmp + 1.0)/K_tmp;
     free(x);
     free(y);

     ESP_LOGI("POTI_INFO", "tau_reg:%lf K:%lf Kp:%lf", tau_tmp, K_tmp, mem.params[motor].Kp[sense]);
     mem.params[motor].K  += K_tmp/2.0;
     mem.params[motor].tau+= tau_tmp/2.0;
   }
 }
 memset(reference, 0, 2*sizeof(float));
 //Maior velocidade considerada
 mem.omegaMax = 0.90*fabs(minOmegaMax);
 esp_err_t err = save_parameters(&mem);
 if(err != ESP_OK)
  ESP_LOGI("POTI_ERRO","Falha ao salvar os parametros, erro:%s", esp_err_to_name(err));
  // ESP_LOGE("POTI_ERRO","Falha ao salvar os parametros, erro:%s", esp_err_to_name(err));
 ESP_LOGI("POTI_INFO","Calibracao finalizada");
}
//Funcao de tratamento de eventos do bluetooth
static void
esp_spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  bt_data_t btdata;
    switch (event){
    case ESP_SPP_INIT_EVT:
        // esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
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
        if((btdata.len == 1) && (btdata.data[0] == (CMD_HEAD | CMD_RESET)))
        {
          esp_restart();
        }
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
  esp_err_t err = load_parameters(&mem);
  if(err != ESP_OK)
    ESP_LOGI("POTI_ERRO","Falha ao carregar os dados da memória, erro:%s", esp_err_to_name(err));
    // ESP_LOGE("POTI_ERRO","Falha ao carregar os dados da memória, erro:%s", esp_err_to_name(err));

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

  //informa os parametros do modelo usado no filtro de kalman para a estrutura de dados nas interrupcoes
  for(int motor = 0; motor < 2; motor++)
  {
    input_encoder_t data;
    data.wss = 0.0; //falta passar o pwm pelo saturator
    data.tau = mem.params[motor].tau;
    xQueueSend(to_encoder_queue[motor], &data, 0);
  }

  //config. PWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM_LEFT);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM_RIGHT);
  //config. canais de PWM
  //PWM freq. 10Khz, up_counter, duty cyclo initial 0
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 10000;  //frequency = 10kHz
  pwm_config.cmpr_a = 0.0;       //duty cycle of PWMxA = 0.0%
  pwm_config.cmpr_b = 0.0;       //duty cycle of PWMxA = 0.0%
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
}
