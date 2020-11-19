/*
* Especificações do ESP32, módulo: ESP-WROOM-32
*
* 2 Núcleos de 32bits + 1 ULP coprocessor (coprocessador de baixo consumo de energia)
* Freq.: até 250Mhz
* Performance: até 600 DMIPS
* ********* MEMÓRIA ********
* Flash Externa: 4MB (expansivel até 16MB)
* SRAM: 512Kbytes
* ROM: 448Kb (boot e funções essenciais)
* RTC slow SRAM: 8Kb (Para acesso do co-processador em modo deep-sleep.)
* RTC fast SRAM: 8Kb (Para armazenamento de dados e uso de CPU em boot de RTC (relógio de tempo-real) do modo deep-sleep.
*
* GPIO: 36
* PWM: 1/16 canais
* ADC: 12 bits
* Temperatura de operação: -40C, 125C
*/
#include <stdint.h>   //uintx_t
#include <string.h>  //memset, memcpy
#include <math.h>    //log, exp, PI...

/*MCPWM*/
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

/*GPIO*/
#include "driver/gpio.h"

//Bluetooth
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "esp_attr.h"

/*System and memory*/
#include <sys/time.h>   //esp_timer_get_time
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

//Debug, information
#include "esp_log.h" //ESP_LOGx (ESP_LOGI, ESP_LOGE, ESP_LOGW)

#define DEVICE_NAME "ESP_ROBO_TEST"
// #define DEVICE_NAME "ESP_ROBO_1"
// #define DEVICE_NAME "ESP_ROBO_2"
// #define DEVICE_NAME "ESP_ROBO_4"

/*************************************************************************************/
/********************************** COMANDOS  ****************************************/
/*************************************************************************************/
#define CMD_HEAD           0xA0

#define CMD_REQ_CAL        0x00
#define CMD_REQ_OMEGA      0x03

#define CMD_CALIBRATION    0x04
#define CMD_IDENTIFY       0x05

#define CMD_REF            0x0A
#define CMD_CONTROL_SIGNAL 0x0B

#define CMD_RESET          0x0E
#define CMD_PING           0x0F


/*************************************************************************************/
/****************************** DEFININDO TEMPOS *************************************/
/*************************************************************************************/
#define TIME_TEST_OMEGA_ZERO  500   //ms, tempo do timer que realiza o teste de velocidade zero
#define TIME_CONTROLLER         5   //ms, periodo de acionamento do controlador
#define Sd                  -10.0   //polo desejado

/*************************************************************************************/
/********************************** PIN MAP ******************************************/
/*************************************************************************************/
//Motor 1 (Esquerdo)
#define  GPIO_PWM_LEFT       23 //Controla a velocidade do motor A (Esquerdo)
#define  GPIO_A1N1_LEFT      21 //Sentido motor A
#define  GPIO_A1N2_LEFT      19 //Sentido motor A
#define  GPIO_OUTA_LEFT      33 //Sinal de saida do encoder Direito (Usado para calcular a velocidade)
#define  GPIO_OUTB_LEFT      32 //Sinal em quadrado com relacao ao CAP1A do motor 2 (usado para calcular o sentido de rotacao)
// Modo Standby do driver
#define  GPIO_STBY         18
//Motor 2 (Direito)
#define  GPIO_PWM_RIGHT    16  //  Controla a velocidade do motor B (Direito)
#define  GPIO_B1N1_RIGHT    5  // Sentido motor
#define  GPIO_B1N2_RIGHT   17  // Sentido motor B
#define  GPIO_OUTA_RIGHT   15  //Sinal de saida do encoder Esquerdo (usado para calcular a velocidade)
#define  GPIO_OUTB_RIGHT    4  //Sinal em quadrado com relacao ao CAP0A do motor 1 (usado para identificar o sentido de rotacao)

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
/****************************** MACROS, DATA ESTRUCT E ENUM *******************************/
/*************************************************************************************/
//sinal de um float, 1/True caso negativo. 0/False caso positivo
#define SENSE(x) ( (x) < 0.0 )
#define LS(x) (1ULL << (x))
#define DELAY_SEC(x) vTaskDelay(((x)*1000.0)/portTICK_PERIOD_MS)
//Padrao utilizado:
/*
*   Motor Esquerdo: 0
*   Motor Direito:  1
*   Sentido para frente: 0
*   Sentido para trás:   1
* Obs.: o sentido para frente/trás são os sentidos que favorecem o movimento para frene ou para trás do robo como
* um todo, ou seja, a referência é o robô e não o eixo do motor.
*
*   Relação entre motor/sentido e index no vetor de 4 elementos do tipo coef de reta (CoefLine).
*   Motor | Sense | index
*     0       0       0         (Motor esquerdo para frente)
*     0       1       1         (Motor esquerdo para trás)
*     1       0       2         (Motor direito  para frente)
*     1       1       3         (Motor direito  para trás)
*/
enum ROTATE_S{ FRONT, BACK};
enum MOTOR{ LEFT, RIGHT};

typedef struct
{
  double ang; //coef. angular da reta
  double lin;  //coef. linear da reta
}coefLine_t;

typedef struct
{
  uint8_t *data;
  uint32_t len;
}bt_data_t;

typedef struct
{
  double  rawOmega;  //ultimo omega medido, sem filtro
  double  omega;     //omega filtrado
  double  pOmega;    //predicted omega
  double  kGain;     //kalman gain
  double  p;         //preditec variance, incerteza da estimativa
  double  r;         //measure variance, incerteza da medição
  // double  exec_time; //tempo gasto na ultima execução
}encoder_data_t;

typedef struct{
  double wss;
  double tau;
}input_encoder_t;

typedef struct
{
  //Parâmetros natural do Sistema
  double K;      //Ganho do sistema
  double tau;    //Constante de tempo do sistema
  // Controlador Proporcional e Forward
  double Kp[2];  //Ganho do controlador proporcional, para cada sentido
  // Coef. Angular == 1/K
  // Coef. Linear  == Zona morta
  coefLine_t coef[2];// Coef. das retas Omega x PWM para cada sentido de rotação
}parameters_t;

typedef struct
{
  //Velocidade máxima do robô
  double omegaMax;
  // Parametros dos motores
  parameters_t params[2];
}memory_data_t;

typedef struct
{
  double dt;    //delta t desde considerando t0 como tempo onde que foi aplicado o input
  encoder_data_t encoder;
}export_data_t;
/*************************************************************************************/
/****************************** FUNCOES AUXILIARES ***********************************/
/*************************************************************************************/
void float2bytes(const float*f, uint8_t *bitstream, uint32_t num_float);
void bytes2float(const uint8_t *bitstream, float*f, uint32_t num_float);
void decodeFloat(const uint8_t *data, float *fa, float *fb);
void linearReg(double x[], double y[], uint32_t n, double *ang, double *lin);//algoritmo de regressao linear, utilizando MMQ
double _calcTau(double t[], double w[], uint32_t n, double Wss);//funcao auxiliar
float saturator(float x); //limita x a ficar no intervalo [-1.0 , +1.0]
void func_controlSignal(const float pwmL,const float pwmR);

#define STORAGE_NAMESPACE "storage"
//ref.: https://docs.espressif.com/projects/esp-idf/en/stable/api-reference/storage/nvs_flash.html
esp_err_t save_parameters(void* ptr_memory);
esp_err_t load_parameters(void* ptr_memory);
esp_err_t erase_parameters();
esp_err_t erase_all(); //apaga todos os pares de chave-valor  dentro do espaco de nomes: STORAGE_NAMESPACE