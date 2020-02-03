#include <stdint.h>

/*************************************************************************************/
/********************************** COMANDOS  ****************************************/
/*************************************************************************************/
#define CMD_HEAD           0xA0

#define CMD_REQ_CAL        0x00
#define CMD_REQ_OMEGA      0x03

#define CMD_CALIBRATION    0x04
#define CMD_IDENTIFY       0x05
#define CMD_SET_KP         0x06

#define CMD_REF            0x0A
#define CMD_CONTROL_SIGNAL 0x0B

#define CMD_PING           0x0F


/*************************************************************************************/
/****************************** DEFINICOES DE TEMPOS *********************************/
/*************************************************************************************/
#define TIME_TEST_OMEGA_ZERO  300   //ms, tempo do timer que realiza o teste de velocidade zero
#define TIME_CONTROLLER         5   //ms, periodo de acionamento do controlador

/*************************************************************************************/
/****************************** ROTINAS PRINCIPAIS ***********************************/
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

/*************************************************************************************/
/****************************** MACROS, ESTRUCT E ENUM *******************************/
/*************************************************************************************/
//sinal de um float, 1 => negativo. 0 => positivo
#define F_IS_NEG(x) (*(uint32_t*)&(x) >> 31)
#define ABS_F(x) (((x)<0.0)?-(x):(x))
#define SATURADOR(x) ((ABS_F(x) > 1.0)?1.0:ABS_F(x))  //0.0 a 1.0
#define LS(x) (1ULL << (x))
#define DELAY_SEC(x) vTaskDelay(((x)*1000.0)/portTICK_PERIOD_MS)


enum ROTATE_S{
  FRONT,
  BACK
};
enum MOTOR{
  LEFT,
  RIGHT
};
struct CoefLine{
  float alpha; //coef. angular da reta
  float beta;  //coef. linear da reta
};
typedef struct{
  uint8_t *data;
  uint32_t len;
}bt_data;

struct Encoder_data{
  int64_t pulse_counter; //quantidade de pulsos desde o inicio da interrupcao
};
/*************************************************************************************/
/****************************** FUNCOES AUXILIARES ***********************************/
/*************************************************************************************/
void float2bytes(const float*f, uint8_t *bitstream, uint32_t num_float);
void bytes2float(const uint8_t *bitstream, float*f, uint32_t num_float);
void decodeFloat(const uint8_t *data, float *fa, float *fb);
double get_time_sec(void);
