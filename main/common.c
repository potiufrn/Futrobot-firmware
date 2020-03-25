#include "common.h"

#include <time.h>
#include <sys/time.h>
#include <string.h>

/*************************************************************************************/
/****************************** DEFINIÇÕES DE FUNÇÕES ********************************/
/*************************************************************************************/

void bytes2float(const uint8_t *bitstream, float*f, uint32_t num_float)
{
  memcpy((float*)bitstream, f, num_float*sizeof(float));
}

void float2bytes(const float*f, uint8_t *bitstream, uint32_t num_float)
{
  memcpy((uint8_t*)f, bitstream, num_float*sizeof(float));
}

void decodeFloat(const uint8_t *data, float *fa, float *fb)
{
  uint16_t ref[2];
  uint8_t  sense[2]; // 0 -> back, 1 -> front

  sense[LEFT]  = (data[1] & 0x80) >> 7;
  sense[RIGHT] = (data[3] & 0x80) >> 7;
  ref[LEFT]    = ((data[1] << 8) | data[2]) & 0x7FFF;
  ref[RIGHT]   = ((data[3] << 8) | data[4]) & 0x7FFF;

  *fa  = (ref[LEFT]/32767.0)*(2.0*sense[LEFT]-1.0);
  *fb  = (ref[RIGHT]/32767.0)*(2.0*sense[RIGHT]-1.0);
}

double _calcTau(double t[], double w[], uint32_t n, double Wss)
{
  double Sxx = 0.0;
  double Sxy = 0.0;
  double y;

  for(uint32_t i = 0; i < n; i++)
  {
    if( (abs(w[i]) > 0.2*abs(Wss)) && (abs(w[i]) < 0.8*abs(Wss)) )
    {
      y = -log(1.0 - w[i]/Wss);
      Sxy+= t[i]*y;
      Sxx+= t[i]*t[i];
    }
  }

  return Sxx/Sxy;
}

void linearReg(double x[], double y[], uint32_t n, double *ang, double *lin)
{
  double Sx = 0.0, Sy = 0.0, Sxx = 0.0, Sxy = 0.0;
  double D;

  for(int i = 0; i < n; i++)
  {
    Sx += x[i];
    Sy += y[i];
    Sxx+= x[i]*x[i];
    Sxy+= x[i]*y[i];
  }
  D = n*Sxx - Sx*Sx;

  *ang = (n*Sxy - Sx*Sy)/D;
  *lin = (Sy*Sxx - Sx*Sxy)/D;
}

esp_err_t save_parameters(void* ptr_parameters)
{
  nvs_handle my_handle;
  esp_err_t err;

  // Open
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) return err;

  // Write value including previously saved blob if available
  err = nvs_set_blob(my_handle, "param_page", ptr_parameters, sizeof(parameters_t));

  if (err != ESP_OK) return err;

  // Commit
  err = nvs_commit(my_handle);
  if (err != ESP_OK) return err;

  // Close
  nvs_close(my_handle);
  return err;
}

esp_err_t load_parameters(void* ptr_parameters)
{
  nvs_handle my_handle;
  esp_err_t err;

  // Open
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) return err;

  // Read the size of memory space required for blob
  size_t required_size = sizeof(parameters_t);  // value will default to 0, if not set yet in NVS
  err = nvs_get_blob(my_handle, "param_page", ptr_parameters, &required_size);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

  // Close
  nvs_close(my_handle);
  return err;
}

esp_err_t erase_parameters()
{
  nvs_handle my_handle;
  esp_err_t err;

  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) return err;

  err = nvs_erase_key(my_handle, "param_page");

  nvs_close(my_handle);
  return err;
}

esp_err_t erase_all()
{
  nvs_handle my_handle;
  esp_err_t err;

  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) return err;

  err = nvs_erase_all(my_handle);

  nvs_close(my_handle);
  return err;
}

 void func_controlSignal(const float pwmL,const float pwmR)
{
  #define SAT(x) (((x) > 1.0)?1.0:(x))
   bool front[2]  = {false, false};
  front[LEFT]  = !F_IS_NEG(pwmL);
  front[RIGHT] = !F_IS_NEG(pwmR);

  REG_WRITE(GPIO_OUT_REG, (1 << GPIO_STBY) |
                          (!front[LEFT]  << GPIO_A1N1_LEFT)  | (front[LEFT]  << GPIO_A1N2_LEFT) |
                          (!front[RIGHT] << GPIO_B1N1_RIGHT)  | (front[RIGHT] << GPIO_B1N2_RIGHT));
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, SAT(ABS_F(pwmL))*100.0);  //set PWM motor esquerdo
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0B, SAT(ABS_F(pwmR))*100.0);//set PWM motor direito
}
