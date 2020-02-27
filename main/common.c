#include "common.h"

#include <time.h>
#include <sys/time.h>
#include <string.h>

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

double get_time_sec()
{
  static struct timeval tbase={-1,-1};
  struct timeval t;

  gettimeofday(&t,NULL);
  if (tbase.tv_sec==-1 && tbase.tv_usec==-1)
    {
      tbase = t;
    }
  return ( t.tv_sec-tbase.tv_sec + (t.tv_usec-tbase.tv_usec)/1000000.0 );
}

esp_err_t save_parameters(void* ptr_parameters)
{
  nvs_handle my_handle;
  esp_err_t err;

  // Open
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) return err;

  // Write value including previously saved blob if available
  err = nvs_set_blob(my_handle, "param_page", ptr_parameters, sizeof(struct Parameters));

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
  size_t required_size = sizeof(struct Parameters);  // value will default to 0, if not set yet in NVS
  err = nvs_get_blob(my_handle, "param_page", ptr_parameters, &required_size);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

  // Close
  nvs_close(my_handle);
  return err;
}

// nvs_flash_erase_partition(STORAGE_NAMESPACE)
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
