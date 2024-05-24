#include "mlx90614.h"

extern I2C_HandleTypeDef hi2c1; // I2C 핸들러

float MLX90614_ReadTemperature(void) {
  uint8_t data[3];
  HAL_I2C_Mem_Read(&hi2c1, MLX90614_I2C_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, data, 3, HAL_MAX_DELAY);

  int16_t rawTemperature = (data[1] << 8) | data[0];
  float temperature = rawTemperature * 0.02 - 273.15;

  return temperature;
}
