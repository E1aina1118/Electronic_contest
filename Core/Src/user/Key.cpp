#include "main.h"

uint8_t getButtomState(uint8_t buttomNum)
{
  if (buttomNum == 1)
  {
    if (HAL_GPIO_ReadPin(Buttom1_GPIO_Port, Buttom1_Pin) == GPIO_PIN_RESET)
    {
      HAL_Delay(20);
      while (HAL_GPIO_ReadPin(Buttom1_GPIO_Port, Buttom1_Pin) == GPIO_PIN_RESET)
      {
      }
      return 1;
    }
    return 0;
  }
  else if (buttomNum == 2)
  {
    if (HAL_GPIO_ReadPin(Buttom2_GPIO_Port, Buttom2_Pin) == GPIO_PIN_RESET)
    {
      HAL_Delay(20);
      while (HAL_GPIO_ReadPin(Buttom2_GPIO_Port, Buttom2_Pin) == GPIO_PIN_RESET)
      {
      }
      return 1;
    }
    return 0;
  }
  else if (buttomNum == 3)
  {
    if (HAL_GPIO_ReadPin(Buttom3_GPIO_Port, Buttom3_Pin) == GPIO_PIN_RESET)
    {
      HAL_Delay(20);
      while (HAL_GPIO_ReadPin(Buttom3_GPIO_Port, Buttom3_Pin) == GPIO_PIN_RESET)
      {
      }
      return 1;
    }
    return 0;
  }
  return 0;
}
