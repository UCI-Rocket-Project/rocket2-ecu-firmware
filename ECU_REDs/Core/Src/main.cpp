#include "main.h"
#include "cpp_main.h"

extern SPI_HandleTypeDef hspi2;

void cpp_main(void) {
    HAL_GPIO_WritePin(REDS_MOSFET1_GPIO_Port, REDS_MOSFET1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(REDS_MOSFET2_GPIO_Port, REDS_MOSFET2_Pin, GPIO_PIN_SET);
    
    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
        HAL_Delay(500);
    }
}