#include "main.h"
#include "cpp_main.h"

void cpp_main(void) {

    while (1) {
        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
        HAL_Delay(500);
    }
}
