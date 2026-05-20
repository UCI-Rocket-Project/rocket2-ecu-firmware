#include "main.h"
#include "cpp_main.h"
#include "crc.h"

#include "altimeter_ms5607_spi.h"
#include "imu_bmi088_spi.h"
#include "gnss_ubloxM8_uart.h"
#include "magnetometer_bmm350_i2c.h"
#include "memory_w25n01gv_spi.h"

#include <stdint.h>
#include <stdbool.h>
#include <cmath>
#include <cstring>

#pragma pack(push, 1)
struct AdcData {
    uint32_t pt0;
    uint32_t pt1;
    uint32_t pt2;
    uint32_t pt3;
    uint32_t pt4;
    uint32_t s0;
    uint32_t s1;
    uint32_t s2;
    uint32_t s3;
};

struct EcuCommand {
    bool solenoidState0;
    bool solenoidState1;
    bool solenoidState2;
    bool solenoidState3;
    bool solenoidState4;
    uint32_t crc;
};

struct EcuData {
    uint32_t timestamp;
    float packetRssi;
    float packetLoss;

    // Command Feedback 
    bool solenoidInternalState0;
    bool solenoidInternalState1;
    bool solenoidInternalState2;
    bool solenoidInternalState3;
    bool solenoidInternalState4;

    // Solenoid Feedback
    bool solenoidFeedbackState0;
    bool solenoidFeedbackState1;
    bool solenoidFeedbackState2;
    bool solenoidFeedbackState3;
    bool solenoidFeedbackState4;

    float supplyVoltage = std::nanf("");
    float batteryVoltage = std::nanf("");
    float solenoidCurrent0 = std::nanf("");
    float solenoidCurrent1 = std::nanf("");
    float solenoidCurrent2 = std::nanf("");
    float solenoidCurrent3 = std::nanf(""); 
    float temperature0 = std::nanf("");
    float pressure0 = std::nanf("");
    float pressure1 = std::nanf("");
    float pressure2 = std::nanf("");    
    float pressure3 = std::nanf(""); 
    float pressure4 = std::nanf("");
    float angularVelocityX = std::nanf("");
    float angularVelocityY = std::nanf("");
    float angularVelocityZ = std::nanf("");
    float accelerationX = std::nanf("");
    float accelerationY = std::nanf("");
    float accelerationZ = std::nanf("");
    float magneticFieldX = std::nanf("");
    float magneticFieldY = std::nanf("");
    float magneticFieldZ = std::nanf("");
    float temperature = std::nanf("");
    float altitude = std::nanf("");
    float ecefPositionX = std::nanf("");
    float ecefPositionY = std::nanf("");
    float ecefPositionZ = std::nanf("");
    float ecefPositionAccuracy = std::nanf("");
    float ecefVelocityX = std::nanf("");
    float ecefVelocityY = std::nanf("");
    float ecefVelocityZ = std::nanf("");
    float ecefVelocityAccuracy = std::nanf("");
    uint32_t crc;
};
#pragma pack(pop)

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern UART_HandleTypeDef huart3; 
extern UART_HandleTypeDef huart4;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi6;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim5;

//  Flight sensors
AltimeterMs5607Spi altimeter(&hspi4, ALT_nCS_GPIO_Port, ALT_nCS_Pin, ALT_MISO_GPIO_Port, ALT_MISO_Pin,  1013.25, 100);
ImuBmi088Spi imu(&hspi6, IMU_nCS1_GPIO_Port, IMU_nCS1_Pin, IMU_nCS2_GPIO_Port, IMU_nCS2_Pin);
MagBmm350i2c magnetometer(&hi2c1, MAG_INT_GPIO_Port, MAG_INT_Pin, MAG_INT_GPIO_Port, MAG_INT_Pin);
GnssUbloxM8Uart gps(&huart4, 100);

// Global Instances
int solState[6] = {0};
bool newCommand = false;
uint8_t commandBuffer[sizeof(EcuCommand)];
EcuCommand command;

void init_ecu_data(EcuData &data);

void cpp_main(void){
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    HAL_Delay(500);

    HAL_GPIO_WritePin(ETH_nRST_GPIO_Port, ETH_nRST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10); // Hold in reset
    HAL_GPIO_WritePin(ETH_nRST_GPIO_Port, ETH_nRST_Pin, GPIO_PIN_SET);
    HAL_Delay(250); // Wait for XPort's internal 200 ms reset to finish 

    HAL_TIM_Base_Start(&htim5);
    
    altimeter.Reset();
    imu.Reset();
    magnetometer.Reset();
    HAL_Delay(100); 

    altimeter.Init();
    imu.Init();
    magnetometer.Init();
    HAL_UART_Receive_IT(&huart3, commandBuffer, sizeof(EcuCommand));

    // Start Ground Station Communication
    HAL_UART_Receive_IT(&huart3, commandBuffer, sizeof(EcuCommand));

    while(1) {
        EcuData data;
        init_ecu_data(data); // Fill with NaNs every loop iteration
        
        uint32_t timestamp = TIM5->CNT;
        data.timestamp = timestamp;

        // update internal states
        if (newCommand) {
            newCommand = false;
            solState[0] = (int)command.solenoidState0;
            solState[1] = (int)command.solenoidState1;
            solState[2] = (int)command.solenoidState2;
            solState[3] = (int)command.solenoidState3;
            solState[4] = (int)command.solenoidState4;
        }

        // internal states feedback
        data.solenoidInternalState0 = (bool)solState[0];
        data.solenoidInternalState1 = (bool)solState[1];
        data.solenoidInternalState2 = (bool)solState[2];
        data.solenoidInternalState3 = (bool)solState[3];
        data.solenoidInternalState4 = (bool)solState[4];

        // switch solenoids
        HAL_GPIO_WritePin(SOLENOID0_EN_GPIO_Port, SOLENOID0_EN_Pin, (GPIO_PinState)solState[0]);
        HAL_GPIO_WritePin(SOLENOID1_EN_GPIO_Port, SOLENOID1_EN_Pin, (GPIO_PinState)solState[1]);
        HAL_GPIO_WritePin(SOLENOID2_EN_GPIO_Port, SOLENOID2_EN_Pin, (GPIO_PinState)solState[2]);
        HAL_GPIO_WritePin(SOLENOID3_EN_GPIO_Port, SOLENOID3_EN_Pin, (GPIO_PinState)solState[3]);
        HAL_GPIO_WritePin(SOLENOID4_EN_GPIO_Port, SOLENOID4_EN_Pin, (GPIO_PinState)solState[4]);

        // ADC operations
        AdcData adcData = {0};

        // Read PTS
        uint32_t *pt_ptr = &adcData.pt0;
        for (int i = 0; i < 6; i++) {
            HAL_ADC_Start(&hadc3);
            HAL_ADC_PollForConversion(&hadc3, 100);
            uint32_t data = HAL_ADC_GetValue(&hadc3);
            *(pt_ptr + i) = data;
        }

        // Read solenoids
        // Note: loop ends at 4th solenoid because the pin PB2 cannot currently be configured on ADCs
        uint32_t *sol_ptr = &adcData.s0;
        for (int i = 0; i < 4; i++) {
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 100);
            uint32_t data = HAL_ADC_GetValue(&hadc1);
            *(sol_ptr + i) = data;
        }

        data.pressure0 = (float)adcData.pt0;
        data.pressure1 = (float)adcData.pt1;
        data.pressure2 = (float)adcData.pt2;
        data.pressure3 = (float)adcData.pt3;
        data.pressure4 = (float)adcData.pt4;

        data.solenoidFeedbackState0 = (adcData.s0 > 500);
        data.solenoidFeedbackState1 = (adcData.s1 > 500);
        data.solenoidFeedbackState2 = (adcData.s2 > 500);
        data.solenoidFeedbackState3 = (adcData.s3 > 500);
        
        if (altimeter.Read(AltimeterMs5607Spi::Rate::OSR4096) == AltimeterMs5607Spi::State::COMPLETE) {
            auto altData = altimeter.GetData();
            data.temperature = altData.temperature;
            data.altitude = altData.altitude;
        }

        auto imuData = imu.Read();
        data.accelerationX = imuData.accelerationX;
        data.accelerationY = imuData.accelerationY;
        data.accelerationZ = imuData.accelerationZ;

        auto magData = magnetometer.Read();
        data.magneticFieldX = magData.magneticFieldX;
        data.magneticFieldY = magData.magneticFieldY;
        data.magneticFieldZ = magData.magneticFieldZ;

        // GPS data
        //bool gpsSuccess = gps.Poll(gpsData);

        //if (gpsSuccess) HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
        
        // ethernet
        uint32_t crc = Crc32((uint8_t *)&data, sizeof(EcuData) - 4);
        data.crc = crc;
        HAL_UART_Transmit(&huart3, (uint8_t *)&data, sizeof(EcuData), 100);

        // loop time control (the timestamp rolls over after 49 hours, should be ok)
        while (TIM5->CNT << 16) {
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    uint32_t crc = Crc32(commandBuffer, sizeof(EcuCommand) - 4);
    if (crc == ((EcuCommand *)commandBuffer)->crc) {
        memcpy((uint8_t *)&command, commandBuffer, sizeof(EcuCommand));
        newCommand = true;
        }
    HAL_UART_Receive_IT(huart, commandBuffer, sizeof(EcuCommand));
}

void init_ecu_data(EcuData &data) {
    data.packetRssi = std::nanf("");
    data.packetLoss = std::nanf("");
    data.angularVelocityX = std::nanf("");
    data.angularVelocityY = std::nanf("");
    data.angularVelocityZ = std::nanf("");
    data.accelerationX = std::nanf("");
    data.accelerationY = std::nanf("");
    data.accelerationZ = std::nanf("");
    data.magneticFieldX = std::nanf("");
    data.magneticFieldY = std::nanf("");
    data.magneticFieldZ = std::nanf("");
    data.temperature = std::nanf("");
    data.altitude = std::nanf("");
    data.ecefPositionX = std::nanf("");
    data.ecefPositionY = std::nanf("");
    data.ecefPositionZ = std::nanf("");
    data.ecefPositionAccuracy = std::nanf("");
    data.ecefVelocityX = std::nanf("");
    data.ecefVelocityY = std::nanf("");
    data.ecefVelocityZ = std::nanf("");
    data.ecefVelocityAccuracy = std::nanf("");
}


