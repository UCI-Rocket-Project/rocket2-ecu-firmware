/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>

#include <cmath>

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "altimeter_ms5607_spi.h"
// #include "gps_ubxm8_i2c.h"
#include "imu_bmi088_spi.h"
#include "magnetometer_bmi150_i2c.h"
#include "memory_w25q1128jv_spi.h"
#include "tc_max31855_spi.h"

#include "radio_sx127x_spi.h"

#include "crc.h"

#define VREF 24.0             // reference voltage
#define VFACTOR (15+100)/15   // Factor for voltage divider  
#define ADC_MAX 4095.0        // 12-bit ADC resolution

using namespace std; 

#pragma(push, 1)
struct AdcData {
  uint32_t s0; 
  uint32_t s1; 
  uint32_t s2; 
  uint32_t s3; 
  uint32_t pt0; 
  uint32_t pt1; 
  uint32_t pt2; 
  uint32_t pt3; 
  uint32_t pt4; 
  uint32_t pt5; // extra unused
  uint32_t pt6; // extra unused
  uint32_t supplyVoltage; 
  uint32_t batteryVoltage; 
};

struct EcuCommand {
  bool alarm; 
  bool solenoidStatePv1;
  bool solenoidStatePv2;
  bool solenoidStateVent;
  bool solenoidStateCoPvVent;
  uint32_t crc; 
};

struct EcuData {
  uint32_t timestamp;
  float packetRssi;
  float packetLoss;
  bool solenoidInternalStateCopvVent;
  bool solenoidInternalStatePv1;
  bool solenoidInternalStatePv2;
  bool solenoidInternalStateVent;
  float supplyVoltage = std::nanf("");
  float batteryVoltage = std::nanf("");
  float solenoidCurrentCopvVent = std::nanf("");
  float solenoidCurrentPv1 = std::nanf("");
  float solenoidCurrentPv2 = std::nanf("");
  float solenoidCurrentVent = std::nanf("");
  float temperatureCopv = std::nanf("");
  float pressureCopv = std::nanf("");
  float pressureLox = std::nanf("");
  float pressureLng = std::nanf("");
  float pressureInjectorLox = std::nanf("");
  float pressureInjectorLng = std::nanf("");
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

struct ecuFluidSystemData {
  uint8_t type = 0x11;
  uint32_t timestamp = 0xFFFFFFFF;
  uint16_t supplyVoltage = 0xFFFF;
  uint16_t batteryVoltage = 0xFFFF;
  uint64_t : 4;
  bool solenoidStateCopvVent : 1;
  bool solenoidStatePv1 : 1;
  bool solenoidStatePv2 : 1;
  bool solenoidStateVent : 1;
  uint16_t solenoidCurrentCopvVent = 0xFFFF;
  uint16_t solenoidCurrentPv1 = 0xFFFF;
  uint16_t solenoidCurrentPv2 = 0xFFFF;
  uint16_t solenoidCurrentVent = 0xFFFF;
  int16_t temperatureCopv = 0xFFFF;
  uint16_t pressureCopv = 0xFFFF;
  uint16_t pressureLox = 0xFFFF;
  uint16_t pressureLng = 0xFFFF;
  uint16_t pressureInjectorLox = 0xFFFF;
  uint16_t pressureInjectorLng = 0xFFFF;
  uint64_t : 64;
  uint64_t : 64;
  uint64_t : 64;
  uint64_t : 64;
  uint16_t crc = 0x0000;
};

#pragma pack(pop)

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;
SPI_HandleTypeDef hspi6;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

// solenoid internal states, these are energized state, not open or closed
int solenoidState0;
int solenoidState1;
int solenoidState2;
int solenoidState3;

// TC handlers 
TcMax31855Spi tc0(&hspi3, TC0_nCS_GPIO_Port, TC0_nCS_Pin, 100);
TcMax31855Spi tc1(&hspi3, TC1_nCS_GPIO_Port, TC1_nCS_Pin, 100);

/* Flight sensor init */
MemoryW25q1128jvSpi memory1(&hspi1, MEM1_nCS_GPIO_Port, MEM1_nCS_Pin);
MemoryW25q1128jvSpi memory2(&hspi1, MEM2_nCS_GPIO_Port, MEM2_nCS_Pin);

AltimeterMs5607Spi altimeter(&hspi4, ALT_nCS_GPIO_Port, ALT_nCS_Pin, ALT_MISO_GPIO_Port, ALT_MISO_Pin, 1014.9, 100);

/////////////////////////////////////////// DO GPS LATER

ImuBmi088Spi imu(&hspi1, IMU_nCS1_GPIO_Port, IMU_nCS1_Pin, IMU_nCS2_GPIO_Port, IMU_nCS2_Pin);

MagBmi150i2c magnetometer(&hi2c1, MAG_INT_GPIO_Port, MAG_INT_Pin, MAG_DRDY_GPIO_Port, MAG_DRDY_Pin);

RadioSx127xSpi radio(&hspi5, RADIO_nCS_GPIO_Port, RADIO_nCS_Pin, RADIO_nRST_GPIO_Port, 
                      RADIO_nRST_Pin, 0xDA, RadioSx127xSpi::RfPort::PA_BOOST, 915000000, 15, 
                      RadioSx127xSpi::RampTime::RT40US, RadioSx127xSpi::Bandwidth::BW250KHZ, 
                      RadioSx127xSpi::CodingRate::CR45, RadioSx127xSpi::SpreadingFactor::SF7, 
                      8, true, 500, 1023);

RadioSx127xSpi radio1(&hspi5, RADIO1_nCS_GPIO_Port, RADIO1_nCS_Pin, RADIO1_nRST_GPIO_Port, 
                      RADIO1_nRST_Pin, 0xDA, RadioSx127xSpi::RfPort::PA_BOOST, 915000000, 15, 
                      RadioSx127xSpi::RampTime::RT40US, RadioSx127xSpi::Bandwidth::BW250KHZ, 
                      RadioSx127xSpi::CodingRate::CR45, RadioSx127xSpi::SpreadingFactor::SF7, 
                      8, true, 500, 1023);

RadioSx127xSpi radio2(&hspi5, RADIO2_nCS_GPIO_Port, RADIO2_nCS_Pin, RADIO2_nRST_GPIO_Port, 
                      RADIO2_nRST_Pin, 0xDA, RadioSx127xSpi::RfPort::PA_BOOST, 915000000, 15, 
                      RadioSx127xSpi::RampTime::RT40US, RadioSx127xSpi::Bandwidth::BW250KHZ, 
                      RadioSx127xSpi::CodingRate::CR45, RadioSx127xSpi::SpreadingFactor::SF7, 
                      8, true, 500, 1023);

// alarm- piezo buzzer 
int alarmState; 

// incoming command
bool newCommand = false; 
uint8_t commandBuffer[sizeof(EcuCommand)];
EcuCommand command; 

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_SPI6_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_SPI6_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();

  // Init data packages
  EcuData data;
  ecuFluidSystemData fluidSystemData;

  AltimeterMs5607Spi::Data altData;
  AltimeterMs5607Spi::State altState;

  //////////////////////////////////////  // GPS data package

  ImuBmi088Spi::Data imuData;

  MagBmi150i2c::Data magData;

  HAL_GPIO_WritePin(ETH_nRST_GPIO_Port, ETH_nRST_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(TC0_nCS_GPIO_Port, TC0_nCS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TC1_nCS_GPIO_Port, TC1_nCS_Pin, GPIO_PIN_SET);

  // TODO: Remove temp
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);

  solenoidState0 = 0;
  solenoidState1 = 0;
  solenoidState2 = 0;
  solenoidState3 = 0;

  alarmState = 0; 
  
  // ethernet recieve-- remove later once TRS receive is done 
  HAL_UART_Receive_IT(&huart3, commandBuffer, sizeof(EcuCommand));

  // radio initialisation
  if (!radio.Init())
    radio.Reset();
  if (!radio1.Init())
    radio1.Reset(); 
  if (!radio2.Init())
    radio2.Reset();

  // receive command from TRS
  radio.Receive((uint8_t *)&commandBuffer, sizeof(EcuCommand), (int *)&data.packetRssi); 
  radio1.Receive((uint8_t *)&commandBuffer, sizeof(EcuCommand), (int *)&data.packetRssi); 
  radio2.Receive((uint8_t *)&commandBuffer, sizeof(EcuCommand), (int *)&data.packetRssi); 

  // Array of radios for fallback
  RadioSx127xSpi* radios[] = {&radio, &radio1, &radio2};
  int currentRadioIndex = 0;

  // Init all of the sensors
  memory1.Init();
  memory2.Init();
  altimeter.Init();
  imu.Init();
  magnetometer.Init();

  HAL_Delay(100); // just like wait until all the sensors are all initialised

  /* Buffer variables */

  // Memory oscillates between writing between the 2 modules
  // Counter for memory write
  int memoryCounter = 0;

  uint8_t memoryBuffer[64];

  while (1){
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
	  HAL_Delay(1000);    
    /*Double check if this is correct*/
    uint32_t timestamp = HAL_GetTick(); // replace later with timers 
    data.timestamp = timestamp;

    // read the command from TRS
    // do one radio for now-- will write others later
    // plan is to basically check if the radio can receive and if it cant then switch radios 
    RadioSx127xSpi::State state = radios[currentRadioIndex]->Update(); // do one radio for now-- will write others later
    if (state == RadioSx127xSpi::State::RX_COMPLETE){
      // check CRC
      uint32_t crc = command.crc; 
      command.crc = 0; 
      if (crc == Crc32((uint8_t *)&command, sizeof(EcuCommand) - 4)){
        newCommand = true; 
        alarmState = command.alarm; 

      }
    }
    else if ((state == RadioSx127xSpi::State::IDLE) || 
             (state == RadioSx127xSpi::State::RX_TIMEOUT)){
      radios[currentRadioIndex]->Receive((uint8_t *)&commandBuffer, sizeof(EcuCommand), (int *)&data.packetRssi);
      // do nothing, just wait for the next command
    }
    else if (state == RadioSx127xSpi::State::ERROR){
      // radio error, reset radio and reinit
      radios[currentRadioIndex]->Reset();
      radios[currentRadioIndex]->Init();

      // switch to the next radio in the array
      currentRadioIndex = (currentRadioIndex + 1) % (sizeof(radios) / sizeof(radios[0]));

      // start receiving on the next radio
      radios[currentRadioIndex]->Receive((uint8_t *)&commandBuffer, sizeof(EcuCommand), (int *)&data.packetRssi);
    }

    // updating internal states
    if (newCommand){
      newCommand = false; 

      solenoidState0 = (int)(command.solenoidStateCoPvVent);
      solenoidState1 = (int)(command.solenoidStatePv1);
      solenoidState2 = (int)(command.solenoidStatePv2);
      solenoidState3 = (int)(command.solenoidStateVent);
    }

    // internal states feedback
    data.solenoidInternalStateCopvVent = (bool)solenoidState0;
    data.solenoidInternalStatePv1 = (bool)solenoidState1;
    data.solenoidInternalStatePv2 = (bool)solenoidState2;
    data.solenoidInternalStateVent = (bool)solenoidState3;

    // switch solenoids
    HAL_GPIO_WritePin(SOLENOID0_EN_GPIO_Port, SOLENOID0_EN_Pin, (GPIO_PinState)solenoidState0);
    HAL_GPIO_WritePin(SOLENOID1_EN_GPIO_Port, SOLENOID1_EN_Pin, (GPIO_PinState)solenoidState1);
    HAL_GPIO_WritePin(SOLENOID2_EN_GPIO_Port, SOLENOID2_EN_Pin, (GPIO_PinState)solenoidState2);
    HAL_GPIO_WritePin(SOLENOID3_EN_GPIO_Port, SOLENOID3_EN_Pin, (GPIO_PinState)solenoidState3);

    // alarm- piezo buzzer
    HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, (GPIO_PinState)alarmState);

    // ADC operations- sample averaging 
    AdcData rawData = {0};
    for (int i = 0; i < 16; i++) {
      HAL_ADC_Start(&hadc1);
      HAL_ADC_Start(&hadc2);
      HAL_ADC_Start(&hadc3); 

      HAL_ADC_PollForConversion(&hadc1, 10);
      HAL_ADC_PollForConversion(&hadc2, 10);
      HAL_ADC_PollForConversion(&hadc3, 10);

      uint32_t data1 = HAL_ADC_GetValue(&hadc1);
      *(((uint32_t *)&rawData) + i) += data1;
      uint32_t data2 = HAL_ADC_GetValue(&hadc2);
      *(((uint32_t *)&rawData) + i) += data2;
      uint32_t data3 = HAL_ADC_GetValue(&hadc3);
      *(((uint32_t *)&rawData) + i) += data3;
    }

    // battery voltages
    data.batteryVoltage = 0.062f * (float)rawData.batteryVoltage + 0.435f; 
    data.supplyVoltage = 0.062f * (float)rawData.supplyVoltage + 0.435f;  

    // pressure data 
    data.pressureCopv = 0.00128 * (float)rawData.pt0;
    data.pressureLng = 0.00128 * (float)rawData.pt1;
    data.pressureInjectorLng = 0.00128 * (float)rawData.pt2;
    data.pressureLox = 0.00128 * (float)rawData.pt3;
    data.pressureInjectorLox = 0.00128 * (float)rawData.pt4;
    (void)(0.00128f * (float)rawData.pt5); // extra PT
    (void)(0.00128f * (float)rawData.pt6); // extra PT

    // read TC
    TcMax31855Spi:: Data tcData; 
    tcData = tc0.Read(); 
    if (tcData.valid){
      data.temperatureCopv = tcData.tcTemperature;
    }
    tcData = tc1.Read(); 
    if (tcData.valid){
      data.temperatureCopv = tcData.tcTemperature;
    }

    // solenoid
    data.solenoidCurrentCopvVent = 0.000817f * (float)rawData.s0;
    data.solenoidCurrentPv1 = 0.000817f * (float)rawData.s1;
    data.solenoidCurrentPv2 = 0.000817f * (float)rawData.s2;
    data.solenoidCurrentVent = 0.000817f * (float)rawData.s3;

    // USB
    char buffer[1024] = {0};
    sprintf(buffer, 
            "Timestamp: %08X\r\n"
            "(1)CoPV Vent: %d-%04d   (2)PV1: %d-%04d  (3)PV2: %d-%04d  (4)Vent: %d-%04d\r\n"
            "PRESSURE\r\n"
            "CoPV Pressure: %04d  LNG Pressure: %04d  LNG INJ Pressure: %04d  LOX Pressure: %04d  LOX INJ Pressure: %04d\r\n"
            "TEMPERATURE\r\n"
            "CoPV Temperature: %03d"
            "BATTERY\r\n"
            "ECU Battery: %04d  Supply Voltage: %04d"
            "---------------------\r\n", 
            (unsigned int)(data.timestamp), 
            (int)(data.solenoidInternalStateCopvVent), (int)(data.solenoidCurrentCopvVent * 1000),
            (int)(data.solenoidInternalStatePv1), (int)(data.solenoidCurrentPv1 * 1000),
            (int)(data.solenoidInternalStatePv2), (int)(data.solenoidCurrentPv2 * 1000),
            (int)(data.solenoidCurrentVent), (int)(data.solenoidInternalStateVent * 1000),
            (int)(data.pressureCopv * 1000),
            (int)(data.pressureLng * 1000), (int)(data.pressureInjectorLng * 1000),
            (int)(data.pressureLox * 1000), (int)(data.pressureInjectorLox * 1000),
            (int)(data.temperatureCopv),
            (int)(data.batteryVoltage), (int)(data.supplyVoltage)
            );
    CDC_Transmit_FS((uint8_t *)buffer, strlen(buffer));

    // ethernet
    uint32_t crc = Crc32((uint8_t *)&data, sizeof(EcuData) - 4);
    data.crc = crc;
    // check if ethernet is connected
    if (HAL_GPIO_ReadPin(ETH_nRST_GPIO_Port, ETH_nRST_Pin) == GPIO_PIN_SET) {
      HAL_UART_Transmit(&huart3, (uint8_t *)&data, sizeof(EcuData), 100);
    }
    else {
      // radio transmit :)
    }

    
    /* Altimeter data */
    altState = altimeter.Read(AltimeterMs5607Spi::Rate::OSR4096);
    if (altState == AltimeterMs5607Spi::State::COMPLETE) {
        altData = altimeter.GetData();
        data.temperature = altData.temperature;
        data.altitude = altData.altitude;
    }

    /* GPS data */

    /* IMU data */
    imuData = imu.Read();
    data.angularVelocityX = -imuData.angularVelocityX;
    data.angularVelocityY = imuData.angularVelocityY;
    data.angularVelocityZ = -imuData.angularVelocityZ;
    data.accelerationX = -imuData.accelerationX;
    data.accelerationY = imuData.accelerationY;
    data.accelerationZ = -imuData.accelerationZ;

    /* Magnetometer data */
    magData = magnetometer.Read();
    data.magneticFieldX = magData.magneticFieldY;
    data.magneticFieldY = -magData.magneticFieldX;
    data.magneticFieldZ = magData.magneticFieldZ;
    
    /* Write to memory */
    memcpy(memoryBuffer, &data, sizeof(memoryBuffer));
    if(memoryCounter % 2 == 0)
    {
      if (memory1.ChipWrite(memoryBuffer) == MemoryW25q1128jvSpi::State::COMPLETE) {
        // reset the data from modules with lead time
        data.temperature = 0xFFFF;
        data.altitude = 0xFFFFFFFF;
        data.ecefPositionX = 0xFFFFFFFF;
        data.ecefPositionY = 0xFFFFFFFF;
        data.ecefPositionZ = 0xFFFFFFFF;
        data.ecefVelocityX = 0xFFFFFFFF;
        data.ecefVelocityY = 0xFFFFFFFF;
        data.ecefVelocityZ = 0xFFFFFFFF;
        data.ecefPositionAccuracy = 0xFFFFFFFF;
        data.ecefVelocityAccuracy = 0xFFFFFFFF;

        // increment memory counter
        memoryCounter++;
      }
    }
    else
    {
      if (memory2.ChipWrite(memoryBuffer) == MemoryW25q1128jvSpi::State::COMPLETE) {
        // reset the data from modules with lead time
        data.temperature = 0xFFFF;
        data.altitude = 0xFFFFFFFF;
        data.ecefPositionX = 0xFFFFFFFF;
        data.ecefPositionY = 0xFFFFFFFF;
        data.ecefPositionZ = 0xFFFFFFFF;
        data.ecefVelocityX = 0xFFFFFFFF;
        data.ecefVelocityY = 0xFFFFFFFF;
        data.ecefVelocityZ = 0xFFFFFFFF;
        data.ecefPositionAccuracy = 0xFFFFFFFF;
        data.ecefVelocityAccuracy = 0xFFFFFFFF;

        // increment memory counter
        memoryCounter++;
      }
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  uint32_t crc = Crc32(commandBuffer, sizeof(EcuCommand) - 4);
  if (crc == ((EcuCommand *)commandBuffer)->crc) {
      memcpy((uint8_t *)&command, commandBuffer, sizeof(EcuCommand));
      newCommand = true;
  }
  HAL_UART_Receive_IT(huart, commandBuffer, sizeof(EcuCommand));
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  /* SPI6 parameter configuration*/
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ALT_nCS_GPIO_Port, ALT_nCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_nRST_GPIO_Port, RADIO_nRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, RADIO_DIO1_Pin|RADIO_DIO0_Pin|RADIO1_DIO1_Pin|RADIO1_nCS_Pin
                          |RADIO1_nRST_Pin|RADIO_nCS_Pin|MEM1_nCS_Pin|SOLENOID0_EN_Pin
                          |SOLENOID1_EN_Pin|SOLENOID2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPS_INT_Pin|MEM2_nCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, SOLENOID3_EN_Pin|STATUS_LED_Pin|IMU_INT2_Pin|IMU_nCS2_Pin
                          |IMU_nCS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ETH_nRST_Pin|ETH_CP2_Pin|MAG_DRDY_Pin|MAG_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TC0_nCS_GPIO_Port, TC0_nCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TC1_nCS_Pin|IMU_INT4_Pin|IMU_INT3_Pin|IMU_INT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ALT_nCS_Pin */
  GPIO_InitStruct.Pin = ALT_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ALT_nCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RADIO2_nRST_Pin RADIO2_nCS_Pin */
  GPIO_InitStruct.Pin = RADIO2_nRST_Pin|RADIO2_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RADIO2_DIO1_Pin RADIO2_DIO0_Pin */
  GPIO_InitStruct.Pin = RADIO2_DIO1_Pin|RADIO2_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_nRST_Pin */
  GPIO_InitStruct.Pin = RADIO_nRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RADIO_nRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RADIO_DIO1_Pin RADIO_DIO0_Pin RADIO1_DIO1_Pin RADIO1_nCS_Pin
                           RADIO1_nRST_Pin RADIO_nCS_Pin MEM1_nCS_Pin SOLENOID0_EN_Pin
                           SOLENOID1_EN_Pin SOLENOID2_EN_Pin */
  GPIO_InitStruct.Pin = RADIO_DIO1_Pin|RADIO_DIO0_Pin|RADIO1_DIO1_Pin|RADIO1_nCS_Pin
                          |RADIO1_nRST_Pin|RADIO_nCS_Pin|MEM1_nCS_Pin|SOLENOID0_EN_Pin
                          |SOLENOID1_EN_Pin|SOLENOID2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS_INT_Pin MEM_nCS_Pin */
  GPIO_InitStruct.Pin = GPS_INT_Pin|MEM2_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : SOLENOID3_EN_Pin STATUS_LED_Pin IMU_INT2_Pin IMU_nCS2_Pin
                           IMU_nCS1_Pin */
  GPIO_InitStruct.Pin = SOLENOID3_EN_Pin|STATUS_LED_Pin|IMU_INT2_Pin|IMU_nCS2_Pin
                          |IMU_nCS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ETH_nRST_Pin ETH_CP2_Pin MAG_DRDY_Pin MAG_INT_Pin */
  GPIO_InitStruct.Pin = ETH_nRST_Pin|ETH_CP2_Pin|MAG_DRDY_Pin|MAG_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TC0_nCS_Pin */
  GPIO_InitStruct.Pin = TC0_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TC0_nCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TC1_nCS_Pin IMU_INT4_Pin IMU_INT3_Pin IMU_INT1_Pin */
  GPIO_InitStruct.Pin = TC1_nCS_Pin|IMU_INT4_Pin|IMU_INT3_Pin|IMU_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
