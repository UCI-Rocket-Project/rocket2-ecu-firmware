#pragma once

#include <cstring>

#include "stm32h7xx_hal.h"
#include "ubx_messages.h"

class GnssUbloxM8Uart {
  public:
    enum class FixType {
        NO_FIX = 0,
        DEAD_RECKONING_ONLY = 1,
        FIX_2D = 2,
        FIX_3D = 3,
        GNSS_DEAD_RECKONING = 4,
        FIX_TIME_ONLY = 5
    };

    struct Data {
        FixType fixType;
        // valid date information
        bool validDate;
        // time of week
        uint32_t tow;
        // UTC year
        uint16_t year;
        // UTC month
        uint8_t month;
        // UTC day
        uint8_t day;
        // valid time information
        bool validTime;
        // UTC hour
        uint8_t hour;
        // UTC minute
        uint8_t minute;
        // UTC second
        uint8_t second;
        // UTC nanosecond
        int32_t nanosecond;
        // time accuracy (ns)
        uint32_t timeAccuracy;
        // valid location information
        bool validLocation;
        // longitude (1e-7 degrees)
        int32_t longitude;
        // latitude (1e-7 degrees)
        int32_t latitude;
        // height above ellipsoid (mm)
        int32_t height;
        // height above mean sea level (mm0)
        int32_t heightMSL;
        // horizontal accuracy (mm)
        uint32_t horizontalAccuracy;
        // vertical accuracy (mm)
        uint32_t verticalAccuracy;
        // north axis velocity (mm/s)
        int32_t velocityNorth;
        // east axis velocity (mm/s)
        int32_t velocityEast;
        // down axis velocity (mm/s)
        int32_t velocityDown;
        // velocity accuracy (mm/s)
        uint32_t velocityAccuracy;
    };

    /**
     * @param huart UART bus handler
     * @param serialTimeout serial bus timeout in milliseconds
     */
    GnssUbloxM8Uart(UART_HandleTypeDef *huart, unsigned int serialTimeout = 100);

    /**
     * @brief Resets GNSS module
     * @retval Operation status, true for success
     */
    bool Reset();

    /**
     * @brief Initializes GNSS module
     * @retval Operation status, true for success
     */
    bool Init();

    /**
     * @brief Polls for data
     * @param data struct to fill, valid when method returns true
     * @retval Operation status, true for success
     */
    bool Poll(Data &data);

    /**
     * @brief Place in `HAL_UART_RxCpltCallback` to handle DMA interrupt
     * @retval None
     */
    void DMACompleteCallback();

  private:
    UART_HandleTypeDef *_huart;
    unsigned int _serialTimeout;

    UBX_NAV_PVT _data;
    bool _polling = false;
    bool _newData = false;

    void ChangeBaud(int baud) {
        _huart->Instance->CR1 &= ~(USART_CR1_UE);
        _huart->Instance->BRR = (uint32_t)((HAL_RCC_GetPCLK2Freq() + baud / 2) / baud);
        _huart->Instance->CR1 |= USART_CR1_UE;
    }

    /**
     * @brief Takes raw bytes and outputs a UBX message struct with decoded data
     * @param packet struct to store decoded data in, use types defined in ubx_messages.h ONLY
     * @param buffer array of raw bytes, the amount of bytes accessed is dependent on the type of `packet`
     * @retval Data validity, true if passed CRC
     */
    template <typename T>
    bool DecodePacket(T &packet, const uint8_t *buffer) {
        // check for sync word
        if (buffer[0] != 0xB5) return false;
        if (buffer[1] != 0x62) return false;

        // check for matching class and ID
        if (buffer[2] != packet.packetClass) return false;
        if (buffer[3] != packet.packetId) return false;

        // check for matching length
        if (*(uint16_t *)&(buffer[4]) != packet.payloadLength) return false;

        // check CRC
        uint8_t checksumA = 0;
        uint8_t checksumB = 0;
        for (uint16_t i = 2; i < packet.payloadLength + 6; i++) {
            checksumA = checksumA + buffer[i];
            checksumB = checksumB + checksumA;
        }
        if (buffer[packet.payloadLength + 6] != checksumA) return false;
        if (buffer[packet.payloadLength + 7] != checksumB) return false;

        // fill struct
        std::memcpy(((uint8_t *)&packet) + 4, buffer + 6, packet.payloadLength);

        return true;
    }

    /**
     * @brief Takes a UBX message struct with data
     * @param buffer array to store encoded bytes in, the amount of bytes stored is dependent on the type of `packet`
     * @param packet struct containing data to encode, use types defined in ubx_messages.h ONLY
     * @retval None
     */
    template <typename T>
    void EncodePacket(uint8_t *payload, T packet) {
        payload[0] = 0xB5;
        payload[1] = 0x62;
        payload[2] = packet.packetClass;
        payload[3] = packet.packetId;
        payload[4] = (uint8_t)packet.payloadLength;
        payload[5] = (uint8_t)(packet.payloadLength >> 8);
        std::memcpy(payload + 6, ((uint8_t *)&packet) + 4, packet.payloadLength);

        uint8_t checksumA = 0;
        uint8_t checksumB = 0;
        for (uint16_t i = 2; i < packet.payloadLength + 6; i++) {
            checksumA = checksumA + payload[i];
            checksumB = checksumB + checksumA;
        }
        payload[packet.payloadLength + 6] = checksumA;
        payload[packet.payloadLength + 7] = checksumB;
    }

    template <typename T>
    bool SetCommand(T packet, bool checkAck) {
        uint8_t payload[packet.payloadLength + 8];
        EncodePacket(payload, packet);

        // clear potential overrun flags and leftover data
        __HAL_UART_CLEAR_OREFLAG(_huart);
        _huart->Instance->RDR;

        if (HAL_UART_Transmit(_huart, payload, packet.payloadLength + 8, _serialTimeout) != HAL_OK) return false;

        if (checkAck) {
            UBX_ACK_ACK ack;
            uint8_t buffer[ack.payloadLength + 8] = {0};
            if (HAL_UART_Receive(_huart, buffer, ack.payloadLength + 8, _serialTimeout) != HAL_OK) return false;
            if (!DecodePacket(ack, buffer)) return false;
            if (ack.clsID != packet.packetClass) return false;
            if (ack.msgID != packet.packetId) return false;
        }

        return true;
    }
};
