#include "gnss_ubloxM8_uart.h"

GnssUbloxM8Uart::GnssUbloxM8Uart(UART_HandleTypeDef *huart, unsigned int serialTimeout) : _huart(huart), _serialTimeout(serialTimeout) {}

bool GnssUbloxM8Uart::Reset() {
    /*UBX_CFG_RST packet;
    packet.navBbrMask = 0x0000;
    packet.resetMode = 0x01;
    uint8_t payload[packet.payloadLength + 8];
    EncodePacket(payload, packet);

    bool success = true;

    // try possible baud rates
    ChangeBaud(9600);
    if (HAL_UART_Transmit(_huart, payload, packet.payloadLength + 8, _serialTimeout) != HAL_OK) {
        success = false;
    }
    
    ChangeBaud(460800);
    if (HAL_UART_Transmit(_huart, payload, packet.payloadLength + 8, _serialTimeout) != HAL_OK) {
        success = false;
    }

    return success;*/

    // GPS Raw Exp.
    /*UBX_NAV_RESETODO packet;
    // UBX_CFG_CFG packet;
    // packet.clearMask = 0x1F1F; // Clear all
    // packet.saveMask = 0x0;
    // packet.loadMask = 0x0;
    uint8_t payload[packet.payloadLength + 8];
    
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

    // CHANGE BAUD WOULD HAPPEN HERE BUT I DONT TRUST IT RN LMAO
    HAL_UART_Receive_DMA(_huart, gnssBuffer, 10);
    // sizeof(gnssBuffer) / sizeof(uint8_t)
    _polling = true;
    HAL_StatusTypeDef stat = HAL_UART_Transmit_DMA(_huart, payload, packet.payloadLength + 8);*/
        
    uint8_t configUBX[]={
        0xB5,0x62, // Sync
        0x06,0x00, // Packet
        0x14,0x00, // Length
  
        // Payload
        0x01, // portID: 1
        0x00, // RESERVED
        0x00,0x00, // txReady: 0000 0000 0000 0000 | DISABLED
        0xD0,0x08,0x00,0x00, // mode: 0000 0000 0000 0000 0000 1000 1101 0000 | 8 bit char len, no parity, 1 stop bit
        0x80,0x25,0x00,0x00, // baud rate: 0000 0000 0000 0000 0010 0101 1000 0000 | 9600
        0x01,0x00, // inProtoMask: 0000 0000 0000 0001 | UBX enabled, disable all other protocols for input to module
        0x01,0x00, // outProtoMask: 0000 0000 0000 0001 | UBX enabled, disable all other protocols for output from module
        0x00,0x00, // flags: 0000 0000 0000 0000 | non-extended TX timeout
        0x00,0x00, // RESERVED
  
        0x9A,0x79 // Checksum
    };
    
    HAL_StatusTypeDef state1 = HAL_UART_Transmit_DMA(_huart, configUBX, sizeof(configUBX) / sizeof(uint8_t));
    HAL_Delay(1250);
  
    uint8_t readPortConfig[]={
        0xB5,0x62, // Sync
        0x06,0x00, // Packet
        0x01,0x00, // Length
  
        // Payload
        0x01, // portID: 1
  
        0x08,0x22 // Checksum
    };
    HAL_StatusTypeDef state2 = HAL_UART_Transmit_DMA(_huart, readPortConfig, sizeof(readPortConfig) / sizeof(uint8_t));
  
    // HAL_StatusTypeDef state3 = HAL_UART_Receive_DMA(_huart, gnssBuffer, 26);
    // HAL_Delay(1250);

    
    return true;
}

bool GnssUbloxM8Uart::Init() {
    ChangeBaud(9600);

    // disable all NMEA message output, set 460800 baud
    UBX_CFG_PRT portConfig;
    portConfig.portID = 1;
    portConfig.txReady = 0x0000;
    portConfig.mode = 0x000008C0;
    portConfig.baudRate = 460800;
    portConfig.inProtoMask = 0x0007;
    portConfig.outProtoMask = 0x0001;
    portConfig.flags = 0x0000;
    bool psuc = SetCommand(portConfig, false);

    ChangeBaud(460800);
/*    
    if (!CheckAck(portConfig)) {

        return false;
    }*/

    HAL_Delay(1000);

    // configure navigation engine: airborne 4g dynamic model, 3D fix only
    UBX_CFG_NAV5 navConfig;
    navConfig.mask = 0x0005;
    navConfig.dynModel = 8;
    navConfig.fixMode = 2;
    navConfig.fixedAlt = 0;
    navConfig.fixedAltVar = 10000;
    bool nsuc = SetCommand(navConfig, true);
    nsuc = CheckAck(navConfig);

    HAL_Delay(1000);

    // take measurement every 50ms, produce solution every measurement, use GPS time reference
    UBX_CFG_RATE rateConfig;
    rateConfig.measRate = 50;
    rateConfig.navRate = 1;
    rateConfig.timeRef = 1;
    bool rsuc = SetCommand(rateConfig, true);
    rsuc = CheckAck(rateConfig);

    HAL_Delay(1000);

    // output NAV-PVT solution as frequently as possible
    UBX_CFG_MSG msgConfig;
    msgConfig.msgClass = 0x01;
    msgConfig.msgID = 0x07;
    msgConfig.rate = 1;
    bool msuc = SetCommand(msgConfig, true);
    msuc = CheckAck(msgConfig);



    return psuc && nsuc && rsuc && msuc;
}

bool GnssUbloxM8Uart::Poll(Data &data) {
    if (!_polling) {
        /*__HAL_UART_CLEAR_OREFLAG(_huart);
        _huart->Instance->DR;*/
        HAL_UART_Receive_DMA(_huart, gnssBuffer, sizeof(gnssBuffer));
        _polling = true;
    }

    if (_newData) {
        data.fixType = (FixType)_data.fixType;
        data.validDate = _data.valid & 0x01;
        if (data.validDate) {
            data.tow = _data.iTOW;
            data.year = _data.year;
            data.month = _data.month;
            data.day = _data.day;
        }
        data.validTime = _data.valid & 0x02;
        if (data.validTime) {
            data.hour = _data.hour;
            data.minute = _data.min;
            data.second = _data.sec;
            data.nanosecond = _data.nano;
            data.timeAccuracy = _data.tAcc;
        }
        data.validLocation = _data.flags & 0x01;
        if (data.validLocation) {
            data.longitude = _data.lon;
            data.latitude = _data.lat;
            data.height = _data.height;
            data.heightMSL = _data.hMSL;
            data.horizontalAccuracy = _data.hAcc;
            data.verticalAccuracy = _data.vAcc;
            data.velocityNorth = _data.velN;
            data.velocityEast = _data.velE;
            data.velocityDown = _data.velD;
            data.velocityAccuracy = _data.sAcc;
        }
        return true;
    }
    return false;
}

void GnssUbloxM8Uart::DMACompleteCallback() {
    _polling = false;
    _newData = DecodePacket(_data, gnssBuffer);
}
