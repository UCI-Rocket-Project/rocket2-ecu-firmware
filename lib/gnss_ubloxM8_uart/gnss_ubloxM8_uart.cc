#include "gnss_ubloxM8_uart.h"

GnssUbloxM8Uart::GnssUbloxM8Uart(UART_HandleTypeDef *huart, unsigned int serialTimeout) : _huart(huart), _serialTimeout(serialTimeout) {}

bool GnssUbloxM8Uart::Reset() {
    UBX_CFG_RST packet;
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

    return success; 
}

bool GnssUbloxM8Uart::Init() {
    ChangeBaud(9600);

    // disable all NMEA message output, set 460800 baud
    bool psuc = true;
    UBX_CFG_PRT portConfig;
    portConfig.portID = 1;
    portConfig.txReady = 0x0000;
    portConfig.mode = 0x000008C0;
    portConfig.baudRate = 460800;
    portConfig.inProtoMask = 0x0007;
    portConfig.outProtoMask = 0x0001;
    portConfig.flags = 0x0000;
    if (!SetCommand(portConfig, false)) {
        psuc = false;
    }

    ChangeBaud(460800);
/*    
    if (!CheckAck(portConfig)) {

        return false;
    }*/

    HAL_Delay(1000);

    // configure navigation engine: airborne 4g dynamic model, 3D fix only
    bool nsuc = true;
    UBX_CFG_NAV5 navConfig;
    navConfig.mask = 0x0005;
    navConfig.dynModel = 8;
    navConfig.fixMode = 2;
    navConfig.fixedAlt = 0;
    navConfig.fixedAltVar = 10000;
    if (!SetCommand(navConfig, true)) {
        nsuc = false;
    }

    HAL_Delay(1000);

    // take measurement every 50ms, produce solution every measurement, use GPS time reference
    bool rsuc = true;
    UBX_CFG_RATE rateConfig;
    rateConfig.measRate = 50;
    rateConfig.navRate = 1;
    rateConfig.timeRef = 1;
    if (!SetCommand(rateConfig, true)) {
        rsuc = false;
    }

    HAL_Delay(1000);

    // output NAV-PVT solution as frequently as possible
    bool msuc = true;
    UBX_CFG_MSG msgConfig;
    msgConfig.msgClass = 0x01;
    msgConfig.msgID = 0x07;
    msgConfig.rate = 1;
    if (!SetCommand(msgConfig, true)) {
        msuc = false;
    }



    return psuc && nsuc && rsuc && msuc;
}

bool GnssUbloxM8Uart::Poll(Data &data) {
    if (!_polling) {
        __HAL_UART_CLEAR_OREFLAG(_huart);
        _huart->Instance->DR;
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
