#include "canbus.h"

bool writeEEPROM;
uint32_t writeTime;

void setup_CAN()
{
    Can0.begin(settings.canSpeed, 255); //no need for enable pin so set it to 255
    Can0.setNumTXBoxes(2);
    Can0.disable_tx_repeat();
    Can0.setRXFilter(0, 0x7F0, settings.canBaseRx, false);
    Can0.setRXFilter(1, 0x7C0, 0x100, false); //for firmware upgrades
    
}

void send32BitParam(int param, uint32_t valu)
{
    CAN_FRAME outFrame;

    outFrame.id = settings.canBaseTx + 0x10;
    outFrame.length = 5;
    outFrame.extended = false;
    outFrame.data.byte[0] = param & 0xFF;
    outFrame.data.byte[1] = valu & 0xFF;
    outFrame.data.byte[2] = (valu >> 8) & 0xFF;
    outFrame.data.byte[3] = (valu >> 16) & 0xFF;
    outFrame.data.byte[4] = (valu >> 24) & 0xFF;

    Can0.sendFrame(outFrame);
}

void send16BitParam(int param, uint16_t valu)
{
    CAN_FRAME outFrame;

    outFrame.id = settings.canBaseTx + 0x10;
    outFrame.length = 3;
    outFrame.extended = false;
    outFrame.data.byte[0] = param & 0xFF;
    outFrame.data.byte[1] = valu & 0xFF;
    outFrame.data.byte[2] = (valu >> 8) & 0xFF;

    Can0.sendFrame(outFrame);
}

void send8BitParam(int param, uint8_t valu)
{
    CAN_FRAME outFrame;

    outFrame.id = settings.canBaseTx + 0x10;
    outFrame.length = 2;
    outFrame.extended = false;
    outFrame.data.byte[0] = param & 0xFF;
    outFrame.data.byte[1] = valu;

    Can0.sendFrame(outFrame);
}


void processConfigMsg(CAN_FRAME& frame)
{
    uint8_t param = frame.data.byte[0] & 0x7F;
    bool isWriting = (frame.data.byte[0] & 0x80)?true:false;    
    uint32_t uint32Val = frame.data.byte[1] + (frame.data.byte[2] * 256ul) + (frame.data.byte[3] * 65536ul) + (frame.data.byte[4] * 256ul * 65536);;
    uint16_t uint16Val = frame.data.byte[1] + (frame.data.byte[2] * 256ul);
    uint8_t uint8Val = frame.data.byte[1];

    switch (param)
    {
    case 0: //Voltage Scale (4 Bytes)
        if (isWriting)
        {
            settings.busVoltageScale = uint32Val;
            writeEEPROM = true;
        }
        else send32BitParam(param, settings.busVoltageScale);
        break;
    case 1: //Voltage Bias (2 Bytes)
        if (isWriting)
        {
            settings.busVoltageBias = uint16Val;
            writeEEPROM = true;
        }
        else send16BitParam(param, settings.busVoltageBias);
        break;
    case 2: //Current 1 Scale (4 Bytes)
        if (isWriting)
        {
            settings.current1Scale = uint32Val;
            writeEEPROM = true;
        }
        else send32BitParam(param, settings.current1Scale);
        break;
    case 3: //Current 1 Bias (2 Bytes)
        if (isWriting)
        {
            settings.current1Bias = uint16Val;
            writeEEPROM = true;
        }
        else send16BitParam(param, settings.current1Bias);
        break;
    case 4: //Current 2 Scale (4 Bytes)
        if (isWriting)
        {
            settings.current2Scale = uint32Val;
            writeEEPROM = true;
        }
        else send32BitParam(param, settings.current2Scale);
        break;
    case 5: //Current 2 Bias (2 Bytes)
        if (isWriting)
        {
            settings.current2Bias = uint16Val;
            writeEEPROM = true;
        }
        else send16BitParam(param, settings.current2Bias);
        break;
    case 6: //Inverter Temp 1 Scale (4 Bytes)
        if (isWriting)
        {
            settings.inverterTemp1Scale = uint32Val;
            writeEEPROM = true;
        }
        else send32BitParam(param, settings.inverterTemp1Scale);
        break;
    case 7: //Inverter Temp 1 Bias (2 Bytes)
        if (isWriting)
        {
            settings.inverterTemp1Bias = uint16Val;
            writeEEPROM = true;
        }
        else send16BitParam(param, settings.inverterTemp1Bias);
        break;
    case 8: //Inverter Temp 2 Scale (4 Bytes)
        if (isWriting)
        {
            settings.inverterTemp2Scale = uint32Val;
            writeEEPROM = true;
        }
        else send32BitParam(param, settings.inverterTemp2Scale);
        break;
    case 9: //Inverter Temp 2 Bias (2 Bytes)
        if (isWriting)
        {
            settings.inverterTemp2Bias = uint16Val;
            writeEEPROM = true;
        }
        else send16BitParam(param, settings.inverterTemp2Bias);
        break;
    case 10: //Motor Temp 1 Scale (4 Bytes)
        if (isWriting)
        {
            settings.motorTemp1Scale = uint32Val;
            writeEEPROM = true;
        }
        else send32BitParam(param, settings.motorTemp1Scale);
        break;
    case 11: //Motor Temp 1 Bias (2 Bytes)
        if (isWriting)
        {
            settings.motorTemp1Bias = uint16Val;
            writeEEPROM = true;
        }
        else send16BitParam(param, settings.motorTemp1Bias);
        break;
    case 12: //Motor Temp 2 Scale (4 Bytes)
        if (isWriting)
        {
            settings.motorTemp2Scale = uint32Val;
            writeEEPROM = true;
        }
        else send32BitParam(param, settings.motorTemp2Scale);
        break;
    case 13: //Motor Temp 2 Bias (2 Bytes)
        if (isWriting)
        {
            settings.motorTemp2Bias = uint16Val;
            writeEEPROM = true;
        }
        else send16BitParam(param, settings.motorTemp2Bias);
        break;
    case 14: //Motor Type (1 Byte) 0 = Inductive, 1 = PMAC, 2 = BLDC
        if (isWriting)
        {
            settings.motorType = uint8Val;
            writeEEPROM = true;
        }
        else send8BitParam(param, settings.motorType);
        break;
    case 15: //Control Type (1 Byte) 0 = V/Hz, 1 = FOC
        if (isWriting)
        {
            settings.controlType = uint8Val;
            writeEEPROM = true;
        }
        else send8BitParam(param, settings.controlType);
        break;
    case 16: //Encoder Count (2 Bytes)
        if (isWriting)
        {
            settings.encoderCount = uint16Val;
            writeEEPROM = true;
        }
        else send16BitParam(param, settings.encoderCount);
        break;
    case 17: //Encoder Direction (1 Byte)
        if (isWriting)
        {
            settings.encoderDirection = uint8Val;
            writeEEPROM = true;
        }
        else send8BitParam(param, settings.encoderDirection);
        break;
    case 18: //Number of pole pairs (1 byte)
        if (isWriting)
        {
            settings.numPoles = uint8Val;
            writeEEPROM = true;
        }
        else send8BitParam(param, settings.numPoles);
        break;
    case 19: //Rotor Time Constant (4 bytes)
        if (isWriting)
        {
            settings.rotorTimeConst = uint32Val;
            writeEEPROM = true;
        }
        else send32BitParam(param, settings.rotorTimeConst);
        break;
    case 20: //PID KP (4 Bytes)
        if (isWriting)
        {
            settings.pid_KP = uint32Val;
            writeEEPROM = true;
        }
        else send32BitParam(param, settings.pid_KP);
        break;
    case 21: //PID KI (4 Bytes)
        if (isWriting)
        {
            settings.pid_KI = uint32Val;
            writeEEPROM = true;
        }
        else send32BitParam(param, settings.pid_KI);
        break;
    case 22: //PID KD (4 Bytes)
        if (isWriting)
        {
            settings.pid_KD = uint32Val;
            writeEEPROM = true;
        }
        else send32BitParam(param, settings.pid_KD);
        break;
    case 23: //Max RPM (2 Bytes)
        if (isWriting)
        {
            settings.maxRPM = uint16Val;
            writeEEPROM = true;
        }
        else send16BitParam(param, settings.maxRPM);
        break;
    case 24: //Max Torque (2 Bytes)
        if (isWriting)
        {
            settings.maxTorque = uint16Val;
            writeEEPROM = true;
        }
        else send16BitParam(param, settings.maxTorque);
        break;
    case 25: //Max Amps Drive (2 bytes)
        if (isWriting)
        {
            settings.maxAmpsDrive = uint16Val;
            writeEEPROM = true;
        }
        else send16BitParam(param, settings.maxAmpsDrive);
        break;
    case 26: //Max Amps Regen (2 bytes)
        if (isWriting)
        {
            settings.maxAmpsRegen = uint16Val;
            writeEEPROM = true;
        }
        else send16BitParam(param, settings.maxAmpsRegen);
        break;
    case 27: //VHz Full RPM (2 bytes)
        if (isWriting)
        {
            settings.vhzProperRPM = uint16Val;
            writeEEPROM = true;
        }
        else send16BitParam(param, settings.vhzProperRPM);
        break;
    case 28: //Theta Offset (2 bytes)
        if (isWriting)
        {
            settings.thetaOffset = uint16Val;
            writeEEPROM = true;
        }
        else send16BitParam(param, settings.thetaOffset);
        break;
    case 29: //Hall AB Sensor (1 byte)
        if (isWriting)
        {
            settings.hallAB = uint8Val;
            writeEEPROM = true;
        }
        else send8BitParam(param, settings.hallAB);
        break;
    case 30: //Hall BC Sensor (1 byte)
        if (isWriting)
        {
            settings.hallBC = uint8Val;
            writeEEPROM = true;
        }
        else send8BitParam(param, settings.hallBC);
        break;
    case 31: //Hall CA Sensor (1 byte)
        if (isWriting)
        {
            settings.hallCA = uint8Val;
            writeEEPROM = true;
        }
        else send8BitParam(param, settings.hallCA);
        break;
    }
    if (writeEEPROM)
    {
        writeTime = millis() + 1000;
        EEPROM.write(EEPROM_PAGE, settings);
    }
}

uint8_t calcCRC8(CAN_FRAME &frame)
{
    const uint8_t generator = 0xAD;
    uint8_t crc = 0;

    for (int byt = 0; byt < 6; byt++)
    {
        crc ^= frame.data.bytes[byt];

        for (int i = 0; i < 8; i++)
        {
            if ((crc & 0x80) != 0)
            {
                crc = (uint8_t)((crc << 1) ^ generator);
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void processPowerMsg(CAN_FRAME& frame)
{
    static int lastAliveByte;
    //SerialUSB.print("Got power frame ");
    if (calcCRC8(frame) == frame.data.bytes[6]) //validity byte is OK
    {
        if (frame.data.bytes[4] == 3) //drive
        {
            if (frame.data.byte[2] == 0 && frame.data.byte[3] == 0) //RPM control
            {
                int16_t targetRPM = frame.data.bytes[0] + (frame.data.bytes[1] * 256);
                //SerialUSB.print(targetRPM);
                //SerialUSB.println(" success!");
                //setVHzSpeed(targetRPM);
                if (targetRPM > 0) 
                    setDesiredTorque(targetRPM);
                else
                    setDesiredTorque(0);
            }
            else //torque control
            {
                int16_t targetTorque = frame.data.bytes[2] + (frame.data.bytes[3] * 256);
                setDesiredTorque(targetTorque);
            }
        }
        else if (frame.data.bytes[4] == 5); //reverse - not there yet
        else {
          setVHzSpeed(0);
          //SerialUSB.println(" null speed");
        }
        lastAliveByte = frame.data.bytes[6];
    }          
    else
    {
      //SerialUSB.println(" crc failure!");
    }    
}

void canRX()
{
    CAN_FRAME frame;
    
    if ((millis() > writeTime) && writeEEPROM)
    {
        writeEEPROM = false;
        EEPROM.write(EEPROM_PAGE, settings);
    }
    
    if (Can0.available())
    {
        Can0.read(frame);
        fwGotFrame(&frame);
        switch (frame.id - settings.canBaseRx)
        {
        case 0:
            processConfigMsg(frame);
            break;
        case 1:
            if (frame.data.byte[0] == 0x10 && frame.data.byte[1] == 0xDE) 
            {
                controllerStatus.rampingTest = true;
                controllerStatus.rampRPM = 4;
            }
            else controllerStatus.rampingTest = false;
            break;
        case 2:
            processPowerMsg(frame);
            break;            
        }
    }
}

