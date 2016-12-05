#include <i2c_t3.h>
#include <MS_5803.h>

i2c_t3& getWire(bool wire) {
    return wire ? Wire : Wire1;
}

MS_5803::MS_5803(uint16_t Resolution, bool w, int a) {
    _Resolution = Resolution;
    wire = w;
    addr = a;
}

//-------------------------------------------------
boolean MS_5803::initializeMS_5803(boolean Verbose) {
    resetSensor();
    if (Verbose) {
        // Display the oversampling resolution or an error message
        if (_Resolution == 256 | _Resolution == 512 | _Resolution == 1024 | _Resolution == 2048 | _Resolution == 4096){
            Serial.print("Oversampling setting: ");
            Serial.println(_Resolution);
        } else {
            Serial.println("*******************************************");
            Serial.println("Error: specify a valid oversampling value");
            Serial.println("Choices are 256, 512, 1024, 2048, or 4096");
            Serial.println("*******************************************");
        }
        
    }
    for (int i = 0; i < 8; i++ ){
        getWire(wire).beginTransmission(addr);
        getWire(wire).write(0xA0 + (i * 2));
        getWire(wire).endTransmission();
        getWire(wire).requestFrom(addr, 2);
        while(getWire(wire).available()) {
            HighByte = getWire(wire).read();
            LowByte = getWire(wire).read();
        }
        sensorCoeffs[i] = (((unsigned int)HighByte << 8) + LowByte);
        if (Verbose){
            Serial.print("C");
            Serial.print(i);
            Serial.print(" = ");
            Serial.println(sensorCoeffs[i]);
            delay(10);
        }
    }
    unsigned char p_crc = sensorCoeffs[7];
    unsigned char n_crc = MS_5803_CRC(sensorCoeffs);
    
    if (Verbose) {
        Serial.print("p_crc: ");
        Serial.println(p_crc);
        Serial.print("n_crc: ");
        Serial.println(n_crc);
    }
    if (p_crc != n_crc) {
        return false;
    }
    return true;
}

//------------------------------------------------------------------
void MS_5803::readSensor() {
    if (_Resolution == 256){
        D1 = MS_5803_ADC(CMD_ADC_D1 + CMD_ADC_256); // read raw pressure
        D2 = MS_5803_ADC(CMD_ADC_D2 + CMD_ADC_256); // read raw temperature
    } else if (_Resolution == 512) {
        D1 = MS_5803_ADC(CMD_ADC_D1 + CMD_ADC_512); // read raw pressure
        D2 = MS_5803_ADC(CMD_ADC_D2 + CMD_ADC_512); // read raw temperature
    } else if (_Resolution == 1024) {
        D1 = MS_5803_ADC(CMD_ADC_D1 + CMD_ADC_1024); // read raw pressure
        D2 = MS_5803_ADC(CMD_ADC_D2 + CMD_ADC_1024); // read raw temperature
    } else if (_Resolution == 2048) {
        D1 = MS_5803_ADC(CMD_ADC_D1 + CMD_ADC_2048); // read raw pressure
        D2 = MS_5803_ADC(CMD_ADC_D2 + CMD_ADC_2048); // read raw temperature
    } else if (_Resolution == 4096) {
        D1 = MS_5803_ADC(CMD_ADC_D1 + CMD_ADC_4096); // read raw pressure
        D2 = MS_5803_ADC(CMD_ADC_D2 + CMD_ADC_4096); // read raw temperature
    }
    dT = (int32_t)D2 - ( (int32_t)sensorCoeffs[5] * 256 );
    TEMP = 2000 + ((int64_t)dT * sensorCoeffs[6]) / 8388608LL;
    TEMP = (int32_t)TEMP;
    if (TEMP < 2000) {
        T2 = ((int64_t)dT * dT) / 2147483648ULL ; // 2^31 = 2147483648
        T2 = (int32_t)T2; // recast as signed 32bit integer
        OFF2 = 3 * ((TEMP-2000) * (TEMP-2000));
        Sens2 = 7 * ((TEMP-2000)*(TEMP-2000)) / 8;
    } else {
        T2 = 0;
        OFF2 = 0;
        Sens2 = 0;
        if (TEMP > 4500) {
            Sens2 = Sens2 - ((TEMP-4500)*(TEMP-4500)) / 8;
        }
    }
    if (TEMP < -1500) {
        Sens2 = Sens2 + 2 * ((TEMP+1500)*(TEMP+1500));
    }
    Offset = (int64_t)sensorCoeffs[2] * 65536 + (sensorCoeffs[4] * (int64_t)dT) / 128;
    Sensitivity = (int64_t)sensorCoeffs[1] * 32768 + (sensorCoeffs[3] * (int64_t)dT) / 256;
    TEMP = TEMP - T2; // both should be int32_t
    Offset = Offset - OFF2; // both should be int64_t
    Sensitivity = Sensitivity - Sens2; // both should be int64_t
    mbarInt = ((D1 * Sensitivity) / 2097152 - Offset) / 32768;
    mbar = (float)mbarInt / 100;
    tempC  = (float)TEMP / 100;
}

unsigned char MS_5803::MS_5803_CRC(unsigned int n_prom[]) {
    int cnt;        // simple counter
    unsigned int n_rem;   // crc reminder
    unsigned int crc_read;  // original value of the CRC
    unsigned char  n_bit;
    n_rem = 0x00;
    crc_read = sensorCoeffs[7];   // save read CRC
    sensorCoeffs[7] = (0xFF00 & (sensorCoeffs[7])); // CRC byte replaced with 0
    for (cnt = 0; cnt < 16; cnt++)
    { // choose LSB or MSB
        if (cnt%2 == 1) {
            n_rem ^= (unsigned short)((sensorCoeffs[cnt>>1]) & 0x00FF);
        }
        else {
            n_rem ^= (unsigned short)(sensorCoeffs[cnt>>1] >> 8);
        }
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000))
            {
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else {
                n_rem = (n_rem << 1);
            }
        }
    }
    n_rem = (0x000F & (n_rem >> 12));// // final 4-bit reminder is CRC code
    sensorCoeffs[7] = crc_read; // restore the crc_read to its original place
    return (n_rem ^ 0x00);
}

unsigned long MS_5803::MS_5803_ADC(char commandADC) {
    long result = 0;
    getWire(wire).beginTransmission(addr);
    getWire(wire).write(CMD_ADC_CONV + commandADC);
    getWire(wire).endTransmission();
    switch (commandADC & 0x0F)
    {
        case CMD_ADC_256 :
            delay(1); // 1 ms
            break;
        case CMD_ADC_512 :
            delay(3); // 3 ms
            break;
        case CMD_ADC_1024:
            delay(4);
            break;
        case CMD_ADC_2048:
            delay(6);
            break;
        case CMD_ADC_4096:
            delay(10);
            break;
    }
    getWire(wire).beginTransmission(addr);
    getWire(wire).write((byte)CMD_ADC_READ);
    getWire(wire).endTransmission();
    getWire(wire).requestFrom(addr, 3);
    while(getWire(wire).available()) {
        HighByte = getWire(wire).read();
        MidByte = getWire(wire).read();
        LowByte = getWire(wire).read();
    }
    result = ((long)HighByte << 16) + ((long)MidByte << 8) + (long)LowByte;
    return result;
}

void MS_5803::resetSensor() {
    getWire(wire).beginTransmission(addr);
    getWire(wire).write(CMD_RESET);
    getWire(wire).endTransmission();
    delay(10);
}