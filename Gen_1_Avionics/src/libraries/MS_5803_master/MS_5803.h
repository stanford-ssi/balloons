#include <i2c_t3.h>

#define CMD_RESET    0x1E  // ADC reset command
#define CMD_ADC_READ  0x00  // ADC read command
#define CMD_ADC_CONV  0x40  // ADC conversion command
#define CMD_ADC_D1    0x00  // ADC D1 conversion
#define CMD_ADC_D2    0x10  // ADC D2 conversion
#define CMD_ADC_256   0x00  // ADC resolution=256
#define CMD_ADC_512   0x02  // ADC resolution=512
#define CMD_ADC_1024  0x04  // ADC resolution=1024
#define CMD_ADC_2048  0x06  // ADC resolution=2048
#define CMD_ADC_4096  0x08  // ADC resolution=4096

class MS_5803 {
public:
    MS_5803(uint16_t Resolution, bool w, int a);
    boolean initializeMS_5803(boolean Verbose = true);
    void resetSensor();
    void readSensor();
    float temperature() const       {return tempC;}
    float pressure() const          {return mbar;}
    unsigned long D1val() const   {return D1;}
    unsigned long D2val() const   {return D2;}
    
    
private:
    bool wire;
    float mbar; // Store pressure in mbar.
    float tempC; // Store temperature in degrees Celsius
    unsigned long D1; // Store D1 value
    unsigned long D2; // Store D2 value
    int32_t mbarInt; // pressure in mbar, initially as a signed long integer
    unsigned char MS_5803_CRC(unsigned int n_prom[]);
    unsigned long MS_5803_ADC(char commandADC);
    uint16_t _Resolution;
    int addr;
    
    unsigned int      sensorCoeffs[8];
    int32_t  dT = 0;
    int32_t  TEMP = 0;
    int64_t  Offset = 0;
    int64_t  Sensitivity  = 0;
    int64_t  T2 = 0;
    int64_t  OFF2 = 0;
    int64_t  Sens2 = 0;
    byte HighByte;
    byte MidByte;
    byte LowByte;
    
};


