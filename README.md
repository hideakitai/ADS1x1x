# ADS1x1x

Arduino library for ADS101x / ADS111x Ultra-Small, Low-Power, I2C-Compatible, ADCs


## Supported ICs

- [ADS1018](https://www.ti.com/product/ADS1018)
- [ADS1015](https://www.ti.com/product/ADS1015)
- [ADS1014](https://www.ti.com/product/ADS1014)
- [ADS1013](https://www.ti.com/product/ADS1013)
- [ADS1118](https://www.ti.com/product/ADS1118)
- [ADS1115](https://www.ti.com/product/ADS1115)
- [ADS1114](https://www.ti.com/product/ADS1114)
- [ADS1113](https://www.ti.com/product/ADS1113)


## Usage

```C++
#include <ADS1x1x.h>
ADS1015 adc;

void setup()
{
    adc.attach(Wire);
    adc.setAddress(0x48);

    adc.inputMux(ADS1x1x::ConfigMux::SINGLE_0);
    adc.gain(ADS1x1x::ConfigPGA::FSR_4_096V);
    adc.mode(ADS1x1x::ConfigMode::CONTINUOUS);
    adc.datarate(ADS1x1x::ConfigDR::DR_1600_SPS);
    adc.compMode(ADS1x1x::ConfigCompMode::TRADITIONAL);
    adc.compPol(ADS1x1x::ConfigCompPol::ACTIVE_L);
    adc.compLatch(ADS1x1x::ConfigCompLatch::DISABLE);
    adc.compQue(ADS1x1x::ConfigCompQue::DISABLE);
    adc.loThreshVoltage(1.0);
    adc.hiThreshVoltage(2.0);
}

void loop()
{
    delay(1000);

    float val = adc.read();
    float vol = adc.voltage();

    Serial.print("adc value = ");
    Serial.print(val);
    Serial.print(", voltage = ");
    Serial.println(vol);
}
```


## APIs

```C++
// i2c settings
void attach(WireType& w);
void setAddress(const uint8_t addr);
uint8_t status() const; // i2c status

// read current mux values
bool available();
int16_t read();
double voltage();

// change mux and read values
int16_t read(const ConfigMux mux);
double voltage(const ConfigMux mux);

// wite config
uint8_t oneshotConvert();
uint8_t inputMux(const ConfigMux mux);
uint8_t gain(const ConfigPGA gain);
uint8_t mode(const ConfigMode m);
uint8_t datarate(const ConfigDR d);
uint8_t compMode(const ConfigCompMode cm);
uint8_t compPol(const ConfigCompPol cp);
uint8_t compLatch(const ConfigCompLatch cl);
uint8_t compQue(const ConfigCompQue cq);
uint8_t loThresh(const int16_t th);
uint8_t loThreshVoltage(const double th);
uint8_t hiThresh(const int16_t th);
uint8_t hiThreshVoltage(const double th);

// read config
ConfigMux inputMux();
ConfigPGA gain();
ConfigMode mode();
ConfigDR datarate();
ConfigCompMode compMode();
ConfigCompPol compPol();
ConfigCompLatch compLatch();
ConfigCompQue compQue();
int16_t loThresh();
double loThreshVoltage();
int16_t hiThresh();
double hiThreshVoltage();

// utility
double voltageResolution() const;
uint32_t conversionDelayUs() const;

// read register directly
uint16_t readRegister(const Reg r);
```

## Configuration

```C++
enum class Reg : uint8_t
{
    CONVERSION,
    CONFIG,
    LO_THRESH,
    HI_THRESH
};

enum class ConfigMux : uint8_t
{
    DIFF_0_1, // default
    DIFF_0_3,
    DIFF_1_3,
    DIFF_2_3,
    SINGLE_0,
    SINGLE_1,
    SINGLE_2,
    SINGLE_3
};

enum class ConfigPGA : uint8_t
{
    FSR_6_144V,
    FSR_4_096V,
    FSR_2_048V, // default
    FSR_1_024V,
    FSR_0_512V,
    FSR_0_256V
};

enum class ConfigMode : uint8_t
{
    CONTINUOUS,
    SINGLE_SHOT
};

enum class ConfigDR : uint8_t
{
    // for 12bit model
    DR_12B_0128_SPS = 0x00,
    DR_12B_0250_SPS,
    DR_12B_0490_SPS,
    DR_12B_0920_SPS,
    DR_12B_1600_SPS, // default
    DR_12B_2400_SPS,
    DR_12B_3300_SPS,
    // for 16bit model
    DR_16B_0008_SPS = 0x00,
    DR_16B_0016_SPS,
    DR_16B_0032_SPS,
    DR_16B_0064_SPS,
    DR_16B_0128_SPS, // default
    DR_16B_0250_SPS,
    DR_16B_0475_SPS,
    DR_16B_0860_SPS
};

enum class ConfigCompMode : uint8_t
{
    TRADITIONAL, // default
    WINDOW
};

enum class ConfigCompPol : uint8_t
{
    ACTIVE_L, // default
    ACTIVE_H
};

enum class ConfigCompLatch : uint8_t
{
    DISABLE, // default
    ENABLE
};

enum class ConfigCompQue : uint8_t
{
    ONE,
    TWO,
    FOUR,
    DISABLE
};
```

## License

MIT
