#include <ADS1x1x.h>

ADS1015 adc;
const uint8_t I2C_ADDR_ADC = 0x48;

void setup()
{
    Serial.begin(115200);
    delay(2000);

    adc.attach(Wire);
    adc.setAddress(I2C_ADDR_ADC);

    adc.inputMux(ADS1x1x::ConfigMux::SINGLE_0);
    adc.gain(ADS1x1x::ConfigPGA::FSR_4_096V);
    adc.mode(ADS1x1x::ConfigMode::CONTINUOUS);
    adc.datarate(ADS1x1x::ConfigDR::DR_12B_1600_SPS);
    adc.compMode(ADS1x1x::ConfigCompMode::TRADITIONAL);
    adc.compPol(ADS1x1x::ConfigCompPol::ACTIVE_L);
    adc.compLatch(ADS1x1x::ConfigCompLatch::DISABLE);
    adc.compQue(ADS1x1x::ConfigCompQue::DISABLE);
    adc.loThreshVoltage(1.0);
    adc.hiThreshVoltage(2.0);

    Serial.print("adc config = ");
    Serial.println(adc.readRegister(ADS1x1x::Reg::CONFIG), HEX);
    Serial.print("adc conversion = ");
    Serial.println(adc.readRegister(ADS1x1x::Reg::CONVERSION), HEX);

    adc.oneshotConvert();
    delay(adc.conversionDelayUs());
    Serial.print("adc available = ");
    Serial.println(adc.available());
    Serial.print("adc digital = ");
    Serial.println(adc.read());
    Serial.print("adc voltage = ");
    Serial.println(adc.voltage());
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
