#pragma once
#ifndef ARDUINO_ADC_ADS1015_H
#define ARDUINO_ADC_ADS1015_H

#include <Arduino.h>
#include <Wire.h>

namespace arduino {
namespace ads1x1x {

enum class Reg : uint8_t
{
    CONVERSION,
    CONFIG,
    LO_THRESH,
    HI_THRESH
};

enum class ConfigBitBegin : uint8_t
{
    OS = 15,
    MUX = 12,
    PGA = 9,
    MODE = 8,
    DR = 5,
    COMP_MODE = 4,
    COMP_POL = 3,
    COMP_LAT = 2,
    COMP_QUE = 0
};

enum class ConfigBitSize : uint8_t
{
    OS = 1,
    MUX = 3,
    PGA = 3,
    MODE = 1,
    DR = 3,
    COMP_MODE = 1,
    COMP_POL = 1,
    COMP_LAT = 1,
    COMP_QUE = 2
};

enum class ConfigBitMask : uint16_t
{
    OS = 0x8000,
    MUX = 0x7000,
    PGA = 0x0E00,
    MODE = 0x0100,
    DR = 0x00E0,
    COMP_MODE = 0x0010,
    COMP_POL = 0x0008,
    COMP_LAT = 0x0004,
    COMP_QUE = 0x0003
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

template <typename WireType, uint8_t N_RESOLUTION, uint8_t N_CHANNELS>
class Ads1x1x
{
    static constexpr uint8_t DEFAULT_I2C_ADDR {0x48};
    const uint8_t bit_offset { (N_RESOLUTION == 12) ? 4 : 0 };
    uint8_t i2c_addr {DEFAULT_I2C_ADDR};

    WireType* wire;
    uint8_t status_;

    ConfigPGA pga { ConfigPGA::FSR_2_048V }; // TODO: temporary
    ConfigDR dr { ConfigDR::DR_12B_1600_SPS }; // TODO: temporary

public:

    void attach(WireType& w) { wire = &w; }

    void setAddress(const uint8_t addr) { i2c_addr = addr; }

    uint8_t status() const { return status_; }

    uint16_t readRegister(const Reg r)
    {
        return readWord(i2c_addr, r);
    }

    bool available() { return readBitWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::OS); }

    uint8_t oneshotConvert() { return writeBitWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::OS, 1); }

    uint8_t inputMux(const ConfigMux mux)
    {
        return writeBitsWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::MUX, ConfigBitSize::MUX, (uint8_t)mux);
    }

    ConfigMux inputMux()
    {
        return (ConfigMux)readBitsWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::MUX, ConfigBitSize::MUX);
    }

    int16_t read()
    {
        return ((int16_t)readWord(i2c_addr, Reg::CONVERSION) >> bit_offset);
    }

    int16_t read(const ConfigMux mux)
    {
        inputMux(mux);
        delay(conversionDelayUs());
        return read();
    }

    double voltage()
    {
        return (double)read() * voltageResolution();
    }

    double voltage(const ConfigMux mux)
    {
        return (double)read(mux) * voltageResolution();
    }

    uint8_t gain(const ConfigPGA gain)
    {
        pga = gain;
        return writeBitsWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::PGA, ConfigBitSize::PGA, (uint8_t)pga);
    }

    ConfigPGA gain()
    {
        return (ConfigPGA)readBitsWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::PGA, ConfigBitSize::PGA);
    }

    uint8_t mode(const ConfigMode m)
    {
        return writeBitWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::MODE, (uint8_t)m);
    }

    ConfigMode mode()
    {
        return (ConfigMode)readBitWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::MODE);
    }

    uint8_t datarate(const ConfigDR d)
    {
        dr = d;
        return writeBitsWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::DR, ConfigBitSize::DR, (uint8_t)dr);
    }

    ConfigDR datarate()
    {
        return (ConfigDR)readBitsWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::DR, ConfigBitSize::DR);
    }

    uint8_t compMode(const ConfigCompMode cm)
    {
        return writeBitWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::COMP_MODE, (uint8_t)cm);
    }

    ConfigCompMode compMode()
    {
        return (ConfigCompMode)readBitWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::COMP_MODE);
    }

    uint8_t compPol(const ConfigCompPol cp)
    {
        return writeBitWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::COMP_POL, (uint8_t)cp);
    }

    ConfigCompPol compPol()
    {
        return (ConfigCompPol)readBitWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::COMP_POL);
    }

    uint8_t compLatch(const ConfigCompLatch cl)
    {
        return writeBitWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::COMP_LAT, (uint8_t)cl);
    }

    ConfigCompLatch compLatch()
    {
        return (ConfigCompLatch)readBitWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::COMP_LAT);
    }

    uint8_t compQue(const ConfigCompQue cq)
    {
        return writeBitsWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::COMP_QUE, ConfigBitSize::COMP_QUE, (uint8_t)cq);
    }

    ConfigCompQue compQue()
    {
        return (ConfigCompQue)readBitsWord(i2c_addr, Reg::CONFIG, ConfigBitBegin::COMP_QUE, ConfigBitSize::COMP_QUE);
    }

    uint8_t loThresh(const int16_t th)
    {
        int16_t v = (th & ((1 << N_RESOLUTION) - 1)) << bit_offset;
        return writeWord(i2c_addr, Reg::LO_THRESH, v);
    }

    int16_t loThresh()
    {
        return readWord(i2c_addr, Reg::LO_THRESH) >> bit_offset;
    }

    uint8_t loThreshVoltage(const double th)
    {
        return loThresh((int16_t)(th / voltageResolution()));
    }

    double loThreshVoltage()
    {
        return loThresh() * voltageResolution();
    }

    uint8_t hiThresh(const int16_t th)
    {
        int16_t v = (th & ((1 << N_RESOLUTION) - 1)) << bit_offset;
        return writeWord(i2c_addr, Reg::HI_THRESH, v);
    }

    int16_t hiThresh()
    {
        return readWord(i2c_addr, Reg::HI_THRESH) >> bit_offset;
    }

    uint8_t hiThreshVoltage(const double th)
    {
        return hiThresh((int16_t)(th / voltageResolution()));
    }

    double hiThreshVoltage()
    {
        return hiThresh() * voltageResolution();
    }

    double voltageResolution() const
    {
        switch(pga)
        {
            case ConfigPGA::FSR_6_144V: return 6.144 / (double)0x7FF;
            case ConfigPGA::FSR_4_096V: return 4.096 / (double)0x7FF;
            case ConfigPGA::FSR_2_048V: return 2.048 / (double)0x7FF;
            case ConfigPGA::FSR_1_024V: return 1.024 / (double)0x7FF;
            case ConfigPGA::FSR_0_512V: return 0.512 / (double)0x7FF;
            case ConfigPGA::FSR_0_256V: return 0.256 / (double)0x7FF;
            default:                    return 2.048 / (double)0x7FF;
        }
    }

    uint32_t conversionDelayUs() const
    {
        if (N_RESOLUTION == 12)
        {
            switch (dr)
            {
                case ConfigDR::DR_12B_0128_SPS: return 1000000 / 128;
                case ConfigDR::DR_12B_0250_SPS: return 1000000 / 250;
                case ConfigDR::DR_12B_0490_SPS: return 1000000 / 490;
                case ConfigDR::DR_12B_0920_SPS: return 1000000 / 920;
                case ConfigDR::DR_12B_1600_SPS: return 1000000 / 1600;
                case ConfigDR::DR_12B_2400_SPS: return 1000000 / 2400;
                case ConfigDR::DR_12B_3300_SPS: return 1000000 / 3300;
                default:                        return 0;
            }
        }
        else
        {
            switch (dr)
            {
                case ConfigDR::DR_16B_0008_SPS: return 1000000 / 8;
                case ConfigDR::DR_16B_0016_SPS: return 1000000 / 16;
                case ConfigDR::DR_16B_0032_SPS: return 1000000 / 32;
                case ConfigDR::DR_16B_0064_SPS: return 1000000 / 64;
                case ConfigDR::DR_16B_0128_SPS: return 1000000 / 128;
                case ConfigDR::DR_16B_0250_SPS: return 1000000 / 250;
                case ConfigDR::DR_16B_0475_SPS: return 1000000 / 475;
                case ConfigDR::DR_16B_0860_SPS: return 1000000 / 860;
                default:                        return 0;
            }
        }
    }

private:

    uint16_t readBitWord(const uint8_t dev, const Reg reg, const ConfigBitBegin bit)
    {
        uint16_t b = readWord(dev, reg);
        b &= (1 << (uint8_t)bit);
        return b;
    }

    uint16_t readBitsWord(const uint8_t dev, const Reg reg, const ConfigBitBegin bit_begin, const ConfigBitSize size)
    {
        uint16_t w = readWord(dev, reg);
        uint16_t mask = ((1 << (uint8_t)size) - 1) << (uint8_t)bit_begin;
        w &= mask;
        w >>= (uint8_t)bit_begin;
        return w;
    }

    uint16_t readWord(const uint8_t dev, const Reg reg)
    {
        uint16_t data;
        readWords(dev, reg, 1, &data);
        return data;
    }

    int8_t readWords(const uint8_t dev, const Reg reg, const uint8_t size, uint16_t *data)
    {
        wire->beginTransmission(dev);
        wire->write((uint8_t)reg);
        status_ = wire->endTransmission();
        if (status_ != 0)
        {
            Serial.print("I2C error : ");
            Serial.println(status_);
        }
        wire->requestFrom(dev, (uint8_t)(size * 2));
        int8_t count = 0;
        bool msb = true;
        while (wire->available() && (count < size))
        {
            if (msb) data[count]    = wire->read() << 8;
            else     data[count++] |= wire->read();
            msb = !msb;
        }
        return count;
    }

    bool writeBitWord(const uint8_t dev, const Reg reg, const ConfigBitBegin bit, const uint16_t data)
    {
        uint16_t w = readWord(dev, reg);
        w = (data != 0) ? (w | (1 << (uint8_t)bit)) : (w & ~(1 << (uint8_t)bit));
        return writeWord(dev, reg, w);
    }

    bool writeBitsWord(const uint8_t dev, const Reg reg, const ConfigBitBegin bit_begin, const ConfigBitSize size, uint16_t data)
    {
        uint16_t w = readWord(dev, reg);
        uint16_t mask = ((1 << (uint8_t)size) - 1) << (uint8_t)bit_begin;
        data <<= (uint8_t)bit_begin; // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        return writeWord(dev, reg, w);
    }

    bool writeWord(const uint8_t dev, const Reg reg, const uint16_t data)
    {
        return writeWords(dev, reg, 1, &data);
    }

    bool writeWords(const uint8_t dev, const Reg reg, const uint8_t size, const uint16_t* data)
    {
        wire->beginTransmission(dev);
        wire->write((uint8_t)reg);
        for (uint8_t i = 0; i < size; i++)
        {
            wire->write((uint8_t)((data[i] >> 8) & 0x00FF));
            wire->write((uint8_t)((data[i] >> 0) & 0x00FF));
        }
        status_ = wire->endTransmission();
        if (status_ != 0)
        {
            Serial.print("I2C error : ");
            Serial.println(status_);
        }
        return (status_ == 0);
    };

};

} // ads1x1x
} // arduino

namespace ADS1x1x = arduino::ads1x1x;
using ADS1115 = ADS1x1x::Ads1x1x<TwoWire, 16, 4>;
using ADS1114 = ADS1x1x::Ads1x1x<TwoWire, 16, 1>;
using ADS1113 = ADS1x1x::Ads1x1x<TwoWire, 16, 1>;
using ADS1015 = ADS1x1x::Ads1x1x<TwoWire, 12, 4>;
using ADS1014 = ADS1x1x::Ads1x1x<TwoWire, 12, 1>;
using ADS1013 = ADS1x1x::Ads1x1x<TwoWire, 12, 1>;
using ADS1118 = ADS1x1x::Ads1x1x<TwoWire, 16, 4>;
using ADS1018 = ADS1x1x::Ads1x1x<TwoWire, 12, 4>;

#endif // ARDUINO_ADC_ADS1015_H
