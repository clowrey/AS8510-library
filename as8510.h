// AS8510.h
#ifndef AS8510_H
#define AS8510_H

#include <Arduino.h>
#include <SPI.h>

// AS8510 Register Addresses (based on typical SPI interface)
#define AS8510_REG_CONFIG1      0x00
#define AS8510_REG_CONFIG2      0x01
#define AS8510_REG_CONFIG3      0x02
#define AS8510_REG_ADC1_DATA    0x10
#define AS8510_REG_ADC2_DATA    0x11
#define AS8510_REG_STATUS       0x20
#define AS8510_REG_RESET        0xFF

// Configuration bits
#define AS8510_CONFIG1_ENABLE   0x01
#define AS8510_CONFIG1_CONT     0x02
#define AS8510_CONFIG2_PGA_MASK 0x07
#define AS8510_CONFIG3_RATE_MASK 0x0F

// PGA Gain settings
enum AS8510_PGA_Gain {
  PGA_GAIN_1X = 0,
  PGA_GAIN_2X = 1,
  PGA_GAIN_4X = 2,
  PGA_GAIN_8X = 3,
  PGA_GAIN_16X = 4,
  PGA_GAIN_32X = 5,
  PGA_GAIN_64X = 6,
  PGA_GAIN_128X = 7
};

// Sample rates
enum AS8510_SampleRate {
  RATE_125_SPS = 0,
  RATE_250_SPS = 1,
  RATE_500_SPS = 2,
  RATE_1000_SPS = 3,
  RATE_2000_SPS = 4,
  RATE_4000_SPS = 5
};

class AS8510 {
private:
  int _csPin;
  SPISettings _spiSettings;
  float _shuntResistance; // in ohms
  uint8_t _pgaGain1, _pgaGain2;
  
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);
  int16_t readADC(uint8_t channel);
  
public:
  AS8510(int csPin, float shuntResistance = 0.0001); // Default 100µΩ shunt
  
  bool begin();
  void reset();
  
  // Configuration methods
  void setPGAGain(uint8_t channel, AS8510_PGA_Gain gain);
  void setSampleRate(AS8510_SampleRate rate);
  void enableContinuousMode(bool enable);
  
  // Data acquisition methods
  int16_t readRawADC(uint8_t channel);
  float readVoltage(uint8_t channel);
  float readCurrent(uint8_t channel);
  float readPower();
  
  // Status methods
  uint8_t getStatus();
  bool isDataReady();
  
  // Utility methods
  float getShuntResistance() { return _shuntResistance; }
  void setShuntResistance(float resistance) { _shuntResistance = resistance; }
};

// AS8510.cpp Implementation
AS8510::AS8510(int csPin, float shuntResistance) {
  _csPin = csPin;
  _shuntResistance = shuntResistance;
  _spiSettings = SPISettings(1000000, MSBFIRST, SPI_MODE0); // 1MHz, Mode 0
  _pgaGain1 = PGA_GAIN_1X;
  _pgaGain2 = PGA_GAIN_1X;
}

bool AS8510::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  
  SPI.begin();
  
  // Reset the device
  reset();
  delay(10);
  
  // Basic configuration
  writeRegister(AS8510_REG_CONFIG1, AS8510_CONFIG1_ENABLE);
  setSampleRate(RATE_1000_SPS);
  
  // Verify communication by reading a register
  uint8_t status = readRegister(AS8510_REG_STATUS);
  return true; // In real implementation, check for expected values
}

void AS8510::reset() {
  writeRegister(AS8510_REG_RESET, 0x01);
}

uint8_t AS8510::readRegister(uint8_t reg) {
  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  
  SPI.transfer(0x80 | reg); // Read command (MSB = 1)
  uint8_t value = SPI.transfer(0x00);
  
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  
  return value;
}

void AS8510::writeRegister(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  
  SPI.transfer(reg & 0x7F); // Write command (MSB = 0)
  SPI.transfer(value);
  
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
}

void AS8510::setPGAGain(uint8_t channel, AS8510_PGA_Gain gain) {
  if (channel == 1) {
    _pgaGain1 = gain;
    uint8_t config = readRegister(AS8510_REG_CONFIG2);
    config = (config & 0xF8) | (gain & 0x07);
    writeRegister(AS8510_REG_CONFIG2, config);
  } else if (channel == 2) {
    _pgaGain2 = gain;
    uint8_t config = readRegister(AS8510_REG_CONFIG2);
    config = (config & 0x8F) | ((gain & 0x07) << 3);
    writeRegister(AS8510_REG_CONFIG2, config);
  }
}

void AS8510::setSampleRate(AS8510_SampleRate rate) {
  uint8_t config = readRegister(AS8510_REG_CONFIG3);
  config = (config & 0xF0) | (rate & 0x0F);
  writeRegister(AS8510_REG_CONFIG3, config);
}

void AS8510::enableContinuousMode(bool enable) {
  uint8_t config = readRegister(AS8510_REG_CONFIG1);
  if (enable) {
    config |= AS8510_CONFIG1_CONT;
  } else {
    config &= ~AS8510_CONFIG1_CONT;
  }
  writeRegister(AS8510_REG_CONFIG1, config);
}

int16_t AS8510::readRawADC(uint8_t channel) {
  uint8_t reg = (channel == 1) ? AS8510_REG_ADC1_DATA : AS8510_REG_ADC2_DATA;
  
  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  
  SPI.transfer(0x80 | reg); // Read command
  uint8_t msb = SPI.transfer(0x00);
  uint8_t lsb = SPI.transfer(0x00);
  
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  
  return (int16_t)((msb << 8) | lsb);
}

float AS8510::readVoltage(uint8_t channel) {
  int16_t rawValue = readRawADC(channel);
  uint8_t gain = (channel == 1) ? _pgaGain1 : _pgaGain2;
  uint8_t gainValue = 1 << gain; // 2^gain
  
  // Convert to voltage (assuming 3.3V reference and 16-bit resolution)
  float voltage = (rawValue * 3.3) / (32768.0 * gainValue);
  return voltage;
}

float AS8510::readCurrent(uint8_t channel) {
  float voltage = readVoltage(channel);
  return voltage / _shuntResistance; // Ohm's law: I = V/R
}

float AS8510::readPower() {
  float voltage = readVoltage(1); // Assuming channel 1 for voltage
  float current = readCurrent(2);  // Assuming channel 2 for current
  return voltage * current;
}

uint8_t AS8510::getStatus() {
  return readRegister(AS8510_REG_STATUS);
}

bool AS8510::isDataReady() {
  return (getStatus() & 0x01) != 0; // Assuming bit 0 indicates data ready
}

// Example usage sketch
/*
#include "AS8510.h"

AS8510 batterySensor(10, 0.0001); // CS pin 10, 100µΩ shunt

void setup() {
  Serial.begin(115200);
  
  if (batterySensor.begin()) {
    Serial.println("AS8510 initialized successfully");
    
    // Configure PGA gains
    batterySensor.setPGAGain(1, PGA_GAIN_1X);  // Channel 1: 1x gain
    batterySensor.setPGAGain(2, PGA_GAIN_4X);  // Channel 2: 4x gain
    
    // Set sample rate
    batterySensor.setSampleRate(RATE_1000_SPS);
    
    // Enable continuous mode
    batterySensor.enableContinuousMode(true);
  } else {
    Serial.println("Failed to initialize AS8510");
  }
}

void loop() {
  if (batterySensor.isDataReady()) {
    float voltage = batterySensor.readVoltage(1);
    float current = batterySensor.readCurrent(2);
    float power = batterySensor.readPower();
    
    Serial.print("Voltage: ");
    Serial.print(voltage, 6);
    Serial.print(" V, Current: ");
    Serial.print(current, 6);
    Serial.print(" A, Power: ");
    Serial.print(power, 6);
    Serial.println(" W");
  }
  
  delay(100);
}
*/

#endif // AS8510_H
