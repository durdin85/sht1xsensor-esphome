#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
// #include "SHT1x.h"

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

namespace esphome {
namespace sht1xsensor {

// #define UPDATE_INTERVAL_MS	15000

enum SHT1xSensorModel {
  SHT1x_SENSOR_MODEL_V3 = 3,
  SHT1x_SENSOR_MODEL_V4 = 4
};

enum SHT1xSensorPower {
  SHT1x_SENSOR_POWER_5V0 = 50,
  SHT1x_SENSOR_POWER_3V3 = 33
};

/// This class implements support for SHT1x sensors (10,11,12) 
class SHT1XSensor : public sensor::Sensor, public PollingComponent {
 public:
  SHT1XSensor() : PollingComponent() { }

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  void setup() override;
  void update() override;
  void dump_config() override;

  // config methods
  void set_data_pin(GPIOPin *data_pin) { data_pin_ = data_pin; }
  void set_clock_pin(GPIOPin *clock_pin) { clock_pin_ = clock_pin; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_humidity_sensor(sensor::Sensor *humidity_sensor) { humidity_sensor_ = humidity_sensor; }
  // this updates internal states
  void set_SHT1x_model(SHT1xSensorModel model) { model_ = model; }; 
  void set_SHT1x_power_rating(SHT1xSensorPower power) { power_rating_ = power; };

 protected:
  GPIOPin *data_pin_;
  GPIOPin *clock_pin_;

  sensor::Sensor *humidity_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};

  SHT1xSensorModel model_{SHT1x_SENSOR_MODEL_V3};
  SHT1xSensorPower power_rating_{SHT1x_SENSOR_POWER_5V0};
  
  // methods merged from the SHT1x.h :
  float readHumidity();
  float readTemperatureC();
  float readTemperatureF();
  int numBits_;
  float readTemperatureRaw();
  int shiftIn( int _numBits );
  void shiftOut( uint8_t bitOrder, uint8_t val );
  void sendCommandSHT( int _command );
  void waitForResultSHT();
  int getData16SHT();
  void skipCrcSHT();
};

}  // namespace sht1xsensor
}  // namespace esphome


