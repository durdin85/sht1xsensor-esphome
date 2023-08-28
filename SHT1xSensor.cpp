#include "SHT1xSensor.h"
#include "esphome/core/log.h"
namespace esphome {
namespace sht1xsensor {

/**
  Implementation of the SHT1x sensor component for ESPHome.
  The original code is inspired by arduino library (https://github.com/practicalarduino/SHT1x), 
  however couple of modifications had been made to allow this to work with
  esphome instance. 
 */

#define MIN_TEMP -40
#define MAX_TEMP 125

static const char *TAG = "sht1xsensor.sensor";

void SHT1XSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SHT1X '%s'...", this->name_.c_str());

  this->data_pin_->setup();
  this->data_pin_->pin_mode(esphome::gpio::FLAG_INPUT);
  this->clock_pin_->setup();
  this->clock_pin_->pin_mode(esphome::gpio::FLAG_INPUT);
  // no longer needed: this->sht1x_ = new SHT1x(this->data_pin_, this->clock_pin_);

  ESP_LOGCONFIG(TAG, "Done setting up SHT1X '%s'...", this->name_.c_str());
}

void SHT1XSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "SHT1x:");
  LOG_PIN("  SHT1x Data pin ", this->data_pin_);
  LOG_PIN("  SHT1x Clock pin ", this->clock_pin_);
  LOG_SENSOR("  ", "Temperature C", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
}

void SHT1XSensor::update() {
  float temp = this->readTemperatureC();

  ESP_LOGD(TAG, "Got temperature=%.1f dgC", temp);

  if (temp >= MIN_TEMP && temp <= MAX_TEMP) {
    if (this->temperature_sensor_ != nullptr) {
      this->temperature_sensor_->publish_state(temp);
    }
  }
  
  float humidity = this->readHumidity();

  ESP_LOGD(TAG, "Got humidity=%.1f perc", humidity);

  if (humidity >= 0 && humidity <= 100) {
    if (this->humidity_sensor_ != nullptr) {
      this->humidity_sensor_->publish_state(humidity);
    }
  }
}

/**
 * Reads the current temperature in degrees Celsius
 */
float SHT1XSensor::readTemperatureC() {
  int _val;                // Raw value returned from sensor
  float _temperature;      // Temperature derived from raw value
  float D1;

  // Conversion coefficients from SHT15 datasheet
  /* 3.5 V: -39.7
     3 V: -39.6
     2.5 V: -39.4  */
  if ( this->power_rating_ == SHT1x_SENSOR_POWER_5V0 ) {
    D1 = -40.1;  // for 14 Bit @ 5V
  } else {
    D1 = -39.66;
  }
  const float D2 =   0.01; // for 14 Bit DEGC

  // Fetch raw value
  _val = this->readTemperatureRaw();

  // Convert raw value to degrees Celsius
  _temperature = (_val * D2) + D1;

  return (_temperature);
}

/**
 * Reads the current temperature in degrees Fahrenheit
 */
float SHT1XSensor::readTemperatureF() {
  int _val;                 // Raw value returned from sensor
  float _temperature;       // Temperature derived from raw value
  float D1;
  // Conversion coefficients from SHT15 datasheet
  /* 3.5 V: -39.5
     3 V: -39.3  */
  if ( this->power_rating_ == SHT1x_SENSOR_POWER_5V0 ) {
    D1 = -40.2;   // for 14 Bit @ 5V
  } else {
    D1 = -39.4;   // for 14 Bit @ 3.3V
  }
  const float D2 =   0.018; // for 14 Bit DEGF

  // Fetch raw value
  _val = this->readTemperatureRaw();

  // Convert raw value to degrees Fahrenheit
  _temperature = (_val * D2) + D1;

  return (_temperature);
}

/**
 * Reads the current raw temperature value
 */
float SHT1XSensor::readTemperatureRaw() {
  int _val;

  // Command to send to the SHT1x to request Temperature
  int _gTempCmd  = 0b00000011;

  this->sendCommandSHT(_gTempCmd);
  this->waitForResultSHT();
  _val = this->getData16SHT();
  this->skipCrcSHT();

  return (_val);
}

/**
 * Reads current temperature-corrected relative humidity
 */
float SHT1XSensor::readHumidity() {
  int _val;                    // Raw humidity value returned from sensor
  float _linearHumidity;       // Humidity with linear correction applied
  float _correctedHumidity;    // Temperature-corrected humidity
  float _temperature;          // Raw temperature value
  float C1, C2, C3;

  // Conversion coefficients from SHT15 datasheet, this is for V3 sensors
  if ( this->model_ == SHT1x_SENSOR_MODEL_V3 ) {
    C1 = -4.0;       // for 12 Bit
    C2 =  0.0405;    // for 12 Bit
    C3 = -0.0000028; // for 12 Bit
  } else {
    // if this->model == SHTV4 :
    C1 = -2.0468;
    C2 = 0.0367;
    C3 = -0.0000015955;
  }
  const float T1 =  0.01;      // for 14 Bit
  const float T2 =  0.00008;   // for 14 Bit

  // Command to send to the SHT1x to request humidity
  int _gHumidCmd = 0b00000101;

  // Fetch the value from the sensor
  this->sendCommandSHT(_gHumidCmd);
  this->waitForResultSHT();
  _val = this->getData16SHT();
  this->skipCrcSHT();

  // Apply linear conversion to raw value
  _linearHumidity = C1 + C2 * _val + C3 * _val * _val;

  // Get current temperature for humidity correction
  _temperature = this->readTemperatureC();

  // Correct humidity value for current temperature
  _correctedHumidity = (_temperature - 25.0 ) * (T1 + T2 * _val) + _linearHumidity;

  return (_correctedHumidity);
}

int SHT1XSensor::shiftIn(int _numBits) {
  int ret = 0;
  int i;

  for ( i=0; i<_numBits; ++i ) {
     this->clock_pin_->digital_write( HIGH );
     delay(10);  // I don't know why I need this, but without it I don't get my 8 lsb of temp
     ret = ret*2 + this->data_pin_->digital_read();
     this->clock_pin_->digital_write( LOW );
  }

  return(ret);
}

void SHT1XSensor::shiftOut( uint8_t bitOrder, uint8_t val ) {
    uint8_t i;

    for (i = 0; i < 8; i++)  {
        if (bitOrder == LSBFIRST)
            this->data_pin_->digital_write( !!(val & (1 << i)) );
        else    
            this->data_pin_->digital_write( !!(val & (1 << (7 - i))) );

        this->clock_pin_->digital_write( HIGH );
        this->clock_pin_->digital_write( LOW );    
    }
}

void SHT1XSensor::sendCommandSHT(int _command ) {
  int ack;

  // Transmission Start
  this->data_pin_->pin_mode(esphome::gpio::FLAG_OUTPUT);
  this->clock_pin_->pin_mode(esphome::gpio::FLAG_OUTPUT);
  this->data_pin_->digital_write( HIGH );
  this->clock_pin_->digital_write( HIGH );
  this->data_pin_->digital_write( LOW );
  this->clock_pin_->digital_write( LOW );
  this->clock_pin_->digital_write( HIGH );
  this->data_pin_->digital_write( HIGH );
  this->clock_pin_->digital_write( LOW );

  // The command (3 msb are address and must be 000, and last 5 bits are command)
  this->shiftOut(MSBFIRST, _command);

  // Verify we get the correct ack
  this->clock_pin_->digital_write( HIGH );
  this->data_pin_->pin_mode(esphome::gpio::FLAG_INPUT);
  ack = this->data_pin_->digital_read();
  if ( ack != LOW ) {
    ESP_LOGD(TAG, "ACK Error 0");
  }
  this->clock_pin_->digital_write( LOW );
  ack = this->data_pin_->digital_read();
  if ( ack != HIGH ) {
    ESP_LOGD(TAG, "ACK Error 1");
  }
}

void SHT1XSensor::waitForResultSHT() {
  int i;
  int ack;

  this->data_pin_->pin_mode(esphome::gpio::FLAG_INPUT);
 
  for(i = 0; i < 100; ++i) {
    delay(10); // well it should be done different way?
    ack = this->data_pin_->digital_read();

    if (ack == LOW) {
      break;
    }
  }

  if (ack == HIGH) {
    ESP_LOGD(TAG, "ACK Error 2");
  }
}

/**
 */
int SHT1XSensor::getData16SHT() {
  int val;

  // Get the most significant bits
  this->data_pin_->pin_mode(esphome::gpio::FLAG_INPUT);
  this->clock_pin_->pin_mode(esphome::gpio::FLAG_OUTPUT);

  val = this->shiftIn( 8 );
  val *= 256;

  // Send the required ack
  this->data_pin_->pin_mode(esphome::gpio::FLAG_OUTPUT);

  this->data_pin_->digital_write( HIGH );
  this->data_pin_->digital_write( LOW );
  this->clock_pin_->digital_write( HIGH );
  this->clock_pin_->digital_write( LOW );

  // Get the least significant bits
  this->data_pin_->pin_mode(esphome::gpio::FLAG_INPUT);
  val |= this->shiftIn( 8 );

  return val;
}

/**
 */
void SHT1XSensor::skipCrcSHT() {
  // Skip acknowledge to end trans (no CRC)
  this->data_pin_->pin_mode(esphome::gpio::FLAG_OUTPUT);
  this->clock_pin_->pin_mode(esphome::gpio::FLAG_OUTPUT);

  this->data_pin_->digital_write( HIGH );
  this->clock_pin_->digital_write( HIGH );
  this->clock_pin_->digital_write( LOW );
}


}  // namespace sht1xsensor
}  // namespace esphome

