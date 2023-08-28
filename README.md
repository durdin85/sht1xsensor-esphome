# sht1xsensor-esphome
SHT1x sensor custom component for ESPHome

# How to use this sensor/custom component?

- Create folder named "custom_components" in your ESPHome configuration folder (where the yaml files are).
- Inside the custom_components, create folder named "sht1xsensor", and download all files from this repository inside the folder.

After files are present, you may use the sensor as usual, but there are couple of configuration options which needs to be set:

|Config option|Explanation|
|-|-|
|data_pin| Pin used as data pin on ESP device |
|clock_pin| Pin used as clock pin on ESP device |
|voltage| (optional) What volatge is used to power sensor? Possible: 5V or 3.3V |
|sensor_model| (optional) It is version/generation 3 or 4 sensor? |

Example:
```
sensor:
  - platform: sht1xsensor
    data_pin: GPIO0
    clock_pin: GPIO2
    temperature:
      name: "Temperature"
      accuracy_decimals: 2
    humidity:
      name: "Humidity"
      accuracy_decimals: 2
    update_interval: 60s
    voltage: 3V3
```
