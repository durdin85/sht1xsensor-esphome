import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor
from esphome.const import (
    CONF_HUMIDITY,
    CONF_ID,
    CONF_MODEL,
    CONF_DATA_PIN,
    CONF_CLOCK_PIN,
    CONF_TEMPERATURE,
    CONF_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
)

from esphome.cpp_helpers import gpio_pin_expression

sht1x_ns = cg.esphome_ns.namespace("sht1xsensor")

SHT1xModel = sht1x_ns.enum("SHT1xSensorModel")
SHT1x_MODELS = {
    "SHTV3": SHT1xModel.SHT1x_SENSOR_MODEL_V3,
    "SHTV4": SHT1xModel.SHT1x_SENSOR_MODEL_V4,
}
SHT1xPowerRating = sht1x_ns.enum("SHT1xSensorPower")
SHT1x_POWER_RATINGS = {
    "5V0": SHT1xPowerRating.SHT1x_SENSOR_POWER_5V0,
    "3V3": SHT1xPowerRating.SHT1x_SENSOR_POWER_3V3,
}

SHT1x = sht1x_ns.class_("SHT1XSensor", cg.PollingComponent)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SHT1x),
        cv.Required(CONF_DATA_PIN): pins.internal_gpio_input_pin_schema,
        cv.Required(CONF_CLOCK_PIN): pins.internal_gpio_input_pin_schema,
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_MODEL, default="shtV3"): cv.enum(
            SHT1x_MODELS, upper=True, space="_"
        ),
        cv.Optional(CONF_VOLTAGE, default="5V0"): cv.enum(
            SHT1x_POWER_RATINGS, upper=True, space="_"
        ),
    }
).extend(cv.polling_component_schema("60s"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    data_pin = await gpio_pin_expression(config[CONF_DATA_PIN])
    cg.add(var.set_data_pin(data_pin))
    clock_pin = await gpio_pin_expression(config[CONF_CLOCK_PIN])
    cg.add(var.set_clock_pin(clock_pin))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))
    if CONF_HUMIDITY in config:
        sens = await sensor.new_sensor(config[CONF_HUMIDITY])
        cg.add(var.set_humidity_sensor(sens))

    cg.add(var.set_SHT1x_model(config[CONF_MODEL]))
    cg.add(var.set_SHT1x_power_rating(config[CONF_VOLTAGE]))

