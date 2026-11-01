import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, select, number, button
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_DURATION,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_SECOND,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "select", "number", "button"]

CONF_UART_ERV = "uart_erv"
CONF_UART_WALL = "uart_wall"

CONF_WALL_TEMPERATURE = "wall_temperature"
CONF_WALL_HUMIDITY = "wall_humidity"
CONF_ERV_TEMPERATURE = "erv_temperature"
CONF_SUPPLY_CFM = "supply_cfm"
CONF_EXHAUST_CFM = "exhaust_cfm"
CONF_FILTER_REMAINING_DAYS = "filter_remaining_days"

broan_proxy_ns = cg.esphome_ns.namespace("broan_erv_proxy")
BroanErvProxy = broan_proxy_ns.class_("BroanErvProxy", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BroanErvProxy),
        cv.Required(CONF_UART_ERV): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_UART_WALL): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_WALL_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_WALL_HUMIDITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ERV_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SUPPLY_CFM): sensor.sensor_schema(
            unit_of_measurement="CFM",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EXHAUST_CFM): sensor.sensor_schema(
            unit_of_measurement="CFM",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_FILTER_REMAINING_DAYS): sensor.sensor_schema(
            unit_of_measurement="d",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    uart_erv = await cg.get_variable(config[CONF_UART_ERV])
    uart_wall = await cg.get_variable(config[CONF_UART_WALL])
    cg.add(var.set_uart_erv(uart_erv))
    cg.add(var.set_uart_wall(uart_wall))

    if CONF_WALL_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_WALL_TEMPERATURE])
        cg.add(var.set_wall_temp_sensor(sens))

    if CONF_WALL_HUMIDITY in config:
        sens = await sensor.new_sensor(config[CONF_WALL_HUMIDITY])
        cg.add(var.set_wall_humidity_sensor(sens))

    if CONF_ERV_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_ERV_TEMPERATURE])
        cg.add(var.set_erv_temp_sensor(sens))

    if CONF_SUPPLY_CFM in config:
        sens = await sensor.new_sensor(config[CONF_SUPPLY_CFM])
        cg.add(var.set_supply_cfm_sensor(sens))

    if CONF_EXHAUST_CFM in config:
        sens = await sensor.new_sensor(config[CONF_EXHAUST_CFM])
        cg.add(var.set_exhaust_cfm_sensor(sens))

    if CONF_FILTER_REMAINING_DAYS in config:
        sens = await sensor.new_sensor(config[CONF_FILTER_REMAINING_DAYS])
        cg.add(var.set_filter_days_sensor(sens))