from esphome import automation
import esphome.codegen as cg
from esphome.components import binary_sensor, climate, remote_transmitter, sensor, uart
from esphome.components.remote_base import CONF_TRANSMITTER_ID
import esphome.config_validation as cv
from esphome.const import (
    CONF_BEEPER,
    CONF_DISPLAY,
    CONF_ID,
    CONF_OUTDOOR_TEMPERATURE,
    CONF_TEMPERATURE,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_FAN,
    ICON_POWER,
    ICON_THERMOMETER,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_WATT,
)
from esphome.core import coroutine

CODEOWNERS = ["@mikesmitty"]
DEPENDENCIES = ["climate", "uart"]
AUTO_LOAD = ["binary_sensor", "sensor"]

CONF_DEFROST_STATUS = "defrost_status"
CONF_INDOOR_FAN_SPEED = "indoor_fan_speed"
CONF_OUTDOOR_FAN_SPEED = "outdoor_fan_speed"
CONF_POWER_USAGE = "power_usage"
ICON_HVAC = "mdi:hvac"
ICON_SNOWFLAKE_THERMOMETER = "mdi:snowflake-thermometer"

pioneer_wyt_ns = cg.esphome_ns.namespace("pioneer").namespace("wyt")
WytClimate = pioneer_wyt_ns.class_(
    "WytClimate", climate.Climate, cg.PollingComponent, uart.UARTDevice
)


CONFIG_SCHEMA = cv.All(
    climate.climate_schema(WytClimate)
    .extend(
        {
            cv.Optional(CONF_BEEPER, default=False): cv.boolean,
            cv.Optional(CONF_DISPLAY, default=True): cv.boolean,
            cv.OnlyWith(CONF_TRANSMITTER_ID, "remote_transmitter"): cv.use_id(
                remote_transmitter.RemoteTransmitterComponent
            ),
            cv.Optional(CONF_DEFROST_STATUS): binary_sensor.binary_sensor_schema(
                icon=ICON_SNOWFLAKE_THERMOMETER,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_INDOOR_FAN_SPEED): sensor.sensor_schema(
                icon=ICON_FAN,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_OUTDOOR_FAN_SPEED): sensor.sensor_schema(
                icon=ICON_HVAC,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_OUTDOOR_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_POWER_USAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                icon=ICON_POWER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
    .extend(cv.polling_component_schema("2s"))
)

RemoteTempAction = pioneer_wyt_ns.class_("RemoteTempAction", automation.Action)
DisplayToggleAction = pioneer_wyt_ns.class_("DisplayToggleAction", automation.Action)
BeeperOnAction = pioneer_wyt_ns.class_("BeeperOnAction", automation.Action)
BeeperOffAction = pioneer_wyt_ns.class_("BeeperOffAction", automation.Action)

PIONEER_WYT_ACTION_BASE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.use_id(WytClimate),
    }
)

PIONEER_WYT_REMOTE_TEMP_MIN = 0  # FIXME
PIONEER_WYT_REMOTE_TEMP_MAX = 37  # FIXME
PIONEER_WYT_REMOTE_TEMP_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_TEMPERATURE): cv.templatable(cv.temperature),
        cv.Optional(CONF_BEEPER, default=False): cv.templatable(cv.boolean),
    }
)


def templatize(value):
    if isinstance(value, cv.Schema):
        value = value.schema
    ret = {}
    for key, val in value.items():
        ret[key] = cv.templatable(val)
    return cv.Schema(ret)


def register_action(name, type_, schema, synchronous=True):
    validator = templatize(schema).extend(PIONEER_WYT_ACTION_BASE_SCHEMA)
    registerer = automation.register_action(
        f"pioneer_wyt.{name}", type_, validator, synchronous=synchronous
    )

    def decorator(func):
        async def new_func(config, action_id, template_arg, args):
            wyt = await cg.get_variable(config[CONF_ID])
            var = cg.new_Pvariable(action_id, template_arg)
            cg.add(var.set_parent(wyt))
            await coroutine(func)(var, config, args)
            return var

        return registerer(new_func)

    return decorator


@register_action("remote_temp", RemoteTempAction, PIONEER_WYT_REMOTE_TEMP_SCHEMA)
async def remote_temp_to_code(var, config, args):
    template_ = await cg.templatable(config[CONF_BEEPER], args, cg.bool_)
    cg.add(var.set_beeper(template_))
    template_ = await cg.templatable(config[CONF_TEMPERATURE], args, cg.float_)
    cg.add(var.set_temperature(template_))


@register_action(
    "display_toggle",
    DisplayToggleAction,
    cv.Schema({}),
)
async def display_toggle_to_code(var, config, args):
    pass


@register_action(
    "beeper_on",
    BeeperOnAction,
    cv.Schema({}),
)
async def beeper_on_to_code(var, config, args):
    pass


@register_action(
    "beeper_off",
    BeeperOffAction,
    cv.Schema({}),
)
async def beeper_off_to_code(var, config, args):
    pass


async def to_code(config):
    var = await climate.new_climate(config)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_beeper(config[CONF_BEEPER]))
    cg.add(var.set_display(config[CONF_DISPLAY]))
    if CONF_TRANSMITTER_ID in config:
        cg.add_define("USE_REMOTE_TRANSMITTER")
        transmitter_ = await cg.get_variable(config[CONF_TRANSMITTER_ID])
        cg.add(var.set_transmitter(transmitter_))
    if CONF_DEFROST_STATUS in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_DEFROST_STATUS])
        cg.add(var.set_defrost_binary_sensor(sens))
    if CONF_INDOOR_FAN_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_INDOOR_FAN_SPEED])
        cg.add(var.set_indoor_fan_speed_sensor(sens))
    if CONF_OUTDOOR_FAN_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_OUTDOOR_FAN_SPEED])
        cg.add(var.set_outdoor_fan_speed_sensor(sens))
    if CONF_OUTDOOR_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_OUTDOOR_TEMPERATURE])
        cg.add(var.set_outdoor_temperature_sensor(sens))
    if CONF_POWER_USAGE in config:
        sens = await sensor.new_sensor(config[CONF_POWER_USAGE])
        cg.add(var.set_power_sensor(sens))
