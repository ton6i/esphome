import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import (uart, text_sensor, button, sensor, binary_sensor, switch, select)
from esphome.core import CORE
from esphome.const import (
  CONF_ID,
  CONF_TYPE,
  CONF_NAME,
  CONF_ICON,
  ICON_BLUETOOTH,
)

#Source: sebcaps, windy0220, rain931215, Ld2410 arduino
DEPENDENCIES = ["uart"]
CODEOWNERS = ["@sebcaps, @ton6i"]

AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor", "button", "number", "select", "switch"]

ld2410_ns = cg.esphome_ns.namespace("ld2410_ts")
LD2410Component = ld2410_ns.class_("LD2410Component", cg.PollingComponent, uart.UARTDevice)
Buttons = ld2410_ns.class_("Buttons", button.Button, cg.Component)
ButtonType = ld2410_ns.enum("eButtonType", is_class=True)
SelectBaudrate = ld2410_ns.class_("SelectBaudrate", select.Select, cg.Component)

BUTTON_TYPES = {
    1: ButtonType.restart_device,
    2: ButtonType.restart_module,
    3: ButtonType.factor_reset,
  }

CONF_LD2410_ID = "ld2410_id"
CONF_ENGINEERINGMODE = "engineeringmode"
CONF_SETTINGS = "settings"
CONF_MAX_MOVE_DISTANCE = "max_move_distance"
CONF_MAX_STILL_DISTANCE = "max_still_distance"
CONF_NONE_DURATION = "none_duration"
CONF_G0_MOVE_THRESHOLD = "g0_move_threshold"
CONF_G0_STILL_THRESHOLD = "g0_still_threshold"
CONF_G1_MOVE_THRESHOLD = "g1_move_threshold"
CONF_G1_STILL_THRESHOLD = "g1_still_threshold"
CONF_G2_MOVE_THRESHOLD = "g2_move_threshold"
CONF_G2_STILL_THRESHOLD = "g2_still_threshold"
CONF_G3_MOVE_THRESHOLD = "g3_move_threshold"
CONF_G3_STILL_THRESHOLD = "g3_still_threshold"
CONF_G4_MOVE_THRESHOLD = "g4_move_threshold"
CONF_G4_STILL_THRESHOLD = "g4_still_threshold"
CONF_G5_MOVE_THRESHOLD = "g5_move_threshold"
CONF_G5_STILL_THRESHOLD = "g5_still_threshold"
CONF_G6_MOVE_THRESHOLD = "g6_move_threshold"
CONF_G6_STILL_THRESHOLD = "g6_still_threshold"
CONF_G7_MOVE_THRESHOLD = "g7_move_threshold"
CONF_G7_STILL_THRESHOLD = "g7_still_threshold"
CONF_G8_MOVE_THRESHOLD = "g8_move_threshold"
CONF_G8_STILL_THRESHOLD = "g8_still_threshold"
CONF_INFO= "info"
CONF_SWITCHES = "switches"
CONF_BUTTONS = "buttons"
CONF_SELECTS = "select"

name = CORE.name

CONFIG_SCHEMA = cv.All(
  cv.Schema(
    {
      cv.GenerateID(): cv.declare_id(LD2410Component),
      cv.Optional(CONF_ENGINEERINGMODE, default=False): cv.boolean,
      cv.Optional(CONF_SETTINGS, default=False): cv.boolean,
      cv.Optional(CONF_MAX_MOVE_DISTANCE, default=6): cv.int_range(min=1, max=8),
      cv.Optional(CONF_MAX_STILL_DISTANCE, default=6): cv.int_range(min=1, max=8),
      cv.Optional(CONF_NONE_DURATION, default=5): cv.int_range(min=0, max=32767),
      cv.Optional(CONF_G0_MOVE_THRESHOLD, default=50): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G0_STILL_THRESHOLD, default=0): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G1_MOVE_THRESHOLD, default=50): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G1_STILL_THRESHOLD, default=0): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G2_MOVE_THRESHOLD, default=40): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G2_STILL_THRESHOLD, default=40): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G3_MOVE_THRESHOLD, default=40): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G3_STILL_THRESHOLD, default=40): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G4_MOVE_THRESHOLD, default=40): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G4_STILL_THRESHOLD, default=40): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G5_MOVE_THRESHOLD, default=40): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G5_STILL_THRESHOLD, default=40): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G6_MOVE_THRESHOLD, default=30): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G6_STILL_THRESHOLD, default=15): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G7_MOVE_THRESHOLD, default=30): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G7_STILL_THRESHOLD, default=15): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G8_MOVE_THRESHOLD, default=30): cv.int_range(min=0, max=100),
      cv.Optional(CONF_G8_STILL_THRESHOLD, default=15): cv.int_range(min=0, max=100),
      
      cv.Optional(CONF_INFO): text_sensor.text_sensor_schema().extend(cv.polling_component_schema("60s")),

      cv.Optional(
          CONF_SELECTS,
          default=[
              {
                  CONF_NAME: name + " Baudrate",
              },
          ],
       ): [select.SELECT_SCHEMA.extend(cv.COMPONENT_SCHEMA).extend({cv.GenerateID(): cv.declare_id(SelectBaudrate)})],


      cv.Optional(
          CONF_BUTTONS,
          default=[
              {
                  CONF_NAME: name + " Restart device",
                  CONF_TYPE: 1,
              },
              {
                  CONF_NAME: name + " Restart module",
                  CONF_TYPE: 2,
              },
              {
                  CONF_NAME: name + " Factory reset",
                  CONF_TYPE: 3,
              },
          ],
      ): [button.BUTTON_SCHEMA.extend(cv.COMPONENT_SCHEMA).extend({cv.GenerateID(): cv.declare_id(Buttons), cv.Required(CONF_TYPE): cv.enum(BUTTON_TYPES, int=True)})],

    }
  ).extend(uart.UART_DEVICE_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ld2410",
#    baud_rate=256000,
    require_tx=True,
    require_rx=True,
)

async def to_code(config):
  var = cg.new_Pvariable(config[CONF_ID])
  await cg.register_component(var, config)
  await uart.register_uart_device(var, config)
  cg.add(var.set_none_duration(config[CONF_NONE_DURATION]))
  cg.add(var.set_max_move_distance(config[CONF_MAX_MOVE_DISTANCE]))
  cg.add(var.set_max_still_distance(config[CONF_MAX_STILL_DISTANCE]))
  cg.add( var.set_range_config(
          config[CONF_G0_MOVE_THRESHOLD],
          config[CONF_G0_STILL_THRESHOLD],
          config[CONF_G1_MOVE_THRESHOLD],
          config[CONF_G1_STILL_THRESHOLD],
          config[CONF_G2_MOVE_THRESHOLD],
          config[CONF_G2_STILL_THRESHOLD],
          config[CONF_G3_MOVE_THRESHOLD],
          config[CONF_G3_STILL_THRESHOLD],
          config[CONF_G4_MOVE_THRESHOLD],
          config[CONF_G4_STILL_THRESHOLD],
          config[CONF_G5_MOVE_THRESHOLD],
          config[CONF_G5_STILL_THRESHOLD],
          config[CONF_G6_MOVE_THRESHOLD],
          config[CONF_G6_STILL_THRESHOLD],
          config[CONF_G7_MOVE_THRESHOLD],
          config[CONF_G7_STILL_THRESHOLD],
          config[CONF_G8_MOVE_THRESHOLD],
          config[CONF_G8_STILL_THRESHOLD],
      )
  )

  if CONF_INFO in config:
    sens = await text_sensor.new_text_sensor(config[CONF_INFO])
    cg.add(var.set_info(sens))
  if CONF_ENGINEERINGMODE in config:
    cg.add(var.set_engineeringmode(config[CONF_ENGINEERINGMODE]))
  if CONF_SETTINGS in config:
    if config[CONF_SETTINGS] is True:
      for conf in config[CONF_BUTTONS]:
        var = await button.new_button(conf)
        parent = await cg.get_variable(config[CONF_ID])
        cg.add(var.set_parent(parent))
        cg.add(var.set_type(conf[CONF_TYPE]))
      for conf in config[CONF_SELECTS]:
        var = await select.new_select(conf, options=list({"9600", "19200", "38400", "57600", "115200", "230400", "256000", "460800"}))
        await cg.register_component(var, conf)
        parent = await cg.get_variable(config[CONF_ID])
        cg.add(var.set_parent(parent))
