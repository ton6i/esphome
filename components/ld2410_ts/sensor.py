import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_POWER_FACTOR,
    UNIT_PERCENT,
)
from . import CONF_LD2410_ID, LD2410Component

DEPENDENCIES = ["ld2410_ts"]
CONF_MOVING_DISTANCE = "moving_distance"
CONF_STILL_DISTANCE = "still_distance"
CONF_MOVING_ENERGY = "moving_energy"
CONF_STILL_ENERGY = "still_energy"
CONF_DETECTION_DISTANCE = "detection_distance"
UNIT_CENTIMETER = "cm"
CONF_G0_MOVE = "g0_move"
CONF_G1_MOVE = "g1_move"
CONF_G2_MOVE = "g2_move"
CONF_G3_MOVE = "g3_move"
CONF_G4_MOVE = "g4_move"
CONF_G5_MOVE = "g5_move"
CONF_G6_MOVE = "g6_move"
CONF_G7_MOVE = "g7_move"
CONF_G8_MOVE = "g8_move"

CONF_G0_STILL = "g0_still"
CONF_G1_STILL = "g1_still"
CONF_G2_STILL = "g2_still"
CONF_G3_STILL = "g3_still"
CONF_G4_STILL = "g4_still"
CONF_G5_STILL = "g5_still"
CONF_G6_STILL = "g6_still"
CONF_G7_STILL = "g7_still"
CONF_G8_STILL = "g8_still"

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_LD2410_ID): cv.use_id(LD2410Component),
    cv.Optional(CONF_DETECTION_DISTANCE): sensor.sensor_schema(device_class=DEVICE_CLASS_DISTANCE, unit_of_measurement=UNIT_CENTIMETER),
    cv.Optional(CONF_MOVING_DISTANCE): sensor.sensor_schema(device_class=DEVICE_CLASS_DISTANCE, unit_of_measurement=UNIT_CENTIMETER),
    cv.Optional(CONF_STILL_DISTANCE): sensor.sensor_schema(device_class=DEVICE_CLASS_DISTANCE, unit_of_measurement=UNIT_CENTIMETER),
    cv.Optional(CONF_MOVING_ENERGY): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_STILL_ENERGY): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G0_MOVE): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G1_MOVE): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G2_MOVE): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G3_MOVE): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G4_MOVE): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G5_MOVE): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G6_MOVE): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G7_MOVE): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G8_MOVE): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G0_STILL): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G1_STILL): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G2_STILL): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G3_STILL): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G4_STILL): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G5_STILL): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G6_STILL): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G7_STILL): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
    cv.Optional(CONF_G8_STILL): sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR, unit_of_measurement=UNIT_PERCENT),
}

async def to_code(config):
    ld2410_component = await cg.get_variable(config[CONF_LD2410_ID])
    if CONF_MOVING_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_MOVING_DISTANCE])
        cg.add(ld2410_component.set_moving_distance_sensor(sens))
    if CONF_STILL_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_STILL_DISTANCE])
        cg.add(ld2410_component.set_still_distance_sensor(sens))
    if CONF_MOVING_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_MOVING_ENERGY])
        cg.add(ld2410_component.set_moving_energy_sensor(sens))
    if CONF_STILL_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_STILL_ENERGY])
        cg.add(ld2410_component.set_still_energy_sensor(sens))
    if CONF_DETECTION_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DETECTION_DISTANCE])
        cg.add(ld2410_component.set_detection_distance_sensor(sens))

    if CONF_G0_MOVE in config:
        sens = await sensor.new_sensor(config[CONF_G0_MOVE])
        cg.add(ld2410_component.set_g0_move_sensor(sens))
    if CONF_G1_MOVE in config:
        sens = await sensor.new_sensor(config[CONF_G1_MOVE])
        cg.add(ld2410_component.set_g1_move_sensor(sens))
    if CONF_G2_MOVE in config:
        sens = await sensor.new_sensor(config[CONF_G2_MOVE])
        cg.add(ld2410_component.set_g2_move_sensor(sens))
    if CONF_G3_MOVE in config:
        sens = await sensor.new_sensor(config[CONF_G3_MOVE])
        cg.add(ld2410_component.set_g3_move_sensor(sens))
    if CONF_G4_MOVE in config:
        sens = await sensor.new_sensor(config[CONF_G4_MOVE])
        cg.add(ld2410_component.set_g4_move_sensor(sens))
    if CONF_G5_MOVE in config:
        sens = await sensor.new_sensor(config[CONF_G5_MOVE])
        cg.add(ld2410_component.set_g5_move_sensor(sens))
    if CONF_G6_MOVE in config:
        sens = await sensor.new_sensor(config[CONF_G6_MOVE])
        cg.add(ld2410_component.set_g6_move_sensor(sens))
    if CONF_G7_MOVE in config:
        sens = await sensor.new_sensor(config[CONF_G7_MOVE])
        cg.add(ld2410_component.set_g7_move_sensor(sens))
    if CONF_G8_MOVE in config:
        sens = await sensor.new_sensor(config[CONF_G8_MOVE])
        cg.add(ld2410_component.set_g8_move_sensor(sens))

    if CONF_G0_STILL in config:
        sens = await sensor.new_sensor(config[CONF_G0_STILL])
        cg.add(ld2410_component.set_g0_still_sensor(sens))
    if CONF_G1_STILL in config:
        sens = await sensor.new_sensor(config[CONF_G1_STILL])
        cg.add(ld2410_component.set_g1_still_sensor(sens))
    if CONF_G2_STILL in config:
        sens = await sensor.new_sensor(config[CONF_G2_STILL])
        cg.add(ld2410_component.set_g2_still_sensor(sens))
    if CONF_G3_STILL in config:
        sens = await sensor.new_sensor(config[CONF_G3_STILL])
        cg.add(ld2410_component.set_g3_still_sensor(sens))
    if CONF_G4_STILL in config:
        sens = await sensor.new_sensor(config[CONF_G4_STILL])
        cg.add(ld2410_component.set_g4_still_sensor(sens))
    if CONF_G5_STILL in config:
        sens = await sensor.new_sensor(config[CONF_G5_STILL])
        cg.add(ld2410_component.set_g5_still_sensor(sens))
    if CONF_G6_STILL in config:
        sens = await sensor.new_sensor(config[CONF_G6_STILL])
        cg.add(ld2410_component.set_g6_still_sensor(sens))
    if CONF_G7_STILL in config:
        sens = await sensor.new_sensor(config[CONF_G7_STILL])
        cg.add(ld2410_component.set_g7_still_sensor(sens))
    if CONF_G8_STILL in config:
        sens = await sensor.new_sensor(config[CONF_G8_STILL])
        cg.add(ld2410_component.set_g8_still_sensor(sens))
