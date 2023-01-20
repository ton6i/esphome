#pragma once
#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"

namespace esphome {
namespace ld2410_ts {

  enum class eButtonType {
    restart_device,
    restart_module,
    factor_reset,
  };

  enum PeriodicDataStructure : uint8_t {
    DATA_TYPES = 6,
    TARGET_STATES = 8,
    MOVING_TARGET_LOW = 9,
    MOVING_TARGET_HIGH = 10,
    MOVING_ENERGY = 11,
    STILL_TARGET_LOW = 12,
    STILL_TARGET_HIGH = 13,
    STILL_ENERGY = 14,
    DETECT_DISTANCE_LOW = 15,
    DETECT_DISTANCE_HIGH = 16,
    G0_MOVE = 17,
    G1_MOVE = 18,
    G2_MOVE = 19,
    G3_MOVE = 20,
    G4_MOVE = 21,
    G5_MOVE = 22,
    G6_MOVE = 23,
    G7_MOVE = 24,
    G8_MOVE = 25,
    G0_STILL = 26,
    G1_STILL = 27,
    G2_STILL = 28,
    G3_STILL = 29,
    G4_STILL = 30,
    G5_STILL = 31,
    G6_STILL = 32,
    G7_STILL = 33,
    G8_STILL = 34,
  };

  typedef struct timeinfo {
    uint32_t ton;
    uint32_t toff;
  } timeinfo;

  typedef struct counterinfo {
    uint16_t value;
    uint16_t endval;
  } counterinfo;

  typedef struct binsensorinfo {
    std::string name;
    bool lastval;
    uint32_t last_seen;
    struct timeinfo mytime;
    struct counterinfo counter;
  } binsensorinfo;

  typedef struct sensorinfo {
    std::string name;
    uint16_t lastval;
    uint16_t maxval;
    uint32_t last_seen;
    bool last_seen_done;
    struct counterinfo counter;
  } sensorinfo;


  struct data {
      struct binsensorinfo connected =    {"connected", false, UINT32_MAX, {100, 500}, {0, 1} };
      struct binsensorinfo target =       {"target"   , false, UINT32_MAX, {100, 500}, {0, 1} };
      struct binsensorinfo mov =          {"mov"      , false, UINT32_MAX, {100, 500}, {0, 1} };
      struct binsensorinfo still =        {"still"    , false, UINT32_MAX, {100, 500}, {0, 1} };
      struct sensorinfo detect_distance = {"detect_dist"  , UINT16_MAX, 10000, UINT32_MAX, false, {0, 1}};
      struct sensorinfo mov_distance =    {"mov_dist"     , UINT16_MAX, 10000, UINT32_MAX, false, {0, 1}};
      struct sensorinfo occ_distance =    {"occ_dist"     , UINT16_MAX, 10000, UINT32_MAX, false, {0, 1}};
      struct sensorinfo mov_energy =      {"mov_ener"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo occ_energy =      {"occ_ener"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_mov_0 =        {"en_mov_0"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_mov_1 =        {"en_mov_1"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_mov_2 =        {"en_mov_2"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_mov_3 =        {"en_mov_3"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_mov_4 =        {"en_mov_4"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_mov_5 =        {"en_mov_5"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_mov_6 =        {"en_mov_6"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_mov_7 =        {"en_mov_7"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_mov_8 =        {"en_mov_8"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_occ_0 =        {"en_occ_0"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_occ_1 =        {"en_occ_1"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_occ_2 =        {"en_occ_2"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_occ_3 =        {"en_occ_3"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_occ_4 =        {"en_occ_4"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_occ_5 =        {"en_occ_5"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_occ_6 =        {"en_occ_6"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_occ_7 =        {"en_occ_7"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
      struct sensorinfo en_occ_8 =        {"en_occ_8"     , UINT16_MAX, 100, UINT32_MAX, false, {0, 1}};
  };


class LD2410Component : public PollingComponent, public uart::UARTDevice {
  public:
    float get_setup_priority() const override;
    void setup() override;
    void update() override;
    void dump_config() override;
    void loop() override;
  
    void set_engineeringmode(bool engineeringmode) { this->engineeringmode_ = engineeringmode; };
    void set_info(text_sensor::TextSensor *text_sensor) { this->info_ = text_sensor; };
    void set_number_id(number::Number *sens) { this->number_id_ = sens; };
    void set_is_connected(binary_sensor::BinarySensor *sens) { this->is_connected_ = sens; };
    void set_target_sensor(binary_sensor::BinarySensor *sens) { this->target_binary_sensor_ = sens; };
    void set_moving_target_sensor(binary_sensor::BinarySensor *sens) { this->moving_binary_sensor_ = sens; };
    void set_still_target_sensor(binary_sensor::BinarySensor *sens) { this->still_binary_sensor_ = sens; };
    void set_moving_distance_sensor(sensor::Sensor *sens) { this->moving_target_distance_sensor_ = sens; };
    void set_still_distance_sensor(sensor::Sensor *sens) { this->still_target_distance_sensor_ = sens; };
    void set_moving_energy_sensor(sensor::Sensor *sens) { this->moving_target_energy_sensor_ = sens; };
    void set_still_energy_sensor(sensor::Sensor *sens) { this->still_target_energy_sensor_ = sens; };
    void set_g0_move_sensor(sensor::Sensor *sens) { this->g0_move_energy_sensor_ = sens; };
    void set_g1_move_sensor(sensor::Sensor *sens) { this->g1_move_energy_sensor_ = sens; };
    void set_g2_move_sensor(sensor::Sensor *sens) { this->g2_move_energy_sensor_ = sens; };
    void set_g3_move_sensor(sensor::Sensor *sens) { this->g3_move_energy_sensor_ = sens; };
    void set_g4_move_sensor(sensor::Sensor *sens) { this->g4_move_energy_sensor_ = sens; };
    void set_g5_move_sensor(sensor::Sensor *sens) { this->g5_move_energy_sensor_ = sens; };
    void set_g6_move_sensor(sensor::Sensor *sens) { this->g6_move_energy_sensor_ = sens; };
    void set_g7_move_sensor(sensor::Sensor *sens) { this->g7_move_energy_sensor_ = sens; };
    void set_g8_move_sensor(sensor::Sensor *sens) { this->g8_move_energy_sensor_ = sens; };
    void set_g0_still_sensor(sensor::Sensor *sens) { this->g0_still_energy_sensor_ = sens; };
    void set_g1_still_sensor(sensor::Sensor *sens) { this->g1_still_energy_sensor_ = sens; };
    void set_g2_still_sensor(sensor::Sensor *sens) { this->g2_still_energy_sensor_ = sens; };
    void set_g3_still_sensor(sensor::Sensor *sens) { this->g3_still_energy_sensor_ = sens; };
    void set_g4_still_sensor(sensor::Sensor *sens) { this->g4_still_energy_sensor_ = sens; };
    void set_g5_still_sensor(sensor::Sensor *sens) { this->g5_still_energy_sensor_ = sens; };
    void set_g6_still_sensor(sensor::Sensor *sens) { this->g6_still_energy_sensor_ = sens; };
    void set_g7_still_sensor(sensor::Sensor *sens) { this->g7_still_energy_sensor_ = sens; };
    void set_g8_still_sensor(sensor::Sensor *sens) { this->g8_still_energy_sensor_ = sens; };
    void set_detection_distance_sensor(sensor::Sensor *sens) { this->detection_distance_sensor_ = sens; };
    void set_none_duration(int value) { this->noneduration_ = value; };
    void set_max_move_distance(int value) { this->max_move_distance_ = value; };
    void set_max_still_distance(int value) { this->max_still_distance_ = value; };
    void set_range_config(int rg0_move, int rg0_still, int rg1_move, int rg1_still, int rg2_move, int rg2_still,
                          int rg3_move, int rg3_still, int rg4_move, int rg4_still, int rg5_move, int rg5_still,
                          int rg6_move, int rg6_still, int rg7_move, int rg7_still, int rg8_move, int rg8_still) {
      this->rg0_move_threshold_ = rg0_move;
      this->rg0_still_threshold_ = rg0_still;
      this->rg1_move_threshold_ = rg1_move;
      this->rg1_still_threshold_ = rg1_still;
      this->rg2_move_threshold_ = rg2_move;
      this->rg2_still_threshold_ = rg2_still;
      this->rg3_move_threshold_ = rg3_move;
      this->rg3_still_threshold_ = rg3_still;
      this->rg4_move_threshold_ = rg4_move;
      this->rg4_still_threshold_ = rg4_still;
      this->rg5_move_threshold_ = rg5_move;
      this->rg5_still_threshold_ = rg5_still;
      this->rg6_move_threshold_ = rg6_move;
      this->rg6_still_threshold_ = rg6_still;
      this->rg7_move_threshold_ = rg7_move;
      this->rg7_still_threshold_ = rg7_still;
      this->rg8_move_threshold_ = rg8_move;
      this->rg8_still_threshold_ = rg8_still;
    };
    bool enter_configuration_mode_();								    //Necessary before sending any command
    bool leave_configuration_mode_();								    //Will not read values without leaving command mode
    bool requestStartEngineeringMode();
    bool requestEndEngineeringMode();
    bool requestCurrentConfiguration();								//Request current configuration
    bool requestFirmwareVersion();									//Request the firmware version
    bool requestRestart();
    bool requestFactoryReset();
    bool setMaxValues(uint16_t moving, uint16_t stationary, uint16_t inactivityTimer);	//Realistically gate values are 0-8 but sent as uint16_t
    bool setGateSensitivityThreshold(uint8_t gate, uint8_t moving, uint8_t stationary);
    bool setbaudrate(uint8_t baudrate);
  
    struct data mydata;
    text_sensor::TextSensor *info_{nullptr};
    std::string baud_rate = "";
  protected:
  
    number::Number *number_id_{nullptr};
    binary_sensor::BinarySensor *is_connected_{nullptr};
    binary_sensor::BinarySensor *target_binary_sensor_{nullptr};
    binary_sensor::BinarySensor *moving_binary_sensor_{nullptr};
    binary_sensor::BinarySensor *still_binary_sensor_{nullptr};
    sensor::Sensor *moving_target_distance_sensor_{nullptr};
    sensor::Sensor *still_target_distance_sensor_{nullptr};
    sensor::Sensor *moving_target_energy_sensor_{nullptr};
    sensor::Sensor *still_target_energy_sensor_{nullptr};
    sensor::Sensor *g0_move_energy_sensor_{nullptr};
    sensor::Sensor *g1_move_energy_sensor_{nullptr};
    sensor::Sensor *g2_move_energy_sensor_{nullptr};
    sensor::Sensor *g3_move_energy_sensor_{nullptr};
    sensor::Sensor *g4_move_energy_sensor_{nullptr};
    sensor::Sensor *g5_move_energy_sensor_{nullptr};
    sensor::Sensor *g6_move_energy_sensor_{nullptr};
    sensor::Sensor *g7_move_energy_sensor_{nullptr};
    sensor::Sensor *g8_move_energy_sensor_{nullptr};
    sensor::Sensor *g0_still_energy_sensor_{nullptr};
    sensor::Sensor *g1_still_energy_sensor_{nullptr};
    sensor::Sensor *g2_still_energy_sensor_{nullptr};
    sensor::Sensor *g3_still_energy_sensor_{nullptr};
    sensor::Sensor *g4_still_energy_sensor_{nullptr};
    sensor::Sensor *g5_still_energy_sensor_{nullptr};
    sensor::Sensor *g6_still_energy_sensor_{nullptr};
    sensor::Sensor *g7_still_energy_sensor_{nullptr};
    sensor::Sensor *g8_still_energy_sensor_{nullptr};
    sensor::Sensor *detection_distance_sensor_{nullptr};
  
    bool engineeringmode_;
    int noneduration_ = -1;
    int max_move_distance_ = -1;
    int max_still_distance_ = -1;
    uint8_t rg0_move_threshold_, rg0_still_threshold_, rg1_move_threshold_, rg1_still_threshold_, rg2_move_threshold_,
            rg2_still_threshold_, rg3_move_threshold_, rg3_still_threshold_, rg4_move_threshold_, rg4_still_threshold_,
            rg5_move_threshold_, rg5_still_threshold_, rg6_move_threshold_, rg6_still_threshold_, rg7_move_threshold_,
            rg7_still_threshold_, rg8_move_threshold_, rg8_still_threshold_;
  
    bool HeaderMatched_(std::vector<uint8_t> bytes, std::vector<uint8_t> header);
    void print_frame_();												          //
    bool read_frame_();												          //Try to read a frame from the UART
    void parse_command_frame_();
    void updatenormalsensors_(bool alsoupdateengenering);
    void publish_binary_sensor_timed(binary_sensor::BinarySensor *thesensor, bool val, struct binsensorinfo *binsensorinfo);
    void publish_sensor_filter(sensor::Sensor *thesensor, int value, struct sensorinfo *sensorinfo);
};

class Buttons : public Component, public button::Button {
  public:
    void dump_config() override;
    void set_parent(LD2410Component *parent) { this->parent_ = parent; }
    void set_type(eButtonType type) { type_ = type; }
  protected:
    void press_action() override;
    LD2410Component *parent_;
    eButtonType type_;
};

class SelectBaudrate : public select::Select, public Component {
  public:
    void dump_config() override;
    void setup() override;
    void set_parent(LD2410Component *parent) { this->parent_ = parent; }

  protected:
    void control(const std::string &value) override;
    LD2410Component *parent_;
};


}  // namespace ld2410
}  // namespace esphome
