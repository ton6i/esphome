#include "ld2410.h"
#include "esphome/core/application.h"

#define highbyte(val) (uint8_t)((val) >> 8)
#define lowbyte(val) (uint8_t)((val) &0xff)

namespace esphome {
namespace ld2410_ts {

static const char *const TAG = "ld2410_ts";
int32_t last_periodic_millis = millis();
uint32_t radar_uart_timeout = 5000;								  //How long to give up on receiving some useful data from the LD2410
uint32_t radar_uart_last_packet_ = 0;							  //Time of the last packet from the radar
uint32_t radar_uart_last_command_ = 0;							//Time of the last command sent to the radar
uint32_t radar_uart_command_timeout_ = 500;				//Timeout for sending commands
uint8_t latest_ack_ = 0;
bool latest_command_success_ = false;
int two_byte_to_int_(char firstbyte, char secondbyte) { return (int16_t)(secondbyte << 8) + firstbyte; }
int32_t test = 0;

std::vector<uint8_t> bytes;
std::vector<uint8_t> header_data =       {0xF4, 0xF3, 0xF2, 0xF1, 0x0D, 0x00, 0x02, 0xAA};
std::vector<uint8_t> header_data_engi =  {0xF4, 0xF3, 0xF2, 0xF1, 0x23, 0x00, 0x01, 0xAA};

float LD2410Component::get_setup_priority() const { return esphome::setup_priority::BUS; }

void LD2410Component::setup() {
  ESP_LOGCONFIG(TAG, "Config LD2410");
  set_update_interval(15000);
  if (this->enter_configuration_mode_()) {}     //Using this construction for waiting responce telegram.
  if (this->setMaxValues(this->max_move_distance_, this->max_still_distance_, this->noneduration_)) {}
  if (this->setGateSensitivityThreshold(0, this->rg0_move_threshold_, this->rg0_still_threshold_)) {}
  if (this->setGateSensitivityThreshold(1, this->rg1_move_threshold_, this->rg1_still_threshold_)) {}
  if (this->setGateSensitivityThreshold(2, this->rg2_move_threshold_, this->rg2_still_threshold_)) {}
  if (this->setGateSensitivityThreshold(3, this->rg3_move_threshold_, this->rg3_still_threshold_)) {}
  if (this->setGateSensitivityThreshold(4, this->rg4_move_threshold_, this->rg4_still_threshold_)) {}
  if (this->setGateSensitivityThreshold(5, this->rg5_move_threshold_, this->rg5_still_threshold_)) {}
  if (this->setGateSensitivityThreshold(6, this->rg6_move_threshold_, this->rg6_still_threshold_)) {}
  if (this->setGateSensitivityThreshold(7, this->rg7_move_threshold_, this->rg7_still_threshold_)) {}
  if (this->setGateSensitivityThreshold(8, this->rg8_move_threshold_, this->rg8_still_threshold_)) {}
  if (engineeringmode_)
    this->requestStartEngineeringMode();
  else
    this->requestEndEngineeringMode();
  if (this->leave_configuration_mode_()) {
    if (this->info_ != nullptr)
      this->info_->publish_state("Config done");        
  }
  ESP_LOGI("get_baud_rate", "<<< %u %s", this->parent_->get_baud_rate(), to_string(this->parent_->get_baud_rate()).c_str() );
  this->baud_rate = to_string(this->parent_->get_baud_rate()).c_str();
}

void LD2410Component::dump_config() {
  ESP_LOGCONFIG(TAG, "LD2410:dump_config");
  
  #ifdef USE_BINARY_SENSOR
    LOG_BINARY_SENSOR("  ", "Isconnected", is_connected_);
    LOG_BINARY_SENSOR("  ", "HasTargetSensor", target_binary_sensor_);
    LOG_BINARY_SENSOR("  ", "MovingSensor", moving_binary_sensor_);
    LOG_BINARY_SENSOR("  ", "StillSensor", still_binary_sensor_);
  #endif
  #ifdef USE_SENSOR
    LOG_SENSOR("  ", "Moving Distance", moving_target_distance_sensor_);
    LOG_SENSOR("  ", "Still Distance", still_target_distance_sensor_);
    LOG_SENSOR("  ", "Moving Energy", moving_target_energy_sensor_);
    LOG_SENSOR("  ", "Still Energy", still_target_energy_sensor_);
    LOG_SENSOR("  ", "Detection Distance", detection_distance_sensor_);
  
    LOG_SENSOR("  ", "G0 move", g0_move_energy_sensor_);
    LOG_SENSOR("  ", "G1 move", g1_move_energy_sensor_);
    LOG_SENSOR("  ", "G2 move", g2_move_energy_sensor_);
    LOG_SENSOR("  ", "G3 move", g3_move_energy_sensor_);
    LOG_SENSOR("  ", "G4 move", g4_move_energy_sensor_);
    LOG_SENSOR("  ", "G5 move", g5_move_energy_sensor_);
    LOG_SENSOR("  ", "G6 move", g6_move_energy_sensor_);
    LOG_SENSOR("  ", "G7 move", g7_move_energy_sensor_);
    LOG_SENSOR("  ", "G8 move", g8_move_energy_sensor_);
  
    LOG_SENSOR("  ", "G0 still", g0_still_energy_sensor_);
    LOG_SENSOR("  ", "G1 still", g1_still_energy_sensor_);
    LOG_SENSOR("  ", "G2 still", g2_still_energy_sensor_);
    LOG_SENSOR("  ", "G3 still", g3_still_energy_sensor_);
    LOG_SENSOR("  ", "G4 still", g4_still_energy_sensor_);
    LOG_SENSOR("  ", "G5 still", g5_still_energy_sensor_);
    LOG_SENSOR("  ", "G6 still", g6_still_energy_sensor_);
    LOG_SENSOR("  ", "G7 still", g7_still_energy_sensor_);
    LOG_SENSOR("  ", "G8 still", g8_still_energy_sensor_);
  #endif
 
  if (this->enter_configuration_mode_()) {}
  if (this->requestFirmwareVersion()) {}
  if (this->requestCurrentConfiguration()) {}
  if (this->leave_configuration_mode_()) {}
}

void LD2410Component::update() {}

void LD2410Component::loop() {
  while (available()) {
    this->read_frame_();
  }
  this->publish_binary_sensor_timed(this->is_connected_, (millis() - radar_uart_last_packet_ < radar_uart_timeout), &mydata.connected);
}

bool LD2410Component::HeaderMatched_(std::vector<uint8_t> bytes, std::vector<uint8_t> header) {
  if (bytes[0] != header[0] || bytes[1] != header[1] || bytes[2] != header[2] || bytes[3] != header[3] || bytes[4] != header[4] || bytes[5] != header[5] || bytes[6] != header[6] || bytes[7] != header[7])
      return false;
  return true;
}

void LD2410Component::publish_binary_sensor_timed(binary_sensor::BinarySensor *thesensor, bool value, struct binsensorinfo *binsensorinfo) {
  if (thesensor != nullptr) {
    if (binsensorinfo->last_seen == UINT32_MAX) {  //compare with UINT32_MAX to update sensor at first cycle
      binsensorinfo->last_seen = millis();
      thesensor->publish_state(value);
    } else {
      if (value != binsensorinfo->lastval) {
        if (value == true) {
          if (binsensorinfo->counter.value++ >= binsensorinfo->counter.endval) {
            ESP_LOGD("publish_binary_sensor_timed", "%s write value:%u lastval:%u", binsensorinfo->name.c_str(), value, binsensorinfo->lastval);
            binsensorinfo->counter.value = 0;
            binsensorinfo->lastval = value;
            thesensor->publish_state(value);
          }
        } else {
          if ((millis() - binsensorinfo->last_seen) > binsensorinfo->mytime.toff) {
            ESP_LOGD("publish_binary_sensor_timed", "%s write value:%u lastval:%u", binsensorinfo->name.c_str(), value, binsensorinfo->lastval);
            binsensorinfo->last_seen = millis();
            binsensorinfo->lastval = value;
            thesensor->publish_state(value);
          }
        }
      } else {
        binsensorinfo->counter.value = 0;
        binsensorinfo->last_seen = millis();
      }
    }
  }
}

void LD2410Component::publish_sensor_filter(sensor::Sensor *thesensor, int value, struct sensorinfo *sensorinfo) {
  if (thesensor != nullptr) {
    if (sensorinfo->lastval == UINT16_MAX) {  //compare with UINT16_MAX to update sensor at first cycle
      thesensor->publish_state(value);
      sensorinfo->lastval = value;
      sensorinfo->last_seen = millis();
    } else {
       if (value != sensorinfo->lastval) {
        if (sensorinfo->counter.value++ >= sensorinfo->counter.endval) {
          ESP_LOGD("publish_sensor_filter", "%s write value:%u lastval:%u. counter.value %u", sensorinfo->name.c_str(), value, sensorinfo->lastval, sensorinfo->counter.value);
//          if (0 <= value && value <= sensorinfo->maxval)
          thesensor->publish_state(value);
          sensorinfo->lastval = value;
          sensorinfo->counter.value = 0;
          sensorinfo->last_seen_done = false;
          sensorinfo->last_seen = millis();
        }
       } else if (sensorinfo->last_seen_done == false and (millis() - sensorinfo->last_seen) > 60000) {
          ESP_LOGD("publish_sensor_filter", "%s write value 0", sensorinfo->name.c_str());
          thesensor->publish_state(0);
          sensorinfo->last_seen_done = true;
       } else {
        sensorinfo->counter.value = 0;
      }
    }
  }
}

void LD2410Component::print_frame_() {
  std::string res = "";
  char buf[256];
  for (size_t i = 0; i < bytes.size(); i++) {
      if (i > 0) {
          res += ":";
      }
      sprintf(buf, "%02X", bytes[i]);
      res += buf;
  }
//  ESP_LOGDTAG, "bytes.size() lastval:%u, max_size:%u", bytes.size(), bytes.max_size());
  ESP_LOGI(TAG, "<<< %s", res.c_str());
}

bool LD2410Component::read_frame_() {
  if(available() > 0) {
    bytes.push_back(read());
    if (bytes.size() < 10) 
      return false;
    if (this->HeaderMatched_(bytes, header_data) || this->HeaderMatched_(bytes, header_data_engi) ) { //header_dataframe
      if (bytes.size() < 10 + bytes[4])   //Check length
        return false;
      radar_uart_last_packet_ = millis();
      this->updatenormalsensors_(bytes[DATA_TYPES] == 0x01);
//      print_frame_();
      bytes.clear();
    } else if (bytes[0] == 0xFD && bytes[1] == 0xFC && bytes[2] == 0xFB && bytes[3] == 0xFA) {  //header_commandframe
        if (bytes.size() < 10 + bytes[4])   //Check length
          return false;
        radar_uart_last_packet_ = millis();
        this->parse_command_frame_();
        bytes.clear();
        return true;
    }
    else {
        bytes.erase(bytes.begin());
    }
    this->publish_binary_sensor_timed(this->is_connected_, (millis() - radar_uart_last_packet_ < radar_uart_timeout), &mydata.connected);
  }
  return false;
}

void LD2410Component::updatenormalsensors_(bool alsoupdateengenering) {
  char target_state = bytes[TARGET_STATES];

  // if ((target_state == 0x01) || (target_state == 0x02) || (target_state == 0x03)) {
    // print_frame_();
  // }
  
  this->publish_binary_sensor_timed(this->target_binary_sensor_, (target_state == 0x01) || (target_state == 0x02) || (target_state == 0x03), &mydata.target);

  int32_t current_millis = millis();      //    Reduce data update rate to prevent home assistant database size glow fast
  if ((current_millis - last_periodic_millis) < 1000)
    return;
  last_periodic_millis = current_millis;
  
  this->publish_binary_sensor_timed(this->moving_binary_sensor_, (target_state == 0x01) || (target_state == 0x03), &mydata.mov);
  this->publish_binary_sensor_timed(this->still_binary_sensor_, (target_state == 0x02) || (target_state == 0x03), &mydata.still);

  this->publish_sensor_filter(this->moving_target_distance_sensor_, two_byte_to_int_(bytes[MOVING_TARGET_LOW], bytes[MOVING_TARGET_HIGH]), &mydata.mov_distance);
  this->publish_sensor_filter(this->still_target_distance_sensor_, two_byte_to_int_(bytes[STILL_TARGET_LOW], bytes[STILL_TARGET_HIGH]), &mydata.occ_distance);
  this->publish_sensor_filter(this->detection_distance_sensor_, two_byte_to_int_(bytes[DETECT_DISTANCE_LOW], bytes[DETECT_DISTANCE_HIGH]), &mydata.detect_distance);
  this->publish_sensor_filter(this->moving_target_energy_sensor_, bytes[MOVING_ENERGY], &mydata.mov_energy);
  this->publish_sensor_filter(this->still_target_energy_sensor_, bytes[STILL_ENERGY], &mydata.occ_energy);
  
  if (alsoupdateengenering) {
    this->publish_sensor_filter(this->g0_move_energy_sensor_, bytes[G0_MOVE], &mydata.en_mov_0);
    this->publish_sensor_filter(this->g1_move_energy_sensor_, bytes[G1_MOVE], &mydata.en_mov_1);
    this->publish_sensor_filter(this->g2_move_energy_sensor_, bytes[G2_MOVE], &mydata.en_mov_2);
    this->publish_sensor_filter(this->g3_move_energy_sensor_, bytes[G3_MOVE], &mydata.en_mov_3);
    this->publish_sensor_filter(this->g4_move_energy_sensor_, bytes[G4_MOVE], &mydata.en_mov_4);
    this->publish_sensor_filter(this->g5_move_energy_sensor_, bytes[G5_MOVE], &mydata.en_mov_5);
    this->publish_sensor_filter(this->g6_move_energy_sensor_, bytes[G6_MOVE], &mydata.en_mov_6);
    this->publish_sensor_filter(this->g7_move_energy_sensor_, bytes[G7_MOVE], &mydata.en_mov_7);
    this->publish_sensor_filter(this->g8_move_energy_sensor_, bytes[G8_MOVE], &mydata.en_mov_8);

    this->publish_sensor_filter(this->g0_still_energy_sensor_, bytes[G0_STILL], &mydata.en_occ_0);
    this->publish_sensor_filter(this->g1_still_energy_sensor_, bytes[G1_STILL], &mydata.en_occ_1);
    this->publish_sensor_filter(this->g2_still_energy_sensor_, bytes[G2_STILL], &mydata.en_occ_2);
    this->publish_sensor_filter(this->g3_still_energy_sensor_, bytes[G3_STILL], &mydata.en_occ_3);
    this->publish_sensor_filter(this->g4_still_energy_sensor_, bytes[G4_STILL], &mydata.en_occ_4);
    this->publish_sensor_filter(this->g5_still_energy_sensor_, bytes[G5_STILL], &mydata.en_occ_5);
    this->publish_sensor_filter(this->g6_still_energy_sensor_, bytes[G6_STILL], &mydata.en_occ_6);
    this->publish_sensor_filter(this->g7_still_energy_sensor_, bytes[G7_STILL], &mydata.en_occ_7);
    this->publish_sensor_filter(this->g8_still_energy_sensor_, bytes[G8_STILL], &mydata.en_occ_8);
  }
}

void LD2410Component::parse_command_frame_() {
  latest_ack_ = bytes[6];
  latest_command_success_ = bytes[7] == 0x01;

  switch (bytes[6]) {
    case 0xFF: //Enable Configuration Command
      if (bytes[7] == 0x01) ESP_LOGI(TAG, "Enable Configuration Command successed.");
      else ESP_LOGI(TAG, "!!!Enable Configuration Command Failed!!!");
      break;
    case 0xFE: //End Configuration Command
      if (bytes[7] == 0x01) ESP_LOGI(TAG, "End Configuration Command successed.");
      else ESP_LOGI(TAG, "!!!End Configuration Command Failed!!!");
      break;
    case 0x60: //Maximum Distance Gate and Unmanned Duration Parameter
      if (bytes[7] == 0x01) ESP_LOGI(TAG, "Maximum Distance Gate and Unmanned Duration Parameter successed.");
      else ESP_LOGI(TAG, "!!!Maximum Distance Gate and Unmanned Duration Parameter Failed!!!");
      break;
    case 0x61: //Read Parameter
      if (bytes[7] == 0x01) {
        ESP_LOGI(TAG, "Read Parameter: max_gate = %u", bytes[11]);
        ESP_LOGI(TAG, "Read Parameter: max_move_distance = %u", bytes[12]);
        ESP_LOGI(TAG, "Read Parameter: max_still_distance = %u", bytes[13]);
        ESP_LOGI(TAG, "Read Parameter: motion_sensitivity 0 = %u", bytes[14]);
        ESP_LOGI(TAG, "Read Parameter: motion_sensitivity 1 = %u", bytes[15]);
        ESP_LOGI(TAG, "Read Parameter: motion_sensitivity 2 = %u", bytes[16]);
        ESP_LOGI(TAG, "Read Parameter: motion_sensitivity 3 = %u", bytes[17]);
        ESP_LOGI(TAG, "Read Parameter: motion_sensitivity 4 = %u", bytes[18]);
        ESP_LOGI(TAG, "Read Parameter: motion_sensitivity 5 = %u", bytes[19]);
        ESP_LOGI(TAG, "Read Parameter: motion_sensitivity 6 = %u", bytes[20]);
        ESP_LOGI(TAG, "Read Parameter: motion_sensitivity 7 = %u", bytes[21]);
        ESP_LOGI(TAG, "Read Parameter: motion_sensitivity 8 = %u", bytes[22]);
        ESP_LOGI(TAG, "Read Parameter: stationary_sensitivity 0 = %u", bytes[23]);
        ESP_LOGI(TAG, "Read Parameter: stationary_sensitivity 1 = %u", bytes[24]);
        ESP_LOGI(TAG, "Read Parameter: stationary_sensitivity 2 = %u", bytes[25]);
        ESP_LOGI(TAG, "Read Parameter: stationary_sensitivity 3 = %u", bytes[26]);
        ESP_LOGI(TAG, "Read Parameter: stationary_sensitivity 4 = %u", bytes[27]);
        ESP_LOGI(TAG, "Read Parameter: stationary_sensitivity 5 = %u", bytes[28]);
        ESP_LOGI(TAG, "Read Parameter: stationary_sensitivity 6 = %u", bytes[29]);
        ESP_LOGI(TAG, "Read Parameter: stationary_sensitivity 7 = %u", bytes[30]);
        ESP_LOGI(TAG, "Read Parameter: stationary_sensitivity 8 = %u", bytes[31]);
        ESP_LOGI(TAG, "Read Parameter: none_duration = %u", bytes[32] + (bytes[33] << 8));
      }
      else ESP_LOGI(TAG, "!!!Read Parameter Failed!!!");
      break;
    case 0x62: //Enable Engineering Mode
      if (bytes[7] == 0x01) ESP_LOGI(TAG, "Enable Engineering Mode successed.");
      else ESP_LOGI(TAG, "!!!Enable Engineering Mode Failed!!!");
      break;
    case 0x63: //Close Engineering Mode
      if (bytes[7] == 0x01) ESP_LOGI(TAG, "Close Engineering Mode successed.");
      else ESP_LOGI(TAG, "!!!Close Engineering Mode Failed!!!");
      break;
    case 0x64: //Range Gate Sensitivity Configuration
      if (bytes[7] == 0x01) ESP_LOGI(TAG, "Range Gate Sensitivity Configuration successed.");
      else ESP_LOGI(TAG, "!!!Range Gate Sensitivity Configuration Failed!!!");
      break;
    case 0xA0: //Read Firmware Version
      if (bytes[7] == 0x01) {
        ESP_LOGI(TAG, "Read Firmware Version successed.");
        uint8_t firmware_major_version = bytes[13];								//Reported major version
        uint8_t firmware_minor_version = bytes[12];								//Reported minor version
        uint32_t firmware_bugfix_version = bytes[14] + (bytes[15]<<8) + (bytes[16]<<16) + (bytes[17]<<24);							//Reported bugfix version (coded as hex)
        ESP_LOGI(TAG, "Firmware Version : v.%u.%u.%u", firmware_major_version, firmware_minor_version, firmware_bugfix_version);
      }
      else ESP_LOGI(TAG, "!!!Read Firmware Version Failed!!!");
      break;
    case 0xA1: //Set Serial Port Baud Rate
      if (bytes[7] == 0x01) ESP_LOGI(TAG, "Set Serial Port Baud Rate successed.");
      else ESP_LOGI(TAG, "!!!Set Serial Port Baud Rate Failed!!!");
      break;
    case 0xA2: //Factory Reset
      if (bytes[7] == 0x01) ESP_LOGI(TAG, "Factory Reset successed.");
      else ESP_LOGI(TAG, "!!!Factory Reset Failed!!!");
      break;
    case 0xA3: //Restart the Module
      if (bytes[7] == 0x01) ESP_LOGI(TAG, "Restart the Module successed.");
      else ESP_LOGI(TAG, "!!!Restart the Module Failed!!!");
      break;
  }
}

bool LD2410Component::enter_configuration_mode_() {
  uint8_t command[14] = {0xFD, 0xFC, 0xFB, 0xFA,
                          0x04, 0x00,     //Command is 4 bytes long
                          0xFF, 0x00,     //command
                          0x01, 0x00,
                          0x04, 0x03, 0x02, 0x01};
  for (int i = 0; i < 2; i+= 1) {
    ESP_LOGD(TAG, "enter_configuration_mode_ %i", i);
    this->write_array(command, 14);
    radar_uart_last_command_ = millis();
    while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_) {
      if(this->read_frame_()) {
        if(latest_ack_ == 0xFF && latest_command_success_) {
          return true;
        }
      }
    }
  }
  ESP_LOGE(TAG, "enter_configuration_mode_ failed");
  return false;
}

bool LD2410Component::leave_configuration_mode_() {
  uint8_t command[12] = {0xFD, 0xFC, 0xFB, 0xFA,
                          0x02, 0x00,     //Command is 2 bytes long
                          0xFE, 0x00,     //command
                          0x04, 0x03, 0x02, 0x01};
  for (int i = 0; i < 2; i+= 1) {
    ESP_LOGD(TAG, "leave_configuration_mode_ %i", i);
    this->write_array(command, 12);
    radar_uart_last_command_ = millis();
    while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_) {
      if(this->read_frame_()) {
        if(latest_ack_ == 0xFE && latest_command_success_)
          return true;
      }
    }
  }
  ESP_LOGE(TAG, "leave_configuration_mode_ failed");
  return false;
}

bool LD2410Component::requestStartEngineeringMode() {
  uint8_t command[12] = {0xFD, 0xFC, 0xFB, 0xFA,
                          0x02, 0x00,     //Command is 2 bytes long
                          0x62, 0x00,     //command
                          0x04, 0x03, 0x02, 0x01};
  for (int i = 0; i < 2; i+= 1) {
    ESP_LOGD(TAG, "requestStartEngineeringMode %i", i);
    this->write_array(command, 12);
    radar_uart_last_command_ = millis();
    while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_) {
      if(this->read_frame_()) {
        if(latest_ack_ == 0x62 && latest_command_success_)
          return true;
      }
    }
  }
  ESP_LOGE(TAG, "requestStartEngineeringMode failed");
  return false;
}

bool LD2410Component::requestEndEngineeringMode() {
  uint8_t command[12] = {0xFD, 0xFC, 0xFB, 0xFA,
                          0x02, 0x00,     //Command is 2 bytes long
                          0x63, 0x00,     //command
                          0x04, 0x03, 0x02, 0x01};
  for (int i = 0; i < 2; i+= 1) {
    ESP_LOGD(TAG, "requestEndEngineeringMode %i", i);
    this->write_array(command, 12);
    radar_uart_last_command_ = millis();
    while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_) {
      if(this->read_frame_()) {
        if(latest_ack_ == 0x63 && latest_command_success_)
          return true;
      }
    }
  }
  ESP_LOGE(TAG, "requestEndEngineeringMode failed");
  return false;
}

bool LD2410Component::requestCurrentConfiguration() {
  uint8_t command[12] = {0xFD, 0xFC, 0xFB, 0xFA,
                          0x02, 0x00,     //Command is 2 bytes long
                          0x61, 0x00,     //command
                          0x04, 0x03, 0x02, 0x01};
  for (int i = 0; i < 2; i+= 1) {
    ESP_LOGD(TAG, "requestCurrentConfiguration %i", i);
    this->write_array(command, 12);
    radar_uart_last_command_ = millis();
    while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_) {
      if(this->read_frame_()) {
        if(latest_ack_ == 0x61 && latest_command_success_)
          return true;
      }
    }
  }
  ESP_LOGE(TAG, "requestCurrentConfiguration failed");
  return false;
}

bool LD2410Component::requestFirmwareVersion() {
  uint8_t command[12] = {0xFD, 0xFC, 0xFB, 0xFA,
                          0x02, 0x00,     //Command is 2 bytes long
                          0xA0, 0x00,     //command
                          0x04, 0x03, 0x02, 0x01};
  for (int i = 0; i < 2; i+= 1) {
    ESP_LOGD(TAG, "requestFirmwareVersion %i", i);
    this->write_array(command, 12);
    radar_uart_last_command_ = millis();
    while((millis() - radar_uart_last_command_) < radar_uart_command_timeout_) {
      if(this->read_frame_()) {
        if(latest_ack_ == 0xA0 && latest_command_success_)
          return true;
      }
    }
  }
  ESP_LOGE(TAG, "requestFirmwareVersion failed");
  return false;
}

bool LD2410Component::requestRestart() {
  uint8_t command[12] = {0xFD, 0xFC, 0xFB, 0xFA,
                          0x02, 0x00,     //Command is 2 bytes long
                          0xA3, 0x00,     //command
                          0x04, 0x03, 0x02, 0x01};
  for (int i = 0; i < 2; i+= 1) {
    ESP_LOGD(TAG, "requestRestart %i", i);
    this->write_array(command, 12);
    radar_uart_last_command_ = millis();
    while((millis() - radar_uart_last_command_) < radar_uart_command_timeout_) {
      if(this->read_frame_()) {
        if(latest_ack_ == 0xA3 && latest_command_success_)
          return true;
      }
    }
  }
  ESP_LOGE(TAG, "requestRestart failed");
  return false;
}

bool LD2410Component::requestFactoryReset() {
  uint8_t command[12] = {0xFD, 0xFC, 0xFB, 0xFA,
                          0x02, 0x00,     //Command is 2 bytes long
                          0xA2, 0x00,     //command
                          0x04, 0x03, 0x02, 0x01};
  for (int i = 0; i < 2; i+= 1) {
    ESP_LOGD(TAG, "requestFactoryReset %i", i);
    this->write_array(command, 12);
    radar_uart_last_command_ = millis();
    while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_) {
      if(this->read_frame_()) {
        if(latest_ack_ == 0xA2 && latest_command_success_)
          return true;
      }
    }
  }
  ESP_LOGE(TAG, "requestFactoryReset failed");
  return false;
}

bool LD2410Component::setMaxValues(uint16_t moving, uint16_t stationary, uint16_t inactivityTimer) {
  uint8_t command[30] = {0xFD, 0xFC, 0xFB, 0xFA,
                      0x14, 0x00,     //Command is 20 bytes long
                      0x60, 0x00,     //command
                      0x00, 0x00, lowbyte(moving), highbyte(moving),
                      0x00, 0x00,     //Spacer
                      0x01, 0x00, lowbyte(stationary), highbyte(stationary),
                      0x00, 0x00,     //Spacer
                      0x02, 0x00, lowbyte(inactivityTimer), highbyte(inactivityTimer),
                      0x00, 0x00,     //Spacer
                      0x04, 0x03, 0x02, 0x01};

  for (int i = 0; i < 2; i+= 1) {
    ESP_LOGD(TAG, "setMaxValues %i", i);
    this->write_array(command, 30);
    radar_uart_last_command_ = millis();
    while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_) {
      if(this->read_frame_()) {
        if(latest_ack_ == 0x60 && latest_command_success_)
          return true;
      }
    }
  }
  ESP_LOGE(TAG, "setMaxValues failed, moving:%i, stationary:%i, inactivityTimer:%i.", moving, stationary, inactivityTimer);
  return false;
}

bool LD2410Component::setGateSensitivityThreshold(uint8_t gate, uint8_t moving, uint8_t stationary) {
  uint8_t command[30] = {0xFD, 0xFC, 0xFB, 0xFA,
                      0x14, 0x00,     //Command is 20 bytes long
                      0x64, 0x00,     //command
                      0x00, 0x00, gate, 0x00,
                      0x00, 0x00,     //Spacer
                      0x01, 0x00, moving, 0x00,
                      0x00, 0x00,     //Spacer
                      0x02, 0x00, stationary, 0x00,
                      0x00, 0x00,     //Spacer
                      0x04, 0x03, 0x02, 0x01};
  for (int i = 0; i < 2; i+= 1) {
    ESP_LOGD(TAG, "setGateSensitivityThreshold %i", i);
    this->write_array(command, 30);
    radar_uart_last_command_ = millis();
    while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_) {
      if(this->read_frame_()) {
        if(latest_ack_ == 0x64 && latest_command_success_)
          return true;
      }
    }
  }
  ESP_LOGE(TAG, "setGateSensitivityThreshold failed, gate:%i, moving:%i, stationary:%i.", gate, moving, stationary);
  return false;
}

bool LD2410Component::setbaudrate(uint8_t baudrate) {
  uint8_t command[14] = {0xFD, 0xFC, 0xFB, 0xFA,
                          0x04, 0x00,     //Command is 4 bytes long
                          0xA1, 0x00,     //command
                          baudrate, 0x00,
                          0x04, 0x03, 0x02, 0x01};
  for (int i = 0; i < 2; i+= 1) {
    ESP_LOGD(TAG, "setbaudrate %i", i);
    this->write_array(command, 14);
    radar_uart_last_command_ = millis();
    while(millis() - radar_uart_last_command_ < radar_uart_command_timeout_) {
      if(this->read_frame_()) {
        if(latest_ack_ == 0xA1 && latest_command_success_)
          return true;
      }
    }
  }
  ESP_LOGE(TAG, "setbaudrate failed");
  return false;
}

void Buttons::dump_config() { LOG_BUTTON(TAG, "Buttons Button", this);}
void Buttons::press_action() {
  ESP_LOGD(TAG, "press_action %s, type_ %i", name_.c_str(), type_);
  switch (type_) {
    case eButtonType::restart_device:
      App.safe_reboot();
      break;
    case eButtonType::restart_module:
      if (this->parent_->enter_configuration_mode_()) {}
      if (this->parent_->requestRestart()){}
      if (this->parent_->leave_configuration_mode_()) {}
      break;
    case eButtonType::factor_reset:
      if (this->parent_->enter_configuration_mode_()) {}
      if (this->parent_->requestFactoryReset()){}
      if (this->parent_->leave_configuration_mode_()) {}
      break;
  }
}
      
void SelectBaudrate::setup() {
  this->traits.set_options({"9600", "19200", "38400", "57600", "115200", "230400", "256000", "460800"});
  this->publish_state(this->parent_->baud_rate.c_str()); 
}
void SelectBaudrate::dump_config() {LOG_SELECT(TAG, "DemoSelect Select", this);}
void SelectBaudrate::control(const std::string &value) {
  this->publish_state(value);

  auto index = this->index_of(value);
  ESP_LOGD(TAG, "Sending state %s (index %d)", state.c_str(), index.value());

  if (value == "9600") {
    ESP_LOGI(TAG, "Baudrate 9600 selected");
    if (this->parent_->enter_configuration_mode_()) {}
    if (this->parent_->setbaudrate(1)) {}
    if (this->parent_->leave_configuration_mode_()) {}
  } else if (value == "19200"){
    ESP_LOGI(TAG, "Baudrate 19200 selected");
    if (this->parent_->enter_configuration_mode_()) {}
    if (this->parent_->setbaudrate(2)) {}
    if (this->parent_->leave_configuration_mode_()) {}
  } else if (value == "38400") {
    ESP_LOGI(TAG, "Baudrate 38400 selected");
    if (this->parent_->enter_configuration_mode_()) {}
    if (this->parent_->setbaudrate(3)) {}
    if (this->parent_->leave_configuration_mode_()) {}
  } else if (value == "57600") {
    ESP_LOGI(TAG, "Baudrate 57600 selected");
    if (this->parent_->enter_configuration_mode_()) {}
    if (this->parent_->setbaudrate(4)) {}
    if (this->parent_->leave_configuration_mode_()) {}
  } else if (value == "115200") {
    ESP_LOGI(TAG, "Baudrate 115200 selected");
    if (this->parent_->enter_configuration_mode_()) {}
    if (this->parent_->setbaudrate(5)) {}
    if (this->parent_->leave_configuration_mode_()) {}
  } else if (value == "230400") {
    ESP_LOGI(TAG, "Baudrate 230400 selected");
    if (this->parent_->enter_configuration_mode_()) {}
    if (this->parent_->setbaudrate(6)) {}
    if (this->parent_->leave_configuration_mode_()) {}
  } else if (value == "256000") {
    ESP_LOGI(TAG, "Baudrate 256000 selected");
    if (this->parent_->enter_configuration_mode_()) {}
    if (this->parent_->setbaudrate(7)) {}
    if (this->parent_->leave_configuration_mode_()) {}
  } else if (value == "460800") {
    ESP_LOGI(TAG, "Baudrate 460800 selected");
    if (this->parent_->enter_configuration_mode_()) {}
    if (this->parent_->setbaudrate(8)) {}
    if (this->parent_->leave_configuration_mode_()) {}
  }
}

}  // namespace ld2410
}  // namespace esphome
