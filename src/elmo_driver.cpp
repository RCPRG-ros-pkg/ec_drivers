/*
 * Copyright (c) 2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rtt/plugin/ServicePlugin.hpp>

#include <ec_hardware/ECDriver.h>
#include <ec_hardware/ECDriverFactory.h>
#include <ec_hardware/ECPDOEntry.h>

const uint16_t SW_ReadyToSwitchOn_Mask = 0x0001;
const uint16_t SW_SwitchedOn_Mask = 0x0002;
const uint16_t SW_OperationEnabled_Mask = 0x0004;
const uint16_t SW_Fault_Mask = 0x0008;
const uint16_t SW_VoltageEnabled_Mask = 0x0010;
const uint16_t SW_QuickStop_Mask = 0x0020;
const uint16_t SW_SwitchOnDisabled_Mask = 0x0040;
const uint16_t SW_Warning_Mask = 0x0080;
const uint16_t SW_ManufactureSpecific_Mask = 0x0100;
const uint16_t SW_Remote_Mask = 0x0200;
const uint16_t SW_TargetReached_Mask = 0x0400;
const uint16_t SW_InternalLimitActive_Mask = 0x0800;
const uint16_t SW_HomingComplete_Mask = 0x1000;

const uint16_t NotReadyToSwitchOn_Mask = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask | SW_SwitchOnDisabled_Mask;
const uint16_t NotReadyToSwitchOn_Pattern = 0x0000;

const uint16_t SwitchOnDisabled_Mask = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask | SW_SwitchOnDisabled_Mask;
const uint16_t SwitchOnDisabled_Pattern = SW_SwitchOnDisabled_Mask;

const uint16_t ReadyToSwitchOn_Mask = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask | SW_QuickStop_Mask | SW_SwitchOnDisabled_Mask;
const uint16_t ReadyToSwitchOn_Pattern = SW_ReadyToSwitchOn_Mask | SW_QuickStop_Mask;

const uint16_t SwitchedOn_Mask = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask | SW_QuickStop_Mask | SW_SwitchOnDisabled_Mask;
const uint16_t SwitchedOn_Pattern = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask | SW_QuickStop_Mask;

const uint16_t OperationEnabled_Mask = SW_ReadyToSwitchOn_Mask  |SW_SwitchedOn_Mask|SW_OperationEnabled_Mask|SW_Fault_Mask|SW_QuickStop_Mask|SW_SwitchOnDisabled_Mask;
const uint16_t OperationEnabled_Pattern = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_QuickStop_Mask;

const uint16_t Fault_Reaction_Mask = SW_ReadyToSwitchOn_Mask|SW_SwitchedOn_Mask|SW_OperationEnabled_Mask|SW_Fault_Mask|SW_SwitchOnDisabled_Mask;
const uint16_t Fault_Reaction_Pattern = SW_Fault_Mask|SW_ReadyToSwitchOn_Mask|SW_SwitchedOn_Mask|SW_OperationEnabled_Mask;

const uint16_t Fault_Mask = SW_ReadyToSwitchOn_Mask|SW_SwitchedOn_Mask|SW_OperationEnabled_Mask|SW_Fault_Mask|SW_SwitchOnDisabled_Mask;
const uint16_t Fault_Pattern = SW_Fault_Mask;

const uint16_t QuickStopActive_Mask = SW_ReadyToSwitchOn_Mask|SW_SwitchedOn_Mask|SW_OperationEnabled_Mask|SW_Fault_Mask|SW_QuickStop_Mask|SW_SwitchOnDisabled_Mask;
const uint16_t QuickStopActive_Pattern = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask;


class ElmoDriver : public ECDriver {
 public:
  enum ControlMode {PROFILE_POSITION = 1, PROFILE_VELOCITY = 2, PROFILE_CURRENT = 3, HOMING = 6, CYCLIC_CURRENT = 10, CYCLIC_VELOCITY = 9, CYCLIC_POSITION = 8};
  enum ServoState {INVALID = 0, NOT_READY_TO_SWITCH_ON = 1, SWITCH_ON_DISABLED = 2, READY_TO_SWITCH_ON = 3, SWITCH_ON = 4, OPERATION_ENABLED = 5, QUICK_STOP_ACTIVE = 6, FAULT_REACTION_ACTIVE = 7, FAULT = 8};

  ElmoDriver(const std::string &name)
      : ECDriver(name),
        statusword_pdo_(0x6041, 0),
        position_pdo_(0x6064, 0),
        velocity_pdo_(0x606C, 0),
        current_pdo_(0x6077, 0),
        controlword_pdo_(0x6040, 0),
        mode_of_operation_pdo_(0x6060, 0),
        position_command_pdo_(0x607A, 0),
        velocity_command_pdo_(0x60FF, 0),
        current_command_pdo_(0x6071, 0),
        control_mode_(HOMING),
        enable_(false),
        homing_(false),
        reset_fault_(false),
        homing_done_(true) {

    this->provides()->addPort("motor_position", motor_position_port_);
    this->provides()->addPort("motor_velocity", motor_velocity_port_);
    this->provides()->addPort("motor_current", motor_current_port_);

    this->provides()->addPort("motor_position_command", motor_position_command_port_);
    this->provides()->addPort("motor_velocity_command", motor_velocity_command_port_);
    this->provides()->addPort("motor_current_command", motor_current_command_port_);


    this->provides()->addAttribute("state", *((int*)&state_));
    this->provides()->addAttribute("homing_done", homing_done_);

    this->provides()->addOperation("beginHoming", &ElmoDriver::beginHoming, this, RTT::OwnThread);
    this->provides()->addOperation("enable", &ElmoDriver::enable, this, RTT::OwnThread);
    this->provides()->addOperation("disable", &ElmoDriver::disable, this, RTT::OwnThread);
    this->provides()->addOperation("resetFault", &ElmoDriver::resetFault, this, RTT::OwnThread);
  }

  ~ElmoDriver() {
  }

  virtual bool configureHook(const YAML::Node &cfg) {
    if (cfg["interpolation_period"]) {
      YAML::Node intp = cfg["interpolation_period"];
      uint8_t value = intp["value"].as<int>();
      int8_t index = intp["index"].as<int>();

      slave_->addSDOConfig(0x60C2, 1, value);
      slave_->addSDOConfig(0x60C2, 2, index);
    } else {
      RTT::log(RTT::Error) << "Driver require parameter interpolation_period" << RTT::endlog();
      return false;
    }

    if (cfg["homing"]) {
      YAML::Node home = cfg["homing"];
      int8_t homing_mode = home["mode"].as<int>();
      if (homing_mode > 0) {
        slave_->addSDOConfig(0x6098, 0, homing_mode);
        uint32_t spdh = home["speed_high"].as<unsigned int>();
        uint32_t spdl = home["speed_low"].as<unsigned int>();
        slave_->addSDOConfig(0x6099, 1, spdh);
        slave_->addSDOConfig(0x6099, 2, spdl);
        uint32_t acc = home["acceleration"].as<unsigned int>();
        slave_->addSDOConfig(0x609A, 0, acc);
        homing_done_ = false;
      }
    }

    if (cfg["control_mode"]) {
      std::string mode_str = cfg["control_mode"].as<std::string>();

      if (mode_str == "position") {
        control_mode_ = CYCLIC_POSITION;
      } else if (mode_str == "velocity") {
        control_mode_ = CYCLIC_VELOCITY;
      } else if (mode_str == "current") {
        control_mode_ = CYCLIC_CURRENT;
      } else {
        return false;
      }
    } else {
      return false;
    }

    this->addPDOEntry(&statusword_pdo_);
    this->addPDOEntry(&position_pdo_);
    this->addPDOEntry(&velocity_pdo_);
    this->addPDOEntry(&current_pdo_);
    this->addPDOEntry(&controlword_pdo_);
    this->addPDOEntry(&mode_of_operation_pdo_);

    if (control_mode_ == CYCLIC_POSITION) {
      this->addPDOEntry(&position_command_pdo_);
    } else if (control_mode_ == CYCLIC_VELOCITY) {
      this->addPDOEntry(&velocity_command_pdo_);
    } else if (control_mode_ == CYCLIC_CURRENT) {
      this->addPDOEntry(&current_command_pdo_);
    } else {
      return false;
    }

    return true;
  }

  virtual void updateInputs() {
    int32_t pos, vel;
    int16_t statusword = 0;

    statusword = statusword_pdo_.read();
    state_ = getState(statusword);

    pos = position_pdo_.read();
    vel = velocity_pdo_.read();

    motor_position_port_.write(pos);
    motor_velocity_port_.write(vel);

    if (homing_) {
      if ((statusword & 0x3400) == 0x1400) {
        homing_ = false;
        homing_done_ = true;
      }
    }

  }

  virtual void updateOutputs() {
    uint16_t cw = 0;
    switch (state_) {
      case NOT_READY_TO_SWITCH_ON:
        enable_ = false;
        cw = 0;
        break;
      case SWITCH_ON_DISABLED:
        enable_ = false;
        cw = 0x06;
        break;
      case READY_TO_SWITCH_ON:
        enable_ = false;
        cw = 0x07;
        break;
      case SWITCH_ON:
        if (enable_) {
          cw = 0x0f;
        } else {
          cw = 0x07;
        }
        break;
      case OPERATION_ENABLED:
          if (enable_) {
            cw = 0x0f;
          } else {
            cw = 0x07;
          }
        break;
      case QUICK_STOP_ACTIVE:
        enable_ = false;
        break;
      case FAULT_REACTION_ACTIVE:
        enable_ = false;
        break;
      case FAULT:
          enable_ = false;
          if (reset_fault_) {
            cw = 0x80;
            reset_fault_ = false;
          } else {
            cw = 0;
          }
        break;
      default:
        break;
    }

    if (homing_done_) {
      mode_of_operation_pdo_.write(control_mode_);
    } else {
      mode_of_operation_pdo_.write(HOMING);
      if (homing_) {
        cw |= 0x10;
      }
    }

    if (state_ != OPERATION_ENABLED || homing_done_ == false) {
      switch (control_mode_) {
        case CYCLIC_CURRENT:
          current_command_pdo_.write(0);
          break;
        case CYCLIC_VELOCITY:
          velocity_command_pdo_.write(0);
          break;
        case CYCLIC_POSITION:
          position_command_pdo_.write(position_pdo_.read());
          break;
      }
    } else {
      switch (control_mode_) {
        case CYCLIC_CURRENT:
          double cur;
          if (motor_current_command_port_.read(cur) == RTT::NewData) {
            current_command_pdo_.write(cur);
          }
          break;
        case CYCLIC_VELOCITY:
          double vel;
          if (motor_velocity_command_port_.read(vel) == RTT::NewData) {
            velocity_command_pdo_.write(vel);
          }
          break;
        case CYCLIC_POSITION:
          double pos;
          if (motor_position_command_port_.read(pos) == RTT::NewData) {
            position_command_pdo_.write(pos);
          }
          break;
      }
    }

    controlword_pdo_.write(cw);
  }

 private:

  bool enable() {
    if (state_ == SWITCH_ON) {
      enable_ = true;
      return true;
    } else {
      return false;
    }
  }

  void disable() {
    if (enable_) {
      enable_ = false;
    }
  }

  bool beginHoming() {
    if (homing_done_ == false) {
      if (state_ == OPERATION_ENABLED) {
        homing_ = true;
      }
    } else {
      RTT::log(RTT::Error) << "Drive not configured for homing" << RTT::endlog();
    }
    return homing_;
  }

  bool resetFault() {
    if (state_ == FAULT) {
      reset_fault_ = true;
      return true;
    } else {
      return false;
    }
  }

  ServoState getState(int16_t statusword) {
    if ((statusword & NotReadyToSwitchOn_Mask) == NotReadyToSwitchOn_Pattern) {
      return NOT_READY_TO_SWITCH_ON;
    } else if ((statusword & SwitchOnDisabled_Mask) == SwitchOnDisabled_Pattern) {
      return SWITCH_ON_DISABLED;
    } else if ((statusword & ReadyToSwitchOn_Mask) == ReadyToSwitchOn_Pattern) {
      return READY_TO_SWITCH_ON;
    } else if ((statusword & SwitchedOn_Mask) == SwitchedOn_Pattern) {
      return SWITCH_ON;
    } else if ((statusword & OperationEnabled_Mask) == OperationEnabled_Pattern) {
      return OPERATION_ENABLED;
    } else if ((statusword & QuickStopActive_Mask) == QuickStopActive_Pattern) {
      return QUICK_STOP_ACTIVE;
    } else if ((statusword & Fault_Reaction_Mask) == Fault_Reaction_Pattern) {
      return FAULT_REACTION_ACTIVE;
    } else if ((statusword & Fault_Mask) == Fault_Pattern) {
      return FAULT;
    } else {
      return INVALID;
    }
  }

  ECPDOEntry<int16_t> statusword_pdo_;
  ECPDOEntry<int32_t> position_pdo_;
  ECPDOEntry<int32_t> velocity_pdo_;
  ECPDOEntry<int16_t> current_pdo_;

  ECPDOEntry<int16_t> controlword_pdo_;
  ECPDOEntry<uint8_t> mode_of_operation_pdo_;
  ECPDOEntry<int32_t> position_command_pdo_;
  ECPDOEntry<int32_t> velocity_command_pdo_;
  ECPDOEntry<int16_t> current_command_pdo_;

  RTT::OutputPort<double> motor_position_port_;
  RTT::OutputPort<double> motor_velocity_port_;
  RTT::OutputPort<double> motor_current_port_;

  RTT::InputPort<double> motor_position_command_port_;
  RTT::InputPort<double> motor_velocity_command_port_;
  RTT::InputPort<double> motor_current_command_port_;

  ControlMode control_mode_;
  ServoState state_;
  bool enable_;
  bool homing_;
  bool reset_fault_;
  bool homing_done_;
};

char elmo_name[] = "elmo_driver";

typedef ECDriverFactoryService<elmo_name, ElmoDriver> ElmoDriverFactory;

ORO_SERVICE_NAMED_PLUGIN(ElmoDriverFactory, "elmo_driver");
