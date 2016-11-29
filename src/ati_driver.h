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

#ifndef ATI_DRIVER_H_
#define ATI_DRIVER_H_

#include <rtt/plugin/ServicePlugin.hpp>

#include <ec_hardware/ECDriver.h>
#include <ec_hardware/ECDriverFactory.h>
#include <ec_hardware/ECPDOEntry.h>
#include <string>
#include "geometry_msgs/Wrench.h"

class ATIDriver : public ECDriver {
 public:
  enum ControlMode {
    PROFILE_POSITION = 1,
    PROFILE_VELOCITY = 2,
    PROFILE_CURRENT = 3,
    HOMING = 6,
    CYCLIC_CURRENT = 10,
    CYCLIC_VELOCITY = 9,
    CYCLIC_POSITION = 8
  };
  enum ServoState {
    INVALID = 0,
    NOT_READY_TO_SWITCH_ON = 1,
    SWITCH_ON_DISABLED = 2,
    READY_TO_SWITCH_ON = 3,
    SWITCH_ON = 4,
    OPERATION_ENABLED = 5,
    QUICK_STOP_ACTIVE = 6,
    FAULT_REACTION_ACTIVE = 7,
    FAULT = 8
  };

  explicit ATIDriver(const std::string &name);

  ~ATIDriver();

  virtual bool configureHook(const YAML::Node &cfg);
  virtual void updateInputs();
  virtual void updateOutputs();

 private:
  void bias();

  bool setFilter(int32_t fl);

  bool setCalib(int32_t cl);

  ECPDOEntry<int32_t> force_x_pdo_;
  ECPDOEntry<int32_t> force_y_pdo_;
  ECPDOEntry<int32_t> force_z_pdo_;
  ECPDOEntry<int32_t> torque_x_pdo_;
  ECPDOEntry<int32_t> torque_y_pdo_;
  ECPDOEntry<int32_t> torque_z_pdo_;

  ECPDOEntry<int32_t> control1_pdo_;

  RTT::OutputPort<geometry_msgs::Wrench> wrench_port_;

  bool bias_;

  int32_t calib_;
  int32_t filter_;
};


#endif  // ATI_DRIVER_H_
