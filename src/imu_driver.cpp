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

#include "imu_driver.h"
#include <string>


IMUDriver::IMUDriver(const std::string &name)
    : ECDriver(name),
      acceleration_x_pdo_(0x2065, 0),
      acceleration_y_pdo_(0x2066, 0),
      acceleration_z_pdo_(0x2067, 0),
      rotation_x_pdo_(0x2068, 0),
      rotation_y_pdo_(0x2069, 0),
      rotation_z_pdo_(0x206a, 0),
      control1_pdo_(0x206b, 0),
      bias_(true),
      calib_(0),
      filter_(0) {
  this->provides()->addPort("IMU_Msr_OUTPORT", port_imu_msr_outport_);


}

IMUDriver::~IMUDriver() {
}

bool IMUDriver::configureHook(const YAML::Node &cfg) {
  uint16_t value = 0x2;
  uint16_t value2 = 0x4;
  rangeScale = Pre150degpsec;

  this->addPDOEntry(&acceleration_x_pdo_);
  this->addPDOEntry(&acceleration_y_pdo_);
  this->addPDOEntry(&acceleration_z_pdo_);
  this->addPDOEntry(&rotation_x_pdo_);
  this->addPDOEntry(&rotation_y_pdo_);
  this->addPDOEntry(&rotation_z_pdo_);
  this->addPDOEntry(&control1_pdo_);


  if (cfg["range"]) {
    value = cfg["range"].as<int>();
    if(value==1)
      rangeScale = Pre75degpsec;
    if(value==2)
      rangeScale = Pre150degpsec;
    if(value==4)
      rangeScale = Pre300degpsec;

  }
  slave_->addSDOConfig(0x2205, 0, value2);

  slave_->addSDOConfig(0x2206, 0, value);

  return true;
}

void IMUDriver::updateInputs() {
  int16_t ax, ay, az, rx, ry, rz;
  sensor_msgs::Imu wr;

  ax = acceleration_x_pdo_.read();
  ay = acceleration_y_pdo_.read();
  az = acceleration_z_pdo_.read();

  rx = rotation_x_pdo_.read();
  ry = rotation_y_pdo_.read();
  rz = rotation_z_pdo_.read();

  wr.linear_acceleration.x = static_cast<double>(ax) / 3003.003003003;
  wr.linear_acceleration.y = static_cast<double>(ay) / 3003.003003003;
  wr.linear_acceleration.z = static_cast<double>(az) / 3003.003003003;

  wr.angular_velocity.x = static_cast<double>(rx) / rangeScale;
  wr.angular_velocity.y = static_cast<double>(ry) / rangeScale;
  wr.angular_velocity.z = static_cast<double>(rz) / rangeScale;

  wr.header.stamp = rtt_rosclock::host_now();
  port_imu_msr_outport_.write(wr);
}

void IMUDriver::updateOutputs() {
  /*
  int32_t cw1 = 0;

  if (bias_) {
    cw1 |= 1;
    bias_ = false;
  }

  cw1 |= filter_ << 4;
  cw1 |= calib_ << 8;

  control1_pdo_.write(cw1);
   */
}


void IMUDriver::bias() {
  bias_ = true;
}

bool IMUDriver::setFilter(int32_t fl) {
  /*
  if (fl < 0 || fl > 8) {
    return false;
  } else {
    filter_ = fl;
    return true;
  }
   */
}

bool IMUDriver::setCalib(int32_t cl) {
  /*
  if (cl < 0 || cl > 8) {
    return false;
  } else {
    calib_ = cl;
    return true;
  }
   */
}

char imu_name[] = "imu_driver";

typedef ECDriverFactoryService<imu_name, IMUDriver> IMUDriverFactory;

ORO_SERVICE_NAMED_PLUGIN(IMUDriverFactory, "imu_driver");
