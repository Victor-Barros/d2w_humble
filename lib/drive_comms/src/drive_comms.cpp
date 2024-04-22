#include "drive_comms.h"

extern "C" {
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
}

#include <iostream>
#include <thread>
#include <string>

DriveComms::DriveComms(std::unique_ptr<I2cCommunicator> i2cBus) : i2cBus_(std::move(i2cBus))
{
  //DriveComms();
}

void DriveComms::initI2c(int address) const {
  if (ioctl(i2cBus_->getFile(), I2C_SLAVE, address) < 0) {
    std::cerr << "Failed to find device address " << std::hex << address << "! Check device address!";
    exit(1);
  }
}

void DriveComms::startTransaction(int joint, double vel, double* raw_odom) {
  //To save time, lets set speed and read odometry in the same transaction
  initI2c(DEFAULT_ADDRESSES[joint]);
  setVel(vel);
  readOdom(raw_odom);
}

void DriveComms::setVel(double joint_vel) {
  // Write setpoints to specific controllers
  char buffer[7] = {0,0,0,0,0,0,0};
  int dir=0;
  if (joint_vel > 0) dir = 1;
  if (std::abs(joint_vel) < 200) {
    joint_vel = 0;
    dir = 0;
  } else if (std::abs(joint_vel) > 4095) {
    joint_vel = 4095;
  } else {
    joint_vel = std::abs(joint_vel);
  } 
  std::snprintf(buffer,sizeof(buffer),"%04d,%1d",(int)joint_vel,(int)dir);
  i2cBus_->write_block(0x69, sizeof(buffer),(unsigned char *)buffer);
}

void DriveComms::readOdom(double* value) {
  //
  unsigned char buffer1[12] = {};
  i2cBus_->read_block(0xFF, buffer1);

  std::string vel = (char*)buffer1;

  auto n = vel.find('e');
  if (n != vel.npos) {
    vel.resize(n);
  }

  try
  {
    *value = std::stod(vel)/100;
  }
  catch(const std::exception& e)
  {
  std::cerr << e.what() << "(" << vel << ")" << std::endl;
  }
}

