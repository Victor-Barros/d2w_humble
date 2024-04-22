#include <memory>
#include <string>
#include <unordered_map>

#include "I2cCommunicator.h"

class DriveComms {
 public:
  DriveComms(std::unique_ptr<I2cCommunicator> i2cBus);
  void startTransaction(int joint, double vel, double* raw_odom);

 private:
  void initI2c(int address) const;
  void setVel(double joint_vel);
  void readOdom(double* value);
  std::unique_ptr<I2cCommunicator> i2cBus_;
  const std::array<int, 4> DEFAULT_ADDRESSES = {0x0A, 0x0B, 0x0C, 0x0D};
};

