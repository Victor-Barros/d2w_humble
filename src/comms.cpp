#include "d2w_ros2/comms.hpp"

#include <chrono>
#include <memory>

#include "LinuxI2cCommunicator.h"

using namespace std::chrono_literals;

Comms::Comms() : Node("comms") {
    // Create I2C communicator
    std::unique_ptr<I2cCommunicator> i2cBus1 = std::make_unique<LinuxI2cCommunicator>();
    std::unique_ptr<I2cCommunicator> i2cBus2 = std::make_unique<LinuxI2cCommunicator>();
    mpu9250_ = std::make_unique<MPU9250Sensor>(std::move(i2cBus1));
    drive_comms_ = std::make_unique<DriveComms>(std::move(i2cBus2));
    // Declare parameters
    declareParameters();

    // Set parameters
    mpu9250_->setGyroscopeRange(
    static_cast<MPU9250Sensor::GyroRange>(this->get_parameter("gyro_range").as_int()));
    mpu9250_->setAccelerometerRange(
    static_cast<MPU9250Sensor::AccelRange>(this->get_parameter("accel_range").as_int()));
    mpu9250_->setDlpfBandwidth(
    static_cast<MPU9250Sensor::DlpfBandwidth>(this->get_parameter("dlpf_bandwidth").as_int()));
    mpu9250_->setGyroscopeOffset(this->get_parameter("gyro_x_offset").as_double(),
        this->get_parameter("gyro_y_offset").as_double(),
        this->get_parameter("gyro_z_offset").as_double());
    mpu9250_->setAccelerometerOffset(this->get_parameter("accel_x_offset").as_double(),
        this->get_parameter("accel_y_offset").as_double(),
        this->get_parameter("accel_z_offset").as_double());
    
    // Check if we want to calibrate the sensor
    if (this->get_parameter("calibrate").as_bool()) {
        RCLCPP_INFO(this->get_logger(), "Calibrating...");
        mpu9250_->calibrate();
    }
    mpu9250_->printConfig();
    mpu9250_->printOffsets();

    // Create subscribers
    subscription_joint_vel = this->create_subscription<std_msgs::msg::Float64MultiArray>("joint_vel", 10, std::bind(&Comms::velCallback, this, std::placeholders::_1));

    // Create publishers
    publisher_imu = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    publisher_raw_odom = this->create_publisher<std_msgs::msg::Float64MultiArray>("raw_odom", 10);

    // Create timer
    //std::chrono::duration<int64_t, std::milli> frequency = 1000ms / this->get_parameter("gyro_range").as_int();
    std::chrono::duration<int64_t, std::milli> frequency = 30ms;
    timer_ = this->create_wall_timer(frequency, std::bind(&Comms::handleComms, this));
}

void Comms::handleComms() {
    // Send commands to motors and read odometry
    auto odom_message = std_msgs::msg::Float64MultiArray();
    for (int i=0; i<4; i++) {
        if (i == 1 || i == 3) { // inverter motores 2 e 4 (caracteristica construtiva do robo)
            drive_comms_->startTransaction(i,-joint_vel[i],&odom_buffer);
            raw_odom[i] = -odom_buffer;
        } else {
            drive_comms_->startTransaction(i,joint_vel[i],&odom_buffer);
            raw_odom[i] = odom_buffer;
        }
    }
    odom_message.data = raw_odom;

    // Handle IMU
    auto imu_message = sensor_msgs::msg::Imu();
    imu_message.header.stamp = this->get_clock()->now();
    imu_message.header.frame_id = "base_link";
    
    // Direct measurements
    imu_message.linear_acceleration_covariance = {0};
    imu_message.linear_acceleration.x = mpu9250_->getAccelerationX();
    imu_message.linear_acceleration.y = mpu9250_->getAccelerationY();
    imu_message.linear_acceleration.z = mpu9250_->getAccelerationZ();
    imu_message.angular_velocity_covariance[0] = {0};
    imu_message.angular_velocity.x = mpu9250_->getAngularVelocityX();
    imu_message.angular_velocity.y = mpu9250_->getAngularVelocityY();
    imu_message.angular_velocity.z = mpu9250_->getAngularVelocityZ();
    
    // Calculate euler angles, convert to quaternion and store in message
    imu_message.orientation_covariance = {0};
    calculateOrientation(imu_message);

    // Publish messages
    publisher_raw_odom->publish(odom_message);
    publisher_imu->publish(imu_message);
}

void Comms::velCallback(const std_msgs::msg::Float64MultiArray & msg) {
    for (int i=0; i<4; i++) {
        joint_vel[i] = (double)msg.data[i];
    }
}
void Comms::declareParameters() {
    this->declare_parameter<bool>("calibrate", true);
    this->declare_parameter<int>("gyro_range", MPU9250Sensor::GyroRange::GYR_250_DEG_S);
    this->declare_parameter<int>("accel_range", MPU9250Sensor::AccelRange::ACC_2_G);
    this->declare_parameter<int>("dlpf_bandwidth", MPU9250Sensor::DlpfBandwidth::DLPF_260_HZ);
    this->declare_parameter<double>("gyro_x_offset", 0.0);
    this->declare_parameter<double>("gyro_y_offset", 0.0);
    this->declare_parameter<double>("gyro_z_offset", 0.0);
    this->declare_parameter<double>("accel_x_offset", 0.0);
    this->declare_parameter<double>("accel_y_offset", 0.0);
    this->declare_parameter<double>("accel_z_offset", 0.0);
    this->declare_parameter<int>("frequency", 0.0);
}

void Comms::calculateOrientation(sensor_msgs::msg::Imu& imu_message) {
    // Calculate Euler angles
    double roll, pitch, yaw;
    roll = atan2(imu_message.linear_acceleration.y, imu_message.linear_acceleration.z);
    pitch = atan2(-imu_message.linear_acceleration.y, (sqrt(imu_message.linear_acceleration.y * imu_message.linear_acceleration.y + imu_message.linear_acceleration.z * imu_message.linear_acceleration.z)));
    yaw = atan2(mpu9250_->getMagneticFluxDensityY(), mpu9250_->getMagneticFluxDensityX());

    // Convert to quaternion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    imu_message.orientation.x = cy * cp * sr - sy * sp * cr;
    imu_message.orientation.y = sy * cp * sr + cy * sp * cr;
    imu_message.orientation.z = sy * cp * cr - cy * sp * sr;
    imu_message.orientation.w = cy * cp * cr + sy * sp * sr;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Comms>());
  rclcpp::shutdown();
  return 0;
}