#include "mpu9250sensor.h"
#include "drive_comms.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class Comms : public rclcpp::Node {
    public:
        Comms();

    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_raw_odom;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_joint_vel;
        std::unique_ptr<MPU9250Sensor> mpu9250_;
        std::unique_ptr<DriveComms> drive_comms_;
        size_t count_;
        rclcpp::TimerBase::SharedPtr timer_;
        void handleComms();
        void velCallback(const std_msgs::msg::Float64MultiArray & msg);
        double joint_vel[4] = {0,0,0,0};
        double odom_buffer = 0;
        std::vector <double> raw_odom = {0,0,0,0};
        void declareParameters();
        void calculateOrientation(sensor_msgs::msg::Imu& imu_message);
};