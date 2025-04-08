#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class IMUConverter : public rclcpp::Node {
    public:
      IMUConverter() : Node("imu_converter") {
        // Configure QoS profile for PX4 messages
        auto qos_profile = rclcpp::QoS(
          rclcpp::QoSInitialization(
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            10
          )
        );
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        
        // Create subscription to sensor_combined topic
        subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
          "/fmu/out/sensor_combined", 
          qos_profile, 
          std::bind(&IMUConverter::sensor_callback, this, _1)
        );
        
        // Create publisher for standard IMU message
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
          "/imu/data_raw", 
          10  // Standard QoS for Madgwick filter
        );
        
        RCLCPP_INFO(this->get_logger(), "IMU Converter node initialized");
      }
    private:
        void sensor_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg);
        rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};
