#include <cstdio>
#include "msr_convertor/sen_com2IMU_data.hpp"

void IMUConverter::sensor_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
{ 
// // Convert PX4 SensorCombined to standard IMU message
  auto imu_msg = sensor_msgs::msg::Imu();
// // Set header
  imu_msg.header.stamp = this->now();
  imu_msg.header.frame_id = "imu_link";
// // Copy accelerometer data
  imu_msg.linear_acceleration.x = msg->accelerometer_m_s2[0];
  imu_msg.linear_acceleration.y = msg->accelerometer_m_s2[1];
  imu_msg.linear_acceleration.z = msg->accelerometer_m_s2[2];
// // Copy gyroscope data
  imu_msg.angular_velocity.x = msg->gyro_rad[0];
  imu_msg.angular_velocity.y = msg->gyro_rad[1];
  imu_msg.angular_velocity.z = msg->gyro_rad[2];
// // Set covariances to unknown
  imu_msg.orientation_covariance[0] = -1;
  imu_msg.angular_velocity_covariance[0] = -1;
  imu_msg.linear_acceleration_covariance[0] = -1;
// // Publish converted message
  publisher_->publish(imu_msg);

}
int main(int argc, char ** argv)
{
  // (void) argc;
  // (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUConverter>());
  rclcpp::shutdown();
  return 0;

  printf("hello world msr_convertor package\n");
  return 0;
}

// // Convert PX4 SensorCombined to standard IMU message
// auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    
// // Set header
// imu_msg->header.stamp = this->now();
// imu_msg->header.frame_id = "base_link";

// // Copy accelerometer data
// imu_msg->linear_acceleration.x = msg->accelerometer_m_s2[0];
// imu_msg->linear_acceleration.y = msg->accelerometer_m_s2[1];
// imu_msg->linear_acceleration.z = msg->accelerometer_m_s2[2];

// // Copy gyroscope data
// imu_msg->angular_velocity.x = msg->gyro_rad[0];
// imu_msg->angular_velocity.y = msg->gyro_rad[1];
// imu_msg->angular_velocity.z = msg->gyro_rad[2];

// // Set covariances to unknown
// imu_msg->orientation_covariance[0] = -1;
// imu_msg->angular_velocity_covariance[0] = -1;
// imu_msg->linear_acceleration_covariance[0] = -1;

// // Publish converted message
// publisher_->publish(*imu_msg);