#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "px4_msgs/msg/vehicle_odometry.hpp"

class TrackingSubscriber : public rclcpp::Node
{
public:
    TrackingSubscriber() : Node("tracking_subscriber")
    {
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
		qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tag_detections", qos, std::bind(&TrackingSubscriber::pose_callback, this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() { this->publish_transform(); });

        vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos, std::bind(&TrackingSubscriber::odometry_callback, this, std::placeholders::_1));

    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received PoseStamped Data:\n"
                    "Position: x=%.2f, y=%.2f, z=%.2f\n"
                    "Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                    msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

    }

    void publish_transform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "ned";
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 1.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 0.0;

        tf_broadcaster_->sendTransform(transform);
    }

    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received VehicleOdometry Data:\n"
                    "Position: x=%.2f, y=%.2f, z=%.2f\n"
                    "Velocity: vx=%.2f, vy=%.2f, vz=%.2f",
                    msg->position[0], msg->position[1], msg->position[2],
                    msg->velocity[0], msg->velocity[1], msg->velocity[2]);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "ned";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = msg->position[0];
        transform.transform.translation.y = msg->position[1];
        transform.transform.translation.z = msg->position[2];
        transform.transform.rotation.x = msg->q[0];
        transform.transform.rotation.y = msg->q[1];
        transform.transform.rotation.z = msg->q[2];
        transform.transform.rotation.w = msg->q[3];
        tf_broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackingSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
