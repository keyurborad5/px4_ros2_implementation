#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/goto_setpoint.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class TrackingSubscriber : public rclcpp::Node
{
public:
    TrackingSubscriber() : Node("tracking_subscriber")
    {
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
		qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tag_detections", qos, std::bind(&TrackingSubscriber::pose_callback, this, std::placeholders::_1));

        // Create a TransformListener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
        timer2_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrackingSubscriber::lookup_transform, this));
        
        
        // Create a TransformBroadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() { this->publish_transform(); });

        vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos, std::bind(&TrackingSubscriber::odometry_callback, this, std::placeholders::_1));
        //---------------------------------------------------------------------------------------------//
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos2 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        vehicle_command_listener_ = this->create_subscription<VehicleControlMode>("/fmu/out/vehicle_control_mode", qos2, [this](const px4_msgs::msg::VehicleControlMode::UniquePtr msg){
            c_mode = *msg;
            });
        offboard_setpoint_counter_ = 0;
        auto timer_callback = [this]() -> void {
 
 
            // offboard_control_mode needs to be paired with trajectory_setpoint

            
            double intpart;
            //printf("modf:%7.3f", );
            if(modf(((double)offboard_setpoint_counter_)/2, &intpart)==0.0){
                publish_offboard_control_mode();
                //printf("published mode msg");

            }
            //print(modf(offboard_setpoint_counter_/5, &intpart))
            publish_trajectory_setpoint();


        };
        timer3_ = this->create_wall_timer(50ms, timer_callback);

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

        geometry_msgs::msg::TransformStamped transform2;
        transform2.header.stamp = this->now();
        transform2.header.frame_id = "base_link";
        transform2.child_frame_id = "x500_depth_0/camera_link/IMX214";
        transform2.transform.translation.x = 0.120;
        transform2.transform.translation.y = -0.03;
        transform2.transform.translation.z = -0.242;
        transform2.transform.rotation.x = 0.2706 ;
        transform2.transform.rotation.y = 0.2706;
        transform2.transform.rotation.z = 0.6533;
        transform2.transform.rotation.w = 0.6533;
        
        // .12 .03 .242 0 0.785 0
        tf_broadcaster_->sendTransform(transform2);
    }

    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received VehicleOdometry Data:\n"
        //             "Position: x=%.2f, y=%.2f, z=%.2f\n"
        //             "Velocity: vx=%.2f, vy=%.2f, vz=%.2f",
        //             msg->position[0], msg->position[1], msg->position[2],
        //             msg->velocity[0], msg->velocity[1], msg->velocity[2]);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "ned";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = msg->position[0];
        transform.transform.translation.y = msg->position[1];
        transform.transform.translation.z = msg->position[2];
        transform.transform.rotation.w = msg->q[0];
        transform.transform.rotation.x = msg->q[1];
        transform.transform.rotation.y = msg->q[2];
        transform.transform.rotation.z = msg->q[3];
        tf_broadcaster_->sendTransform(transform);
        // if(offboard_setpoint_counter_>=10 || (base_x==0.0 && base_y==0.0)){
            base_x = msg->position[0];
            base_y = msg->position[1];
        // }
    }

    void lookup_transform() {
        geometry_msgs::msg::TransformStamped transformStamped;
        // geometry_msgs::msg::TransformStamped transformStamped_baselink;

        try {
            transformStamped = tf_buffer_->lookupTransform(
                "base_link",         // target_frame
                "test",        // source_frame
                tf2::TimePointZero);  // latest available
    
            RCLCPP_INFO(this->get_logger(), "Transform from map to april tag:");
            RCLCPP_INFO(this->get_logger(), "Translation: x=%.2f, y=%.2f, z=%.2f",
                        transformStamped.transform.translation.x,
                        transformStamped.transform.translation.y,
                        transformStamped.transform.translation.z);
            april_x = transformStamped.transform.translation.x;
            april_y = transformStamped.transform.translation.y;
            april_z = transformStamped.transform.translation.z;
            
            
            // if(offboard_setpoint_counter_>=10){
                diff_x=april_x-5.0;
                diff_y=april_y-0.0;
                // offboard_setpoint_counter_ = 0;}
            // offboard_setpoint_counter_++;
    
        } catch (tf2::TransformException &ex) {
            diff_x=0.0;
            diff_y=0.0;
            RCLCPP_WARN(this->get_logger(), "Could not transform map to test: %s", ex.what());
        }
        // try {
        //     transformStamped_baselink = tf_buffer_->lookupTransform(
        //         "map",         // target_frame
        //         "base_link",        // source_frame
        //         tf2::TimePointZero);  // latest available
    
        //     // RCLCPP_INFO(this->get_logger(), "Transform from map to baselink:");
        //     // RCLCPP_INFO(this->get_logger(), "Translation: x=%.2f, y=%.2f, z=%.2f",
        //     //             transformStamped.transform.translation.x,
        //     //             transformStamped.transform.translation.y,
        //     //             transformStamped.transform.translation.z);

        //     base_x = transformStamped.transform.translation.x;
        //     base_y = transformStamped.transform.translation.y;
        //     base_z = transformStamped.transform.translation.z;
    
        // } catch (tf2::TransformException &ex) {
        //     RCLCPP_WARN(this->get_logger(), "Could not transform map to Baselink: %s", ex.what());
        // }

        
    }
    

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;
    float base_x=0.0;
    float base_y=0.0;
    float base_z;
    float april_x;
    float april_y;
    float april_z;
    float diff_x=0.0;
    float diff_y=0.0;
    float diff_z=0.0;


    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    //---------------------------------------------------------------//
    rclcpp::TimerBase::SharedPtr timer3_;
    
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_command_listener_;

    // std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    

    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
    VehicleControlMode c_mode;    

        
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();

};

/**
  * @brief Publish the offboard control mode.
  *        For this example, only position and altitude controls are active.
  */
 void TrackingSubscriber::publish_offboard_control_mode()
 {
     OffboardControlMode msg{};
     msg.position = true;
     msg.velocity = false;
     msg.acceleration = false;
     msg.attitude = false;
     msg.body_rate = false;
     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
     offboard_control_mode_publisher_->publish(msg);
 
 }
void TrackingSubscriber::publish_trajectory_setpoint()
{

        // Increment the setpoint counter
        offboard_setpoint_counter_++;
        // if(offboard_setpoint_counter_<=100){
        // offboard_setpoint_counter_ = 0;}

        auto msg = px4_msgs::msg::TrajectorySetpoint();

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;  // microseconds
    
        msg.position = {base_x-diff_x, base_y-diff_y, -6.0};  // NED position [x, y, z]
        msg.yaw = 3.14;  // radians
        RCLCPP_INFO(this->get_logger(), "Publishing Trajectory Translation: x=%.2f, y=%.2f, z=%.2f",
                        base_x-diff_x,
                        base_y-diff_y,
                        -6.0);
        // msg.flag_control_heading = false;
        // msg.heading = 1.57;  // radians
    
        // msg.flag_set_max_horizontal_speed = true;
        // msg.max_horizontal_speed = 2.0;
    
        // msg.flag_set_max_vertical_speed = true;
        // msg.max_vertical_speed = 1.0;
    
        // msg.flag_set_max_heading_rate = false;
        // msg.max_heading_rate = 0.3;
    
    if(c_mode.flag_control_offboard_enabled==1 && offboard_setpoint_counter_>=30){

        trajectory_setpoint_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published TrajectorySetpoint");
        offboard_setpoint_counter_ = 0;

    }
}
 

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackingSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
