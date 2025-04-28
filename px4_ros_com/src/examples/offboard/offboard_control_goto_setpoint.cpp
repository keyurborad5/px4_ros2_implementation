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
 

 class OffboardControl : public rclcpp::Node
 {
 public:
    OffboardControl() : Node("offboard_control")
    {
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        goto_setpoint_publisher_ = this->create_publisher<GotoSetpoint>("/fmu/in/goto_setpoint", 10);
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        vehicle_command_listener_ = this->create_subscription<VehicleControlMode>("/fmu/out/vehicle_control_mode", qos, [this](const px4_msgs::msg::VehicleControlMode::UniquePtr msg){
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
            publish_goto_setpoint();


        };
        timer_ = this->create_wall_timer(50ms, timer_callback);
    }
 private:
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<GotoSetpoint>::SharedPtr goto_setpoint_publisher_;
    rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_command_listener_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    

    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
    VehicleControlMode c_mode;    

        
    void publish_offboard_control_mode();
    void publish_goto_setpoint();

 };
  /**
  * @brief Publish the offboard control mode.
  *        For this example, only position and altitude controls are active.
  */
 void OffboardControl::publish_offboard_control_mode()
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
 
 void OffboardControl::publish_goto_setpoint()
 {
 
         // Increment the setpoint counter
         offboard_setpoint_counter_++;
         if(offboard_setpoint_counter_>=100){
         offboard_setpoint_counter_ = 0;}

         auto msg = px4_msgs::msg::GotoSetpoint();

         msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;  // microseconds
     
         msg.position = {5.0, 2.0, -3.0};  // NED position [x, y, z]
     
         msg.flag_control_heading = true;
         msg.heading = 1.57;  // radians
     
         msg.flag_set_max_horizontal_speed = true;
         msg.max_horizontal_speed = 2.0;
     
         msg.flag_set_max_vertical_speed = true;
         msg.max_vertical_speed = 1.0;
     
         msg.flag_set_max_heading_rate = true;
         msg.max_heading_rate = 0.3;
        
        if(c_mode.flag_control_offboard_enabled==1){

         goto_setpoint_publisher_->publish(msg);
         RCLCPP_INFO(this->get_logger(), "Published GotoSetpoint");

        }
         //if(offboard_setpoint_counter_>=STEPS){
         //offboard_setpoint_counter_=0;
         //offboard_sub_counter_++;
         //}
         //float x = 5.0 * std::cos(0.25 * M_PI * offboard_sub_counter_);
         //float y = 5.0 * std::sin(0.25 * M_PI * offboard_sub_counter_);
        //float z = -5.0;
     
     //TrajectorySetpoint msg{};
     //msg.position = {x, y, z};
     //msg.yaw = 0; // [-PI:PI]
    //  path[offboard_setpoint_counter_].timestamp = this->get_clock()->now().nanoseconds() / 1000;
     
    //  if(c_mode.flag_control_offboard_enabled==1){
    //  trajectory_setpoint_publisher_->publish(path[offboard_setpoint_counter_]);
    //  printf("x:%7.3f y:%7.3f z:%7.3f yaw:%7.1f\n", path[offboard_setpoint_counter_].position[0], path[offboard_setpoint_counter_].position[1],path[offboard_setpoint_counter_].position[2], path[offboard_setpoint_counter_].yaw*180.0f/PI);
    //  }
    //  else{
    //      offboard_setpoint_counter_=0;
    //  }
 
 
 }
 
 int main(int argc, char *argv[])
 {
     std::cout << "Starting offboard control node..." << std::endl;
     setvbuf(stdout, NULL, _IONBF, BUFSIZ);
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<OffboardControl>());
 
     rclcpp::shutdown();
     return 0;
 }
 