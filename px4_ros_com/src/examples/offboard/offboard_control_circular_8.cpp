/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

 #include <px4_msgs/msg/offboard_control_mode.hpp>
 #include <px4_msgs/msg/trajectory_setpoint.hpp>
 #include <px4_msgs/msg/vehicle_command.hpp>
 #include <px4_msgs/msg/vehicle_control_mode.hpp>
 #include <rclcpp/rclcpp.hpp>
 #include <stdint.h>
 
 #include <chrono>
 #include <iostream>
 
 #define FLIGHT_ALTITUDE -6.50f
 #define RATE            20  // loop rate hz
 #define RADIUS          6.00 // radius of figure 8 in meters
 #define CYCLE_S         20   // time to complete one figure 8 cycle in seconds
 #define STEPS           (CYCLE_S*RATE) // Hence Total number of steps in the path is 400
 
 #define PI  3.14159265358979323846264338327950
 
 using namespace std::chrono;
 using namespace std::chrono_literals;
 using namespace px4_msgs::msg;
 
 
 class OffboardControl : public rclcpp::Node
 {
 public:
     OffboardControl() : Node("offboard_control")
     {
 
         offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
         trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
         vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
         rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
         auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
 
         vehicle_command_listener_ = this->create_subscription<VehicleControlMode>("/fmu/out/vehicle_control_mode", qos, [this](const px4_msgs::msg::VehicleControlMode::UniquePtr msg){
             c_mode = *msg;
             //if(c_mode.flag_control_offboard_enabled==1){
             //    printf("offboard away!!!");
             // }
         });
 
         offboard_setpoint_counter_ = 0;
 
 
 
         InitPath();
 
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
         timer_ = this->create_wall_timer(50ms, timer_callback);
     }
 
 
 private:
     rclcpp::TimerBase::SharedPtr timer_;
 
     rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
     rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
     rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
     rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_command_listener_;
 
     std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
     
 
     uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
         
     TrajectorySetpoint path[STEPS];
     VehicleControlMode c_mode;
         
     void publish_offboard_control_mode();
     void publish_trajectory_setpoint();
     //void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
     void InitPath();
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
 
 /**
  * @brief Publish a trajectory setpoint
  *        For this example, it sends a trajectory setpoint to make the
  *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
  */
 void OffboardControl::publish_trajectory_setpoint()
 {
 
         // Increment the setpoint counter
         offboard_setpoint_counter_++;
         if(offboard_setpoint_counter_>=STEPS){
         offboard_setpoint_counter_ = 0;
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
     path[offboard_setpoint_counter_].timestamp = this->get_clock()->now().nanoseconds() / 1000;
     
     if(c_mode.flag_control_offboard_enabled==1){
     trajectory_setpoint_publisher_->publish(path[offboard_setpoint_counter_]);
     printf("x:%7.3f y:%7.3f z:%7.3f yaw:%7.1f\n", path[offboard_setpoint_counter_].position[0], path[offboard_setpoint_counter_].position[1],path[offboard_setpoint_counter_].position[2], path[offboard_setpoint_counter_].yaw*180.0f/PI);
     }
     else{
         offboard_setpoint_counter_=0;
     }
 
 
 }
 
 void OffboardControl::InitPath()
 {
 // generate a path following Bernoulli's lemiscate as a parametric equation
 // note this is in ENU coordinates since mavros will convert to NED
 // x right, y forward, z up.
    //  int i;
     const double dt = 1.0/RATE; // time step, as we have rate of 20Hz, this implies 0.05s =dt
     const double dadt = (2.0*PI)/CYCLE_S; // first derivative of angle with respect to time, this divides entire circle into 400 steps
     const double r = RADIUS;
     double centerX = 0.0;  // You can change this to offset the center
     double centerY = 0.0;  // You can change this to offset the center
    // **************************//OLD CODE for Figure 8//************************************** */
    
    //  // Iterating Through Each steps
    //  for(i=0;i<STEPS;i++){
    //      // basic fields in the message
    //      //path[i].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    //      //path[i].type_mask = 0;
 
    //      // calculate the parameter a which is an angle sweeping from -pi/2 to 3pi/2
    //      // through the curve
    //      double a = (-PI/2.0) + i*(2.0*PI/STEPS);
    //      double c = cos(a);
    //      double c2a = cos(2.0*a);
    //      double c4a = cos(4.0*a);
    //      double c2am3 = c2a-3.0;
    //      double s = sin(a);
    //      double cc = c*c;
    //      double ss = s*s;
    //      double sspo = (s*s)+1.0; // sin squared plus one
    //      double ssmo = (s*s)-1.0; // sin squared minus one
    //      double sspos = sspo*sspo;
 
    //      // Position
    //      // https://www.wolframalpha.com/input/?i=%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
    //      path[i].position[0] = cc; //(r*c)   / sspo;
    //      // https://www.wolframalpha.com/input/?i=%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
    //      path[i].position[1] = -cc; //-(r*c*s) / sspo;
    //      path[i].position[2] =  FLIGHT_ALTITUDE;
 
    //      // Velocity
    //      // https://www.wolframalpha.com/input/?i=derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
    //      path[i].velocity[0] =  -dadt*r* s*( ss + 2.0f*cc + 1.0f )  / sspos;
    //      if(path[i].velocity[0]>5)
    //          path[i].velocity[0]=5;
    //      if(path[i].velocity[0]<-5)
    //          path[i].velocity[0]=-5;
    //      // https://www.wolframalpha.com/input/?i=derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
    //      path[i].velocity[1] =   dadt*r* ( ss*ss + ss + ssmo*cc )   /  sspos;
    //      if(path[i].velocity[1]>5)
    //          path[i].velocity[0]=5;
    //      if(path[i].velocity[1]<-5)
    //          path[i].velocity[0]=-5;
 
    //      path[i].velocity[2] =  0;
 
    //      // Acceleration
    //      // https://www.wolframalpha.com/input/?i=second+derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
    //      path[i].acceleration[0] =  -dadt*dadt*8.0*r*s*c*((3.0*c2a) + 7.0)/(c2am3*c2am3*c2am3);
    //      // see https://www.wolframalpha.com/input/?i=second+derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
    //      path[i].acceleration[1] =  dadt*dadt*r*((44.0*c2a) + c4a -21.0)/(c2am3*c2am3*c2am3);
    //      path[i].acceleration[2] =  0.0;
 
    //      // calculate yaw as direction of velocity
    //      // plus pi/2 since ROS yaw=0 lines up with x axis with points out to
    //      // the right, not forward along y axis
    //     //  path[i].yaw = atan2(-path[i].velocity[0], path[i].velocity[1]) + (PI/2.0f);
 
    //      printf("x:%7.3f y:%7.3f yaw:%7.1f\n", path[i].position[0], path[i].position[1], path[i].yaw*180.0f/PI);
    //  }
 
    //  // calculate yaw_rate by dirty differentiating yaw
    //  for(i=0;i<STEPS;i++){
    //      double next = path[(i+1)%STEPS].yaw;
    //      double curr = path[i].yaw;
    //      // account for wrap around +- PI
    //      if((next-curr) < -PI) next+=(2.0*PI);
    //      if((next-curr) >  PI) next-=(2.0*PI);
    //      path[i].yawspeed = (next-curr)/dt;
    //  }
    // **************************//OLD CODE for Figure 8//************************************** */
    for (int i = 0; i < STEPS; i++) {
        double theta = i * (2.0 * PI / STEPS);  // Angle for circular path
        
    
        // Position (circle centered at origin)
        path[i].position[0] = centerX + RADIUS * cos(theta);
        path[i].position[1] = centerY +RADIUS * sin(theta);
        path[i].position[2] = FLIGHT_ALTITUDE- 0.5*sin(theta);  // Keep altitude constant
    
        // Velocity (Derivative of position w.r.t time)
        path[i].velocity[0] = -RADIUS * sin(theta);  // dx/dt
        path[i].velocity[1] = RADIUS * cos(theta);   // dy/dt
        path[i].velocity[2] = 0.5*cos(theta);  // No change in altitude
    
        // Acceleration (Second derivative of position)
        path[i].acceleration[0] = -RADIUS * cos(theta);  // d^2x/dt^2
        path[i].acceleration[1] = -RADIUS * sin(theta);  // d^2y/dt^2
        path[i].acceleration[2] = -0.5*sin(theta);
    
        // Yaw angle to always face the center of the circle
        path[i].yaw = atan2(-path[i].position[1], -path[i].position[0])+PI/6;  // Always facing center
    }
    
    // Compute Yaw Rate (Dirty Differentiation)
    for (int i = 0; i < STEPS; i++) {
        double next = path[(i + 1) % STEPS].yaw;  // Next step yaw
        double curr = path[i].yaw;  // Current step yaw
    
        // Handle yaw wrap-around at ±π
        if ((next - curr) < -PI) next += (2.0 * PI);
        if ((next - curr) > PI) next -= (2.0 * PI);
    
        // Compute yaw rate (yaw speed)
        path[i].yawspeed = (next - curr) / dt;
    }
    
    // Debugging Output
    for (int i = 0; i < STEPS; i++) {
        printf("Step: %d | x: %.3f y: %.3f z: %.3f yaw: %.1f° yawspeed: %.3f rad/s\n",
               i, path[i].position[0], path[i].position[1], path[i].position[2], path[i].yaw * 180.0 / PI, path[i].yawspeed);
    }
 
 }
 
 /* 
 void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
 {
     VehicleCommand msg{};
     msg.param1 = param1;
     msg.param2 = param2;
     msg.command = command;
     msg.target_system = 1;
     msg.target_component = 1;
     msg.source_system = 1;
     msg.source_component = 1;
     msg.from_external = true;
     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
     vehicle_command_publisher_->publish(msg);
 } */
 
 int main(int argc, char *argv[])
 {
     std::cout << "Starting offboard control node..." << std::endl;
     setvbuf(stdout, NULL, _IONBF, BUFSIZ);
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<OffboardControl>());
 
     rclcpp::shutdown();
     return 0;
 }
 