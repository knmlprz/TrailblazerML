#include "ros_communication.h"
#include "motor_control.h"

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Parametry robota
const float WHEEL_BASE = 1;     // Odległość między kołami [m]

RosCommunication::RosCommunication(){
   
}

void RosCommunication::initialize(){
     // Configure serial transport
     Serial.begin(115200);
     // set_microros_serial_transports(Serial);
     // Adding Wifi
     IPAddress agent_ip(192, 168, 4, 64);
     size_t agent_port = 8888;
 
     char ssid[] = "Bafau";
     char psk[]= "bafaubafau";
 
     set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
     delay(2000);
     allocator = rcl_get_default_allocator();
 
     //create init_options
     rclc_support_init(&support, 0, NULL, &allocator);
     // create node
     rclc_node_init_default(&node, "micro_controler_cmd_vel", "", &support);
}

void RosCommunication::executors_start(){
    // create executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &RosCommunication::cmd_vel_callback, ON_NEW_DATA);

};
void RosCommunication::subscriber_define(){
    rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel_nav");
};
void RosCommunication::cmd_vel_callback(const void *msg_recv){
    const geometry_msgs__msg__Twist * received_data = (const geometry_msgs__msg__Twist *) msg_recv;
    float linear_vel = received_data->linear.x;
    float angular_vel = received_data->angular.z;
    Serial.print(linear_vel);Serial.print("/");Serial.println(angular_vel);

    // Obliczenie prędkości kół na podstawie równań kinematycznych
    float v_r = linear_vel + (angular_vel * WHEEL_BASE) / 2;
    float v_l = linear_vel - (angular_vel * WHEEL_BASE) / 2;

    Serial.print("Left Wheel Velocity: "); Serial.println(v_l);
    Serial.print("Right Wheel Velocity: "); Serial.println(v_r);

    setMotorSpeeds(v_l * 255, v_r * 255);

}
void RosCommunication::start_receiving_msgs(){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
};