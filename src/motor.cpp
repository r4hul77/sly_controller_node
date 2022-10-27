#include "motor.hpp"


Motor::Motor(){
    m_angular_velocity = 0;
    m_angular_velocity_topic = "angular_velocity_topic";

    m_current = 0;
    m_current_topic = "current_topic";

    m_desired_angular_velocity = 0;
    m_desired_angular_velocity_topic = "desired_angular_velocity";
    
}

Motor::Motor(std::string motor_name):Motor(){
    m_angular_velocity_topic = motor_name+"/"+m_angular_velocity_topic;
    m_desired_angular_velocity_topic = motor_name + "/" + m_desired_angular_velocity_topic;
    m_current_topic = motor_name + "/"  + m_current_topic;
}


void Motor::init(rclcpp::Node* node){
    mp_desired_angular_velocity_pub = node->create_publisher<std_msgs::msg::Float64>
    (m_desired_angular_velocity_topic, 10);

    mp_current_sub = node->create_subscription<std_msgs::msg::Float64>(m_current_topic,10,
    std::bind(&Motor::current_callback, this, std::placeholders::_1));

    mp_angular_velocity_sub = node->create_subscription<std_msgs::msg::Float64>(m_angular_velocity_topic, 10,
    std::bind(&Motor::angular_vel_callback, this, std::placeholders::_1));
}


void Motor::angular_vel_callback(std_msgs::msg::Float64::SharedPtr msg){
    m_angular_velocity = msg->data;
    m_ang_vel_time = rclcpp::Clock(RCL_ROS_TIME).now();    
}


void Motor::current_callback(std_msgs::msg::Float64::SharedPtr msg){
    m_current = msg->data;
    m_ang_vel_time = rclcpp::Clock(RCL_ROS_TIME).now();
}

void Motor::publish_desired_ang_vel(float& angular_vel){
    auto message = std_msgs::msg::Float64();

    message.data = angular_vel;

    mp_desired_angular_velocity_pub->publish(message);

}


void Motor::get_current(float& current, float& time){
    current = this->m_current;
    time = this->m_current_time.nanoseconds()*1e-6;
}

void Motor::get_ang_vel(float& ang_vel, float& time){
    ang_vel = m_angular_velocity;
    time = this->m_ang_vel_time.nanoseconds()*1e-6;
}
