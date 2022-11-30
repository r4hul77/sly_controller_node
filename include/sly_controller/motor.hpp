#ifndef MOTOR_H
#define MOTOR_H


#include <string>
#include <std_msgs/msg/float64.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

class Motor{

    std::string m_desired_angular_velocity_topic;
    std::string m_angular_velocity_topic;
    std::string m_current_topic;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mp_desired_angular_velocity_pub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr mp_angular_velocity_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr mp_current_sub;

    float m_current;
    float m_desired_angular_velocity;

    rclcpp::Node* mp_parent_node;

public:

    rclcpp::Time m_current_time;
    rclcpp::Time m_ang_vel_time;

    Motor();

    float m_angular_velocity;

    Motor(std::string motor_name);

    void init(rclcpp::Node*);

    void get_current(float&, float&);

    void get_ang_vel(float& ang_vel, float& time);

    void angular_vel_callback(const std_msgs::msg::Float64::SharedPtr msg);

    void current_callback(const std_msgs::msg::Float64::SharedPtr msg);

    void publish_desired_ang_vel(float&);

};
#endif
