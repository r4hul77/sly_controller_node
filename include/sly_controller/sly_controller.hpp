#include "motor.hpp"
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
struct MotorVelsStruct{
    std::string m_name;
    float m_radi;
    float m_current;
    float m_ang_vel;
    float m_desired_ang_vel;
    Motor m_motor;

    MotorVelsStruct(std::string, float);

};


class sly_controller_node: public rclcpp::Node{
    
    std::vector<std::shared_ptr<MotorVelsStruct>> mp_motors_container;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mp_desired_vel_sub;

    float m_max_angular_velocity;

    float m_track_width;

    float m_wheel_base;

    float desired_vel;
    float desired_ang_vel;

public:

    void declare_parameters();

    sly_controller_node();

    void initialize_motors();

    void initialize_subs();

    void calculate_vels();

    void publish_vels();

    void init();

    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
};