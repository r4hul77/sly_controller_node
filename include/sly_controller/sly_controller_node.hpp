#include "motor.hpp"
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sly_controller.hpp"


struct MotorVelsStruct{
    std::string m_name;
    float m_radi;
    float m_desired_ang_vel;
    float m_ang_vel;
    Motor m_motor;

    MotorVelsStruct(std::string, float);


};


class sly_controller_node: public rclcpp::Node{
    
    std::vector<std::shared_ptr<MotorVelsStruct>> mp_motors_container;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mp_desired_vel_sub;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mp_odom_pub;

    nav_msgs::msg::Odometry m_odom_msg;

    geometry_msgs::msg::TransformStamped m_odom_tf;

    float m_max_angular_velocity;

    std::unique_ptr<SlyController> mp_controller;

    float m_track_width;

    float m_wheel_base;

    Eigen::Vector2d m_cmd_vels;

    Eigen::Matrix<double, 6, 1> m_state;

    tf2::Quaternion m_quat;
 
    Eigen::Matrix2d m_omega_maper;

    rclcpp::Time m_odom_update_time;

    std::unique_ptr<tf2_ros::TransformBroadcaster> mp_tf_broadcaster;
    
    rclcpp::TimerBase::SharedPtr mp_timer_;

    bool m_first_odom_update;

public:

    void declare_parameters();

    sly_controller_node();

    void initialize_motors();

    void initialize_subs();

    void calculate_vels();

    void publish_vels();

    void init();

    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void publish_odom_msg();

    void build_odom_msg();

    void init_odom_msg();

    void initialize_publishers();

    void publish_tf();

    void build_tf_msg();

    void init_tf_msg();

    void odom_loop();

    void odom_update();

};