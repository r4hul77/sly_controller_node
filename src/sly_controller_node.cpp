#include "sly_controller_node.hpp"
#include "motor.hpp"

MotorVelsStruct::MotorVelsStruct(std::string motor_name, float radius):m_name(motor_name),
m_radi(radius), m_desired_ang_vel(0),
 m_motor(Motor(motor_name)){}



sly_controller_node::sly_controller_node():Node("sly_controller_node"), mp_motors_container(4){
  init();


}

void sly_controller_node::declare_parameters(){
  
  this->declare_parameter("motor_names", std::vector<std::string>({"front_left", "front_right",
               "rear_left", "rear_right"}));
  
  this->declare_parameter("wheel_radius", 0.1651);

  this->declare_parameter("max_angular_velocity", 0.5);

  this->declare_parameter("wheel_base", 0.6096);

  this->declare_parameter("track_width", 0.3556);

  this->declare_parameter("odom_frequency", 50.0);
}

void sly_controller_node::initialize_publishers(){
  mp_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  mp_tf_broadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}


void sly_controller_node::init(){
  
  this->declare_parameters();

  auto motor_names = this->get_parameter("motor_names").as_string_array();

  auto radi = this->get_parameter("wheel_radius").as_double();

  this->m_max_angular_velocity  = this->get_parameter("max_angular_velocity").as_double();

  this->m_wheel_base = this->get_parameter("wheel_base").as_double();

  this->m_track_width = this->get_parameter("track_width").as_double();

  auto freq = this->get_parameter("odom_frequency").as_double();
  long period = (long)1000/freq;
  mp_timer_ = this->create_wall_timer(std::chrono::milliseconds(period), std::bind(&sly_controller_node::odom_loop, this));

  this->mp_controller = std::make_unique<SlyController>(radi, m_track_width);

  m_omega_maper << radi, radi, 2*radi/m_track_width, -2*radi/m_track_width;

  m_state << 0, 0, 0, 0, 0, 0;

  m_first_odom_update = false;

  int i = 0;
  for(auto& motor_name: motor_names){
    mp_motors_container[i%4] = std::make_shared<MotorVelsStruct>(motor_name, radi);
    if(i > 3){
      RCLCPP_ERROR(this->get_logger(), "More Than Four Motor Names Defined The Controller Will Not Work !!");
    }
    i++;
  }


  this->initialize_subs();

  this->initialize_motors();

  init_odom_msg();

  init_tf_msg();

  this->initialize_publishers();

}




void sly_controller_node::init_odom_msg(){
  m_odom_update_time = this->get_clock()->now();
  
  m_odom_msg.header.stamp = m_odom_update_time;
  m_odom_msg.header.frame_id = "odom";

  build_odom_msg();

}

void sly_controller_node::init_tf_msg(){
  m_odom_tf.header.stamp = this->get_clock()->now();
  m_odom_tf.header.frame_id = "odom";
  m_odom_tf.child_frame_id = "base_link";

  build_tf_msg();

}


void sly_controller_node::build_tf_msg(){
  m_odom_tf.header.stamp = m_odom_update_time;

  m_odom_tf.transform.translation.x = m_state(0, 0);
  m_odom_tf.transform.translation.y = m_state(1, 0);
  m_odom_tf.transform.translation.z = 0;
  m_odom_tf.transform.rotation.w = m_quat.getW();
  m_odom_tf.transform.rotation.x = m_quat.getX();
  m_odom_tf.transform.rotation.y = m_quat.getY();
  m_odom_tf.transform.rotation.z = m_quat.getZ();



}

void sly_controller_node::initialize_motors(){
  
  for(auto& motor_struct: mp_motors_container){
      motor_struct->m_motor.init(this);
  }

}

void sly_controller_node::build_odom_msg(){
    
    m_odom_msg.header.stamp = this->get_clock()->now();
    m_odom_msg.pose.pose.position.x = m_state(0, 0);
    m_odom_msg.pose.pose.position.y = m_state(1, 0);
    m_odom_msg.pose.pose.position.z = 0;
    m_quat.setRPY(0, 0, m_state(2, 0));
    m_odom_msg.pose.pose.orientation.w = m_quat.getW();
    m_odom_msg.pose.pose.orientation.x = m_quat.getX();
    m_odom_msg.pose.pose.orientation.y = m_quat.getY();
    m_odom_msg.pose.pose.orientation.z = m_quat.getZ();

    m_odom_msg.twist.twist.linear.x = m_state(3, 0);
    m_odom_msg.twist.twist.linear.y = m_state(4, 0);
    m_odom_msg.twist.twist.linear.z = 0;

    m_odom_msg.twist.twist.angular.x = 0;
    m_odom_msg.twist.twist.angular.y = 0;
    m_odom_msg.twist.twist.angular.z = m_state(5, 0);


}


void sly_controller_node::odom_update(){
  Eigen::Vector2d omega(2, 0);
  
  rclcpp::Time t_ = mp_motors_container[0]->m_motor.m_ang_vel_time;
  
  for(int i=0; i<mp_motors_container.size(); i++){
    omega(i%2, 0) += 0.5*mp_motors_container[i]->m_motor.m_angular_velocity;
    t_ = t_ >= mp_motors_container[i]->m_motor.m_ang_vel_time ? mp_motors_container[i]->m_motor.m_ang_vel_time :
    t_;
  }
  double dt = 0;
  if(m_first_odom_update)
  { 
    dt = t_.seconds() - m_odom_update_time.seconds();
    
    if(dt<=0){
      RCLCPP_ERROR(this->get_logger(),"Having an Issue with odom update dt of ang vels is %lf which is unacceptable", dt);
      dt = 0;
    }
  }


  m_odom_update_time = t_;

  Eigen::Vector2d vels = m_omega_maper*omega; //Velocity and Omega
  
  Eigen::Rotation2D<double> rot_matrix(m_state(2, 0));

  Eigen::Vector2d velocity_odom = rot_matrix*Eigen::Vector2d(vels(0, 0), 0);

  m_state(3, 0) = velocity_odom(0, 0);
  
  m_state(4, 0) = velocity_odom(1, 0); //State 3, 4 = velocity x and velocity y

  m_state(5, 0) = vels(1, 0);

  m_state(0, 0) += velocity_odom(0, 0)*dt;

  m_state(1, 0) += velocity_odom(1, 0)*dt;

  m_state(2, 0) += vels(1, 0) * dt;

  m_first_odom_update = true;

}

void sly_controller_node::publish_odom_msg(){
  build_odom_msg();
  static bool first_pub = false;
  if(m_first_odom_update && first_pub){
    mp_odom_pub->publish(m_odom_msg);
  }

  if(m_first_odom_update)
    first_pub = true;

}


void sly_controller_node::publish_tf(){
  
  this->mp_tf_broadcaster->sendTransform(m_odom_tf);

}

void sly_controller_node::odom_loop(){
  this->odom_update();
  this->build_odom_msg();
  this->build_tf_msg();
  this->publish_odom_msg();
  this->publish_tf();
}


void sly_controller_node::initialize_subs(){
  this->mp_desired_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
  std::bind(&sly_controller_node::velocity_callback, this, std::placeholders::_1));
}

void sly_controller_node::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
  m_cmd_vels(0, 0) = msg->linear.x;
  m_cmd_vels(1, 0) = msg->angular.z;
  RCLCPP_DEBUG(this->get_logger(), "Got Vel %lf and Ang Vel %lf", m_cmd_vels(0, 0), m_cmd_vels(1, 0));
  calculate_vels();
}


void sly_controller_node::calculate_vels(){

  auto vels = mp_controller->calculate_control(m_cmd_vels);

  for(int i = 0; i < this->mp_motors_container.size(); i++){
    mp_motors_container[i]->m_desired_ang_vel = vels(i, 0);
  }

  publish_vels();

}

void sly_controller_node::publish_vels(){

  for(auto& motorstruct:mp_motors_container){
    motorstruct->m_motor.publish_desired_ang_vel(motorstruct->m_desired_ang_vel);
  }

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sly_controller_node>());
  rclcpp::shutdown();
  return 0;
}