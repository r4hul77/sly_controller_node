#include "sly_controller.hpp"
#include "motor.hpp"

MotorVelsStruct::MotorVelsStruct(std::string motor_name, float radius):m_name(motor_name),
m_radi(radius), m_current(0), m_ang_vel(0), m_desired_ang_vel(0),
 m_motor(Motor(motor_name)){}



sly_controller_node::sly_controller_node():Node("sly_controller_node"), mp_motors_container(4){
  this->declare_parameter("motor_names", std::vector<std::string>({"front_right", "front_left",
               "rear_left", "rear_right"}));
  this->declare_parameter("wheel_radius", 0.1651);


}

void sly_controller_node::declare_parameters(){
  
  this->declare_parameter("motor_names", std::vector<std::string>({"front_right", "front_left",
               "rear_left", "rear_right"}));
  
  this->declare_parameter("wheel_radius", 0.1651);

  this->declare_parameter("max_angular_velocity", 0.5);

  this->declare_parameter("wheel_base", 0.6096);

  this->declare_parameter("track_width", 0.3556);
}


void sly_controller_node::init(){
  
  this->declare_parameters();

  auto motor_names = this->get_parameter("motor_names").as_string_array();

  auto radi = this->get_parameter("wheel_radius").as_double();

  this->m_max_angular_velocity  = this->get_parameter("max_angular_velocity").as_double();

  this->m_wheel_base = this->get_parameter("wheel_base").as_double();

  this->m_track_width = this->get_parameter("track_width").as_double();

  int i = 0;
  for(auto& motor_name: motor_names){
    mp_motors_container[i%4] = std::make_shared<MotorVelsStruct>(motor_name, radi);
    i++;
    if(i > 3){
      RCLCPP_ERROR(this->get_logger(), "More Than Four Motor Names Defined The Controller Will Not Work !!");
    }
  }

  this->initialize_subs();

  this->initialize_motors();

}

void sly_controller_node::initialize_motors(){
  
  for(auto& motor_struct: mp_motors_container){
      motor_struct->m_motor.init(this);
  }

}


void sly_controller_node::initialize_subs(){
  this->mp_desired_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
  std::bind(&sly_controller_node::velocity_callback, this, std::placeholders::_1));
}

void sly_controller_node::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
  desired_vel = msg->linear.x;
  desired_ang_vel = msg->angular.z;
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world sly_controller package\n");
  return 0;
}
