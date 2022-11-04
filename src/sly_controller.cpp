#include "sly_controller.hpp"

void SlyController::build_control_mapper(){
      m_control_mapper << 1/(m_radius), m_track_width/(m_radius),
                          1/m_radius, -m_track_width/m_radius;
}

void SlyController::set_track_width(float& track_width){
    m_track_width = track_width;
    build_control_mapper();
}

void SlyController::set_radius(float& radius){
    m_radius = radius;
    build_control_mapper();
}


Eigen::Vector4d SlyController::calculate_control(Eigen::Vector2d& cmd_vels){
    auto sol = m_control_mapper*cmd_vels;
    Eigen::Vector4d ret;
    for(int i = 0; i< 4; i++){
        ret(i, 0) = sol(i%2, 0);
    }
    return ret;
}

SlyController::SlyController(){
    m_radius = 1;
    m_track_width = 0.5;
    build_control_mapper();
}

SlyController::SlyController(double radius, double track_width){
    m_radius = radius;
    m_track_width = track_width;
}