#include "sly_estimator.hpp"

SlyEstimator::SlyEstimator(){
    m_radius = 1;
    m_track_width = 1;
    build_mapper();
    clear_state();
}

SlyEstimator::SlyEstimator(double radius, double track_width){
    m_radius = radius;
    m_track_width = track_width;
    clear_state();
}

void SlyEstimator::set_radius(double radius){
    m_radius = radius;
    build_mapper()
}

void SlyEstimator::set_track_width(double track_width){
    m_track_width = track_width;
    build_mapper();
}


void SlyEstimator::clear_state(){
    m_state << 0, 0, 0;
    m_state_dot << 0,0,0;
}

Eigen::Matrix<double, 6, 1>  SlyEstimator::estimate(Eigen::Vector4d cmd_vels, double dt){
    m_state+=m_state_dot*dt;
    m_state_dot = m_estimator_mapper*cmd_vels;

    Eigen::Matrix<double, 6, 1>  ret<< m_state << m_state_dot;

    return ret;
}


void SlyEstimator::build_mapper(){

}