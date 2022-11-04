#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

class SlyController{

    Eigen::Vector4d m_control_vels;

    Eigen::Vector2d m_target_vels;

    Eigen::Matrix2d m_control_mapper;

    double m_radius;

    double m_track_width;

public:

    SlyController();


    SlyController(double, double);

    Eigen::Vector4d calculate_control(Eigen::Vector2d&);

    void set_radius(float&);

    void set_track_width(float&);

    void build_control_mapper();

};
