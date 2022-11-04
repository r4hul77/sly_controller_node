#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

class SlyEstimator{

    Eigen::Matrix<double, 6, 6> m_estimator_mapper;

    Eigen::Vector3d m_state;

    Eigen::Vector3d m_state_dot;

    double m_radius;

    double m_track_width;

public:

    SlyEstimator();

    SlyEstimator(double, double);

    Eigen::Matrix<double, 6, 1> estimate(Eigen::Vector4d&, double);

    void set_radius(float&);

    void set_track_width(float&);

    void build_mapper();

    void clear_state();

};