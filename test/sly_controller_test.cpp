#include "sly_controller.hpp"
#include "gtest/gtest.h"


class ControllerTest: public ::testing::Test{
    protected:
    void SetUp() override{
        float radius(1), track_width(1);
        mp_controller.set_radius(radius);
        mp_controller.set_track_width(track_width);
    }

    SlyController mp_controller;

};

void compare(Eigen::Vector4d v1, Eigen::Vector4d v2){
    
        EXPECT_EQ(v1(0), v2(0));
        EXPECT_EQ(v1(1), v2(1));
        EXPECT_EQ(v1(2), v2(2));
        EXPECT_EQ(v1(3), v2(3));


}

TEST_F(ControllerTest, Radius_TRACK_WIDTH_1){
    float radius(1), track_width(1);
    mp_controller.set_radius(radius);
    mp_controller.set_track_width(track_width);
    Eigen::Vector2d vels(0, 0);
    auto sol = mp_controller.calculate_control(vels);
    compare(sol, Eigen::Vector4d(0,0,0,0));    
    
}

TEST_F(ControllerTest, Radius_TRACK_WIDTH_1_VEL){
    float radius(1), track_width(1);
    mp_controller.set_radius(radius);
    mp_controller.set_track_width(track_width);
    Eigen::Vector2d vels(1, 0);
    auto sol = mp_controller.calculate_control(vels);
    compare(sol, Eigen::Vector4d(1, 1, 1, 1));
    vels(0) = 0;
    vels(1) = 1;
    sol = mp_controller.calculate_control(vels);
    compare(sol, Eigen::Vector4d(1, -1, 1, -1));

}

TEST_F(ControllerTest, Radius_TRACK_WIDTH_2_VEL){
    float radius(0.35), track_width(0.67);
    mp_controller.set_radius(radius);
    mp_controller.set_track_width(track_width);
    Eigen::Vector2d vels(1, 1);
    auto sol = mp_controller.calculate_control(vels);
    compare(sol, Eigen::Vector4d(167./35, 33./35, 167./35, 33./35));  
}