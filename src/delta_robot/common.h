#pragma once

#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>

// Avoid "using namespace std" in headers; use std:: in .cpp files as needed

#define sqrt3   1.732050808
#define pi      3.141592654
#define sin120  0.8660254038
#define cos120  -0.5

#define sin240  -0.8660254038
#define cos240  -0.5

#define tan60   1.732050808
#define sin30   0.5
#define tan30   0.5773502692

#define ee      86.5       // endeffector 
#define ff      346.4      // base
#define re      465        // endeffector arm
#define rf      200        // Base arm

#define hf      299.99119987
#define he      74.911197427

#define mmtm    0.001
#define mtmm    1000
#define dtr     (pi/180)
#define rtd     (180/pi)

class Point {
public:
    double x;
    double y;
    double z;

    Point(double x, double y, double z) {
        init();
        this->x = x;
        this->y = y;
        this->z = z;
    }

    Point() {
        init();
    }
    Point(const Point& other) {
        this->x = other.x;
        this->y = other.y;
        this->z = other.z;
    }

    bool operator==(const Point& other) const {
        return this->x == other.x && this->y == other.y && this->z == other.z;
    }

    bool operator!=(const Point& other) const {
        return !(*this == other);
    }

    Point& operator=(const Point& other) {
        this->x = other.x;
        this->y = other.y;
        this->z = other.z;
        return *this;
    }

    void init() {
        x = 0;
        y = 0;
        z = 0;
    }
};

/// @brief The Theta class represents the angles of the delta robot's arms.
class Theta {
public:
    double angle1;
    double angle2;
    double angle3;

    Theta() {
        init();
    }

    Theta(const Theta& other) {
        this->angle1 = other.angle1;
        this->angle2 = other.angle2;
        this->angle3 = other.angle3;
    }

    Theta& operator=(const Theta& other) {
        this->angle1 = other.angle1;
        this->angle2 = other.angle2;
        this->angle3 = other.angle3;
        return *this;
    }

    void init(void) {
        angle1 = 0;
        angle2 = 0;
        angle3 = 0;
    }
};

/// @brief 
typedef struct data_delta {
    double pos;     //
    Point position_val;

    double vel; //
    double vel_x;
    double vel_y;
    double vel_z;

    double acel; //
    double acel_x;
    double acel_y;
    double acel_z;

    Theta theta_val;

    double time_point; //
}data_delta_t;
