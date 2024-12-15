#ifndef __DELTA_ROBOT__
#define __DELTA_ROBOT__

#include "delta_define.h"
#include "common.h"

class Point {
public:
    double x;
    double y;
    double z;

    Point() {
        init();
    }

    bool operator!=(const Point& other) const {
        return this->x != other.x || this->y != other.y || this->z != other.z;
    }

    void operator=(const Point& other) {
        this->x = other.x;
        this->y = other.y;
        this->z = other.z;
    }

    void init() {
        x = 0;
        y = 0;
        z = 0;
    }
};

class Theta {
public:
    double angle1;
    double angle2;
    double angle3;

    Theta() {
        init();
    }

    void operator=(const Theta& other) {
        this->angle1 = other.angle1;
        this->angle2 = other.angle2;
        this->angle3 = other.angle3;
    }

    void init(void) {
        angle1 = 0;
        angle2 = 0;
        angle3 = 0;
    }
};

typedef struct data_delta {
    double pos;     //
    Point position_val;

    double vel;     //
    double vel_x;
    double vel_y;
    double vel_z;

    double acel;    //
    double acel_x;
    double acel_y;
    double acel_z;

    Theta theta_val;

    double time_point;  //
}data_delta_t;

class delta_robot{
private:
    double CalculateAngleYZ(Point);
    Point unit_vector(Point point0, Point pointf);
    void angle_rotation(Point unit_vector);
    double angle_yz(Point point0);
    void system_linear_invese(double xprima, double(&xyz_res)[4][1]);

public:
    double vmax;
    double amax;

    unsigned int num_point_1;
    unsigned int num_point_2;

    double rot_z[3][3];
    double rot_y[3][3];
    double rot_tras[4][4];
    double dis, theta_y, theta_z;

    Point mStartPoint;
    Point mEndPoint;
    
    data_delta_t *data = NULL;
    std::vector<data_delta_t*> m_data_delta;

    delta_robot(void);
    ~delta_robot(void);

    //void path_linear_speed(double xo, double yo, double zo, double xf, double yf, double zf);
    Theta inverse(Point point0);

    void TrapezoidalVelocityProfile(void);
    void ls_v_a_puntual(double q0, double q1, double tactual, double &q_actual, double &v_actual, double &a_actual);

    void system_linear(void);
    void system_linear_matrix(void);
    void CreateJointStateList(Point point, Theta theta, int gripper, double(&position)[13]);
    void InverseAllJointStateExist(void);
};
#endif
