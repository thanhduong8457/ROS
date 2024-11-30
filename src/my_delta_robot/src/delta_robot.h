#ifndef __DELTA_ROBOT__
#define __DELTA_ROBOT__

#include "delta_define.h"
#include "common.h"

typedef struct Point {
    double x;
    double y;
    double z;
    void init(void) {
        x = 0;
        y = 0;
        z = 0;
    }
}point_t;

typedef struct Theta {
    double angle1;
    double angle2;
    double angle3;
    void init(void) {
        angle1 = 0;
        angle2 = 0;
        angle3 = 0;
    }
}point_t;

typedef struct data_delta {
    double pos;     //
    Point position_val;

    double vel;     //
    //double vel_x;
    //double vel_y;
    //double vel_z;

    double acel;    //
    //double acel_x;
    //double acel_y;
    //double acel_z;

    Theta theta_val;

    double time_point;  //
}data_delta_t;

class delta_robot{
private:
    double delta_calcAngleYZ(Point point0);
    Point unit_vector(Point point0, Point pointf);
    void angle_rotation(double nx, double ny, double nz, double& theta_y, double& theta_z);
    Theta inverse(Point point0);
    double angle_yz(Point point0);
    void system_linear_invese(double xprima, double rot_z[3][3], 
    double rot_y[3][3], double theta_y, double theta_z, double m_trans[4][4], double(&xyz_res)[4][1]);

public:
    double vmax;
    double amax;
    
    data_delta_t *data = NULL;
    std::vector<data_delta_t*> m_data_delta;

    delta_robot(void);
    ~delta_robot(void);

    //void path_linear_speed(double xo, double yo, double zo, double xf, double yf, double zf);
    //void system_linear(double xo, double yo, double zo, double xf, double yf, double zf);

    void trapezoidal_velocity_profile(double q0, double q1, int pas_1, int pas_2);
    void ls_v_a_puntual(double q0, double q1, double tactual, double &q_actual, double &v_actual, double &a_actual);

    void system_linear(
        Point point0, 
        Point pointf, 
        double& dis, double(&rot_z)[3][3], 
        double(&rot_y)[3][3], 
        double& theta_y, 
        double& theta_z, 
        double(&rot_tras)[4][4]
    );

    void system_linear_matrix(double time_point, double rot_z[3][3], double rot_y[3][3], double theta_y, double theta_z, double m_trans[4][4]);
    Theta delta_calcInverse(Point point0);
    Point delta_calcForward(Theta theta);
    void angulos_eulerianos(double ti, Point point, Theta theta, int gripper, double(&position)[13]);

    void inverse_m(void);
};
#endif
