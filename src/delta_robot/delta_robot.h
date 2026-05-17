#ifndef __DELTA_ROBOT__
#define __DELTA_ROBOT__

#include "common.h"
#include <memory>

class delta_robot{
private:
    double vmax;
    double amax;
    unsigned int mResolution;

    double A1[3] = {(hf * mmtm) / 3, 0, 0};
    double A2[3] = {-A1[0] * cos(60 * dtr), -A1[0] * sin(60 * dtr), 0};
    double A3[3] = {A2[0], -A2[1], 0};

    // double CalculateAngleYZ(Point);
    Point unit_vector(Point point0, Point pointf);
    void angle_rotation(Point unit_vector);
    // double delta_calcAngleYZ(double x0, double y0, double z0);
    double angle_yz(Point point0);
    void system_linear_invese(double xprima, double(&xyz_res)[4]);

    void punto_codo(double theta, double(&b1)[3]);
    void rotation120(double ent[3], double(&sal)[3]);
    void rotation240(double ent[3], double(&sal)[3]);
    void punto_ee(int brazo, double eee[3], double(&sal)[3]);
    void sum_vector(double v1[3], double v2[3], double(&s)[3]);
    void angulos_codo(int brazo, double codo[3], double eee[3], double& ang_a, double& ang_b);
    void rotation_y(double ent[3], double ang, double(&sal)[3]);

public:
    double rot_z[3][3];
    double rot_y[3][3];
    double rot_tras[4][4];
    double dis, theta_y, theta_z;

    Point mStartPoint;
    Point mEndPoint;

    std::vector<std::unique_ptr<data_delta_t>> m_data_delta;

    delta_robot(void);
    ~delta_robot(void);

    void initialize(void);

    Theta inverse(Point point0);

    void TrapezoidalVelocityProfile(void);
    void ls_v_a_puntual(double q0, double q1, double tactual, double &q_actual, double &v_actual, double &a_actual);

    void system_linear(void);
    void system_linear_matrix(void);
    void create_joint_state_list(Point point, Theta theta, double(&position)[12]);
    void inverse_all_joint_state_exist(void);

    void set_vmax_amax(unsigned int vmax, unsigned int amax);
    void set_resolution(unsigned int resolution);
};
#endif
