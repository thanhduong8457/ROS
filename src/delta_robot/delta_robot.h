#ifndef __DELTA_ROBOT__
#define __DELTA_ROBOT__

#include "common.h"

class delta_robot{
private:
    double vmax;
    double amax;

    double CalculateAngleYZ(Point);
    Point unit_vector(Point point0, Point pointf);
    void angle_rotation(Point unit_vector);
    double delta_calcAngleYZ(double x0, double y0, double z0);
    double angle_yz(Point point0);
    void system_linear_invese(double xprima, double(&xyz_res)[4]);

public:
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

    void initialize(void);

    Theta inverse(Point point0);

    void TrapezoidalVelocityProfile(void);
    void ls_v_a_puntual(double q0, double q1, double tactual, double &q_actual, double &v_actual, double &a_actual);

    void system_linear(void);
    void system_linear_matrix(void);
    void CreateJointStateList(Point point, Theta theta, double(&position)[13]);
    void InverseAllJointStateExist(void);

    void set_vmax_amax(unsigned int vmax, unsigned int amax);
};
#endif
