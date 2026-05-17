#include "delta_robot.h"
#include "joint_state_config.hpp"

/// @brief 
/// @param  
delta_robot::delta_robot(void) {
    initialize();
}

/// @brief 
/// @param  
delta_robot::~delta_robot(void) {
    m_data_delta.clear();
}

/// @brief 
/// @param  
void delta_robot::initialize(void) {
    A1[0] = (hf * mmtm) / 3;
    A1[1] = 0;
    A1[2] = 0;
    A2[0] = -A1[0] * std::cos(60 * dtr);
    A2[1] = -A1[0] * std::sin(60 * dtr);
    A2[2] = 0;
    A3[0] = A2[0];
    A3[1] = -A2[1];
    A3[2] = 0;
    this->vmax = 1500;
    this->amax = 200000;
    this->mResolution = 120;
    m_data_delta.clear();
}

/// @brief 
/// @param point0 
/// @param pointf 
/// @return 
Point delta_robot::unit_vector(Point point0, Point pointf) {
    Point return_point;
    double denta_x = pointf.x - point0.x;
    double denta_y = pointf.y - point0.y;
    double denta_z = pointf.z - point0.z;

    double modulo = std::sqrt(denta_x * denta_x + denta_y * denta_y + denta_z * denta_z);

    return_point.x = denta_x / modulo;
    return_point.y = denta_y / modulo;
    return_point.z = denta_z / modulo;
    return return_point;
}

/// @brief 
/// @param mStartPoint 
/// @param mEndPoint 
/// @param dis 
/// @param rot_z 
/// @param rot_y 
/// @param theta_y 
/// @param theta_z 
/// @param rot_tras 
void delta_robot::system_linear(void) {
    initialize();

    double xoo = mStartPoint.x * mmtm;
    double yoo = mStartPoint.y * mmtm;
    double zoo = mStartPoint.z * mmtm;

    double xff = mEndPoint.x * mmtm;
    double yff = mEndPoint.y * mmtm;
    double zff = mEndPoint.z * mmtm;

    double pf[3] = {0, 0, 0};
    double pf_trans[4] = {0, 0, 0, 0};
    double x_trans[4] = {0, 0, 0, 0};

    // Traslacion(Xo, Yo, Zo)
    rot_tras[0][0] = 1;
    rot_tras[1][1] = 1;
    rot_tras[2][2] = 1;
    rot_tras[3][3] = 1;
    rot_tras[0][3] = -xoo;
    rot_tras[1][3] = -yoo;
    rot_tras[2][3] = -zoo;
    //[1  0  0 -xoo]
    //[0  1  0 -yoo]
    //[0  0  1 -zoo]
    //[0  0  0   1 ]

    // Traslacion(Xf, Yf, Zf)
    pf_trans[0] = xff;
    pf_trans[1] = yff;
    pf_trans[2] = zff;
    pf_trans[3] = 1;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            x_trans[i] += rot_tras[i][j] * pf_trans[j];
        }
    }

    Point my_unit_vector;
    Point temp_point;
    Point temp_point1;
    temp_point.init();
    temp_point1.init();
    temp_point1.x = x_trans[0];
    temp_point1.y = x_trans[1];
    temp_point1.z = x_trans[2];
    my_unit_vector = unit_vector(temp_point, temp_point1);
    angle_rotation(my_unit_vector);

    double cos_axisz = std::cos(this->theta_y);
    double sin_axisz = std::sin(this->theta_y);

    double cos_axisy = std::cos(this->theta_z);
    double sin_axisy = std::sin(this->theta_z);

    pf[0] = x_trans[0];
    pf[1] = x_trans[1];
    pf[2] = x_trans[2];

    rot_z[0][0] = cos_axisz;
    rot_z[0][1] = sin_axisz;
    rot_z[1][0] = -1 * sin_axisz;
    rot_z[1][1] = cos_axisz;
    rot_z[2][2] = 1;

    double m_rot1[3] = {};
    double m_rot2[3] = {};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m_rot1[i] += rot_z[i][j] * pf[j];
        }
    }

    rot_y[0][0] = cos_axisy;
    rot_y[0][2] = sin_axisy;
    rot_y[1][1] = 1;
    rot_y[2][0] = -sin_axisy;
    rot_y[2][2] = cos_axisy;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m_rot2[i] += rot_y[i][j] * m_rot1[j];
        }
    }

    dis = m_rot2[0] * mtmm;
    theta_z = theta_z * rtd;
    theta_y = theta_y * rtd;
}

/// @brief 
/// @param nx 
/// @param ny 
/// @param nz 
/// @param theta_z 
/// @param theta_y 
void delta_robot::angle_rotation(Point unit_vector) {
    if (unit_vector.z < 0) {
        theta_z = (360 * dtr) + std::asin(unit_vector.z);
    } else {
        theta_z = std::asin(unit_vector.z);
    }

    if (unit_vector.x < 0) {
        theta_y = (180 * dtr) + std::atan(unit_vector.y / unit_vector.x);
    } else if (unit_vector.x == 0) {
        if (unit_vector.y < 0) {
            theta_y = 270 * dtr;
        } else if (unit_vector.y == 0) {
            theta_y = 0;
        } else {
            theta_y = 90 * dtr;
        }
    } else {
        if (unit_vector.y < 0) {
            theta_y = (360 * dtr) + std::atan(unit_vector.y / unit_vector.x);
        } else {
            theta_y = std::atan(unit_vector.y / unit_vector.x);
        }
    }
}

// /// @brief 
// /// @param x0 
// /// @param y0 
// /// @param z0 
// /// @return 
// double delta_robot::delta_calcAngleYZ(double x0, double y0, double z0) {
//     double y1 = -0.5 * 0.57735 * ff; // f/2 * tg 30
//     y0 -= 0.5 * 0.57735 * ee;        // shift center to edge
//                                      // z = a + b*y
//     double a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0);
//     double b = (y1 - y0) / z0;

//     // discriminant
//     double d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);
//     if (d < 0) {
//         return -1; // non-existing point
//     }
//     double yj = (y1 - a * b - sqrt(d)) / (b * b + 1); // choosing outer point
//     double zj = a + b * yj;
//     double theta = 180.0 * atan(-zj / (y1 - yj)) / pi + ((yj > y1) ? 180.0 : 0.0);
//     return theta;
// }

/// @brief 
/// @param  
void delta_robot::TrapezoidalVelocityProfile(void) {
    try {
        double q0 = 0; // start from zero
        q0 *= mmtm;
        dis *= mmtm;
        vmax *= mmtm;
        amax *= mmtm;

        if (mResolution <= 0) mResolution = 2;

        double tau = vmax / amax;

        double T = 0;
        if ((this->dis - q0) > 0) {
            T = (this->dis - q0) / vmax + tau;
        }
        else if ((this->dis - q0) < 0) {
            T = (q0 - this->dis) / vmax + tau;
        }

        double paso1 = tau / mResolution;
        double paso2 = (T - (2 * tau)) / mResolution;
        unsigned int pas_total = mResolution + mResolution + mResolution;

        double Tf = 2 * (std::sqrt((this->dis - q0) / (amax)));
        double vmax_acel = amax * (Tf / 2);

        if (vmax_acel <= vmax) {
            vmax = vmax_acel;
            tau = Tf / 2;
            T = Tf;
            paso1 = tau / mResolution;
            paso2 = 0;
            mResolution = 0;
            pas_total = mResolution + mResolution + mResolution;
        }

        for (unsigned int i = 0; i < pas_total; i++) {
            auto data = std::make_unique<data_delta_t>();
            data->pos = 0;
            data->position_val.init();
            data->theta_val.init();
            data->acel = 0;
            data->vel = 0;
            data->time_point = 0;
            m_data_delta.push_back(std::move(data));
        }

        double ti = -1 * paso1;
        double q_actual, v_actual, a_actual;

        for (unsigned int i = 0; i < pas_total; i++) {
            if (i < mResolution) {
                ti += paso1;
            }
            else if (i < (mResolution + mResolution)) {
                ti += paso2;
            }
            else if (i < pas_total) {
                ti += paso1;
            }
            else {
                ti = T;
            }

            ls_v_a_puntual(q0, dis, ti, q_actual, v_actual, a_actual);
            m_data_delta[i]->pos = q_actual;
            m_data_delta[i]->vel = v_actual;
            m_data_delta[i]->acel = a_actual;
            m_data_delta[i]->time_point = ti;
            // cout << "dis=" << q_actual << ", v=" << v_actual << ", a_actual=" << a_actual << ", time_point=" << ti << endl;
        }
    }
    catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
}

/// @brief 
/// @param q0 
/// @param dis 
/// @param vmax 
/// @param amax 
/// @param tactual 
/// @param q_actual 
/// @param v_actual 
/// @param a_actual 
void delta_robot::ls_v_a_puntual(
    double q0, 
    double q1,
    double tactual, 
    double &q_actual, 
    double &v_actual, 
    double &a_actual
) {
    double tau = this->vmax / this->amax;

    double T;
    int s = 0;

    if (q1 - q0 > 0) {
        s = 1;
    }
    else if (q1 - q0 < 0) {
        s = -1;
    }

    T = s * (q1 - q0) / vmax + tau;

    double timer1 = tau;
    double timer2 = T - tau;
    double timer3 = T;

    if ((0 <= tactual) && (tactual <= timer1)) {
        q_actual = (q0) + ((s) * (amax / 2) * (tactual * tactual));
        v_actual = s * amax * tactual;
        a_actual = s * amax;
    }
    else if ((timer1 < tactual) && (tactual <= timer2)) {
        q_actual = (q0) - ((s) * ((vmax * vmax) / (2 * amax))) + (s * vmax * tactual);
        v_actual = s * vmax;
        a_actual = 0;
    }
    else if ((timer2 < tactual) && (tactual <= timer3)) {
        q_actual = (q1) + ((s) * ((-1 * ((amax * (T * T)) / (2))) + (amax * T * tactual) + (-1 * (amax / 2) * (tactual * tactual))));
        v_actual = s * ((amax * T) - (amax * tactual));
        a_actual = s * (-1 * amax);
    }
    else {
        q_actual = 0;
        v_actual = 0;
        a_actual = 0;
    }
}

/// @brief 
/// @param  
void delta_robot::system_linear_matrix(void) {
    theta_y = theta_y * dtr;
    theta_z = theta_z * dtr;
    double xyz_res[4] = {0, 0, 0, 0};
    
    for (size_t i = 0; i < m_data_delta.size(); i++) {
        for (int u = 0; u < 4; u++) {
            xyz_res[u] = 0; // init the array
        }
        system_linear_invese(m_data_delta[i]->pos, xyz_res);

        m_data_delta[i]->position_val.x = xyz_res[0];
        m_data_delta[i]->position_val.y = xyz_res[1];
        m_data_delta[i]->position_val.z = xyz_res[2];
        // cout << "x=" << xyz_res[0] << ", y=" << xyz_res[1] << ", z=" << xyz_res[2] << endl;
        
        // // ######  velocidad xyz  ########## 未来用
        // system_linear_invese(m_data_delta[i]->vel, rot_z, rot_y, theta_y, theta_z, rot_tras, xyz_res);
        // m_data_delta[i]->vel_x = xyz_res[0][0];
        // m_data_delta[i]->vel_y = xyz_res[1][0];
        // m_data_delta[i]->vel_z = xyz_res[2][0];

        // // ######  aceleracion xyz  ########## 未来用
        // system_linear_invese(m_data_delta[i]->acel, rot_z, rot_y, theta_y, theta_z, rot_tras, xyz_res);
        // m_data_delta[i]->acel_x = xyz_res[0][0];
        // m_data_delta[i]->acel_y = xyz_res[1][0];
        // m_data_delta[i]->acel_z = xyz_res[2][0];
    }
}

/// @brief 
/// @param xprima 
/// @param rot_z 
/// @param rot_y 
/// @param theta_y 
/// @param theta_z 
/// @param m_trans 
/// @param xyz_res 
void delta_robot::system_linear_invese(double xprima, double (&xyz_res)[4]) {
    double rot_tras_invese[4][4] = {};
    double pf_invese[3] = {};
    double pf_trans_invese[4] = {};
    double m_trans_invese[4][4] = {};
    // ######   Rotacion Y  ######
    pf_invese[0] = xprima;
    pf_invese[1] = 0;
    pf_invese[2] = 0;

    double rot_y_tras[3][3] = {};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rot_y_tras[j][i] = rot_y[i][j];
        }
    }

    double m_rot1_invese[3] = {};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m_rot1_invese[i] += rot_y_tras[i][j] * pf_invese[j];
        }
    }
    // ######   Rotacion Z  ######
    double rot_z_tras[3][3] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rot_z_tras[j][i] = rot_z[i][j];
        }
    }

    double m_rot2_invese[3] = {};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m_rot2_invese[i] += rot_z_tras[i][j] * m_rot1_invese[j];
        }
    }

    pf_trans_invese[0] = m_rot2_invese[0];
    pf_trans_invese[1] = m_rot2_invese[1];
    pf_trans_invese[2] = m_rot2_invese[2];
    pf_trans_invese[3] = 1;

    m_trans_invese[0][3] = -1 * rot_tras[0][3];
    m_trans_invese[1][3] = -1 * rot_tras[1][3];
    m_trans_invese[2][3] = -1 * rot_tras[2][3];
    m_trans_invese[0][0] = 1;
    m_trans_invese[1][1] = 1;
    m_trans_invese[2][2] = 1;
    m_trans_invese[3][3] = 1;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            xyz_res[i] += m_trans_invese[i][j] * pf_trans_invese[j];
        }
    }
}

// /// @brief 
// /// @param mStartPoint 
// /// @return 
// double delta_robot::CalculateAngleYZ(Point mStartPoint) {
//     double y1 = -0.5 * 0.57735 * ff; // f/2 * tg 30
//     mStartPoint.y -= 0.5 * 0.57735 * ee; // shift center to edge
//                                      // z = a + b*y
//     double a = (mStartPoint.x * mStartPoint.x + mStartPoint.y * mStartPoint.y + mStartPoint.z * mStartPoint.z + rf * rf - re * re - y1 * y1) / (2 * mStartPoint.z);
//     double b = (y1 - mStartPoint.y) / mStartPoint.z;

//     // discriminant
//     double d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);
//     if (d < 0) {
//         return 0; // non-existing point
//     }
//     double yj = (y1 - a * b - sqrt(d)) / (b * b + 1); // choosing outer point
//     double zj = a + b * yj;
//     double theta = 180.0 * atan(-zj / (y1 - yj)) / pi + ((yj > y1) ? 180.0 : 0.0);
//     return theta;
// }

/// @brief 
/// @param  
void delta_robot::inverse_all_joint_state_exist(void) {
    for (size_t i = 0; i < m_data_delta.size(); i++) {
        m_data_delta[i]->theta_val = inverse(m_data_delta[i]->position_val);
    }
}

/// @brief 
/// @param vmax 
/// @param amax 
void delta_robot::set_vmax_amax(unsigned int vmax, unsigned int amax) {
    this->vmax = vmax;
    this->amax = amax;
}

/// @brief 
/// @param vmax 
/// @param amax 
void delta_robot::set_resolution(unsigned int resolution) {
    this->mResolution = resolution;
}

/// @brief 
/// @param mStartPoint 
/// @return 
Theta delta_robot::inverse(Point point) {
    Theta theta;
    Point point_temp;

    // Rotacion(0, 120, 240)
    theta.angle2 = angle_yz(point);

    point_temp.x = point.x * cos120 + point.y * sin120;
    point_temp.y = point.y * cos120 - point.x * sin120;
    point_temp.z = point.z;
    theta.angle3 = angle_yz(point_temp); // rotate  to +119 deg

    point_temp.x = point.x * cos120 - point.y * sin120;
    point_temp.y = point.y * cos120 + point.x * sin120;
    point_temp.z = point.z;
    theta.angle1 = angle_yz(point_temp); // rotate to -120 deg

    // cout << "Point=(" << point_temp.x << ", " << point_temp.y << ", " << point_temp.z ;
    // cout << ") -> Thetal=(" << theta.angle1 << ", " << theta.angle2 << ", " << theta.angle3 << ")" << endl;
    
    return theta;
}

/// @brief 
/// @param x0 
/// @param y0 
/// @param z0 
/// @param theta 
/// @return 
double delta_robot::angle_yz(Point point) {
    double theta = 0;

    double y1 = -0.5 * tan30 * ff * mmtm;
    point.y = point.y - 0.5 * tan30 * ee * mmtm;
    // z = a + b * y
    double a = (point.x * point.x + point.y * point.y + point.z * point.z + rf * mmtm * rf * mmtm - re * mmtm * re * mmtm - y1 * y1) / (2.0 * point.z);
    double b = (y1 - point.y) / point.z;

    // discriminante
    // d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf)
    double d = ((-1) * (a + b * y1) * (a + b * y1)) + (rf * mmtm * rf * mmtm * (b * b + 1.0));

    if (d < 0) {
        std::cout << "discriminante - angle_yz" << std::endl;
        return 1; // error inversa
    }

    double yj = ((y1 - a * b) - std::sqrt(d)) / (b * b + 1.0);
    double zj = a + b * yj;

    if ((y1 - yj) != 0.0) {
        theta = std::atan(((-1) * (zj)) / (std::abs(y1 - yj)));
        if (yj < y1) {
            theta = theta * (rtd);
        }
        else {
            if (zj < 0) {
                theta = theta * (180.0 / pi);
                theta = 180.0 - theta;
            }
            else {
                theta = theta * (180.0 / pi);
                theta = (-1.0 * 180.0) + theta;
            }
        }
    }
    else {
        if (zj < 0) {
            theta = 90.0;
        }
        else {
            theta = -90.0;
        }
    }

    return theta;
}

/// @brief 
/// @param ti 
/// @param pointi 
/// @param theta 
/// @param gripper 
/// @param position 
void delta_robot::create_joint_state_list(
    Point pointi,
    Theta theta,
    double (&position)[12]
) {
    // Rviz interior angles in Radians
    double punto[3] = {-pointi.y, -pointi.x, -pointi.z};

    double a1_a = 0;
    double a2_a = 0;
    double a3_a = 0;
    double a1_b = 0;
    double a2_b = 0;
    double a3_b = 0;

    double temp[3] = {0,0,0};
    double c1[3] = {0,0,0};
    double p1[3] = {0,0,0};
    double c2[3] = {0,0,0};
    double p2[3] = {0,0,0};
    double c3[3] = {0,0,0};
    double p3[3] = {0,0,0};

    punto_codo(theta.angle1, temp);
    rotation240(temp, c1);
    punto_ee(1, punto, p1);
    angulos_codo(1, c1, p1, a1_a, a1_b);

    punto_codo(theta.angle2, c2);
    punto_ee(2, punto, p2);
    angulos_codo(2, c2, p2, a2_a, a2_b);

    punto_codo(theta.angle3, temp);
    rotation120(temp, c3);
    punto_ee(3, punto, p3);
    angulos_codo(3, c3, p3, a3_a, a3_b);

    // Data to publish in Rviz
    position[0]  = theta.angle1 * dtr;
    position[1]  = theta.angle2 * dtr;
    position[2]  = theta.angle3 * dtr;
    position[3]  = theta.angle1 * dtr + a1_a;
    position[4]  = a1_b;
    position[5]  = theta.angle2 * dtr + a2_a;
    position[6]  = a2_b;
    position[7]  = theta.angle3 * dtr + a3_a;
    position[8]  = a3_b;
    delta_robot_config::mapTcpToActuatorJoints(
        pointi.x, pointi.y, pointi.z,
        position[9], position[10], position[11]);

    // cout << "position[0]=" << position[0] << ", position[1]=" << position[1] << ", position[2]=" << position[2] << endl;
    // cout << "position[3]=" << position[3] << ", position[4]=" << position[4] << ", position[5]=" << position[5] << endl;
    // cout << "position[6]=" << position[6] << ", position[7]=" << position[7] << ", position[8]=" << position[8] << endl;
    // cout << "position[9]=" << position[9] << ", position[10]=" << position[10] << ", position[11]=" << position[11] << endl << endl;
}

//Plano XZ
/// @brief punto_codo  (J 1,2,3)
/// @param theta 
/// @param b1 
void delta_robot::punto_codo(double theta, double (&b1)[3]) {
    theta *= dtr;
    b1[0] = A1[0] + rf * mmtm * std::cos(theta);
    b1[1] = 0.0;
    b1[2] = rf * mmtm * std::sin(theta);
}

/// @brief rotation120
/// @param ent 
/// @param sal 
void delta_robot::rotation120(double ent[3], double(&sal)[3]) {
    sal[0] = cos120 * ent[0] + sin120 * ent[1];
    sal[1] = -sin120 * ent[0] + cos120 * ent[1];
    sal[2] = ent[2];
}

/// @brief rotation240
/// @param ent 
/// @param sal 
void delta_robot::rotation240(double ent[3], double(&sal)[3]) {
    sal[0] = cos240 * ent[0] + sin240 * ent[1];
    sal[1] = -sin240 * ent[0] + cos240 * ent[1];
    sal[2] = ent[2];
}

/// @brief punto_ee  EE1,2,3 ]
/// @param eee 
/// @param brazo 
/// @param sal 
void delta_robot::punto_ee(int brazo, double eee[3], double (&sal)[3]) {
    double vhe1[3] = { 0, 0, 0 };
    double vhe2[3] = { (he * mmtm) / 3, 0.0, 0.0 };
    double vhe3[3] = { 0, 0, 0 };

    rotation120(vhe2, vhe3);
    rotation120(vhe3, vhe1);

    if (brazo == 2) {
        sum_vector(eee, vhe2, sal);
    }
    else if (brazo == 3) {
        sum_vector(eee, vhe3, sal);
    }
    else if (brazo == 1) {
        sum_vector(eee, vhe1, sal);
    }
}

/// @brief 
/// @param v1 
/// @param v2 
/// @param s 
void delta_robot::sum_vector(double v1[3], double v2[3], double (&s)[3]) {
    s[0] = v1[0] + v2[0];
    s[1] = v1[1] + v2[1];
    s[2] = v1[2] + v2[2];
}

/// @brief angulos_codo
/// @param codo 
/// @param eee 
/// @param brazo 
/// @param ang_a 
/// @param ang_b 
void delta_robot::angulos_codo(int brazo, double codo[3], double eee[3], double &ang_a, double &ang_b) {
    double _codo[3] = {};
    double _eee[3] = {};
    if (brazo == 3) {
        rotation240(codo, _codo);
        rotation240(eee, _eee);
    }
    else if (brazo == 1) {        
        rotation120(codo, _codo);
        rotation120(eee, _eee);
    }
    else {
        for (int i = 0; i < 3; i++) {
            _codo[i] = codo[i];
            _eee[i] = eee[i];
        }
    }

    if ((_codo[0] - _eee[0]) != 0) {
        ang_a = std::atan((_eee[2] - _codo[2]) / (_codo[0] - _eee[0]));
        if ((_codo[0] - _eee[0]) < 0) ang_a = ang_a + (180 * dtr);
    }
    else {
        std::cout << "entra" << std::endl;
        ang_a = 1.570796326794897;
    }

    //prep
    rotation_y(_codo, ang_a, _codo);
    rotation_y(_eee, ang_a, _eee);

    //calc
    if ((_codo[0] - _eee[0]) != 0) {
        ang_b = std::atan((_eee[1] - 0) / (_codo[0] - _eee[0]));
        if ((_codo[0] - _eee[0]) < 0) {
            ang_a = ang_a + (180 * dtr);
        }
    }
    else { 
        ang_b = 1.570796326794897;
    }
}

/// @brief rotation_y
/// @param ent 
/// @param ang 
/// @param sal 
void delta_robot::rotation_y(double ent[3], double ang, double(&sal)[3]) {
    sal[0] = ent[0] * std::cos(ang) - ent[2] * std::sin(ang);
    sal[1] = ent[1];
    sal[2] = -ent[0] * std::sin(ang) + ent[2] * std::cos(ang);
}
