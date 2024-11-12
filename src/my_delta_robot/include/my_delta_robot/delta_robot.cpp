#include "delta_robot.h"

/// @brief 
/// @param  
delta_robot::delta_robot(void) {
    data = new data_delta_t;
}

/// @brief 
/// @param  
delta_robot::~delta_robot(void) {
    delete data;
}

/// @brief 
/// @param point0 
/// @param pointf 
/// @return 
Point delta_robot::unit_vector(Point point0, Point pointf) {
    Point retrun_point;
    double delta_x = pointf.x - point0.x;
    double delta_y = pointf.y - point0.y;
    double delta_z = pointf.z - point0.z;

    double modulo = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

    retrun_point.x = delta_x / modulo;
    retrun_point.y = delta_y / modulo;
    retrun_point.z = delta_z / modulo;
    return retrun_point;
}

// void delta_robot::path_linear_speed(double xo,double yo,double zo,double xf,double yf,double zf)
//{
//    //######  international units change  ##########
//    double xoo = xo * mmtm;
//    double yoo = yo * mmtm;
//    double zoo = zo * mmtm;
//    double xff = xf * mmtm;
//    double yff = yf * mmtm;
//    double zff = zf * mmtm;
//
//    system_linear(xoo, yoo, zoo, xff, yff, zff);
//
// }

/// @brief 
/// @param point0 
/// @param pointf 
/// @param dis 
/// @param rot_z 
/// @param rot_y 
/// @param theta_y 
/// @param theta_z 
/// @param rot_tras 
void delta_robot::system_linear(
    Point point0, 
    Point pointf,
    double &dis, 
    double (&rot_z)[3][3], 
    double (&rot_y)[3][3], 
    double &theta_y, 
    double &theta_z, 
    double (&rot_tras)[4][4]
) {
    double xoo = point0.x * mmtm;
    double yoo = point0.y * mmtm;
    double zoo = point0.z * mmtm;

    double xff = pointf.x * mmtm;
    double yff = pointf.y * mmtm;
    double zff = pointf.z * mmtm;

    double pf[3][1] = {0, 0, 0};
    double pf_trans[4][1] = {0, 0, 0, 0};
    double x_trans[4][1] = {0, 0, 0, 0};

    // Traslacion(Xo, Yo, Zo)
    rot_tras[0][0] = 1;
    rot_tras[1][1] = 1;
    rot_tras[2][2] = 1;
    rot_tras[3][3] = 1;
    rot_tras[0][3] = -xoo;
    rot_tras[1][3] = -yoo;
    rot_tras[2][3] = -zoo;

    // Traslacion(Xf, Yf, Zf)
    pf_trans[0][0] = xff;
    pf_trans[1][0] = yff;
    pf_trans[2][0] = zff;
    pf_trans[3][0] = 1;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            x_trans[i][0] += rot_tras[i][j] * pf_trans[j][0];
        }
    }

    double nx, ny, nz;
    Point my_unit_vector;
    Point temp_point;
    Point temp_point1;
    temp_point.init();
    temp_point1.init();
    temp_point1.x = x_trans[0][0];
    temp_point1.y = x_trans[1][0];
    temp_point1.z = x_trans[2][0];
    my_unit_vector = unit_vector(temp_point, temp_point1);
    angle_rotation(nx, ny, nz, theta_z, theta_y);

    double cos_axisz = cos(theta_y);
    double sin_axisz = sin(theta_y);

    double cos_axisy = cos(theta_z);
    double sin_axisy = sin(theta_z);

    // plano xy ya trasladado
    pf[0][0] = x_trans[0][0];
    pf[1][0] = x_trans[1][0];
    pf[2][0] = x_trans[2][0];
    // rotacion respecto a eje z
    rot_z[0][0] = cos_axisz;
    rot_z[0][1] = sin_axisz;
    rot_z[1][0] = -1 * sin_axisz;
    rot_z[1][1] = cos_axisz;
    rot_z[2][2] = 1;

    double m_rot1[3][1] = {};
    double m_rot2[3][1] = {};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m_rot1[i][0] += rot_z[i][j] * pf[j][0];
        }
    }

    // rotacion respecto a eje y
    rot_y[0][0] = cos_axisy;
    rot_y[0][2] = sin_axisy;
    rot_y[1][1] = 1;
    rot_y[2][0] = -sin_axisy;
    rot_y[2][2] = cos_axisy;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m_rot2[i][0] += rot_y[i][j] * m_rot1[j][0];
        }
    }

    dis = m_rot2[0][0] * mtmm;
    theta_z = theta_z * rtd;
    theta_y = theta_y * rtd;
}

/// @brief 
/// @param nx 
/// @param ny 
/// @param nz 
/// @param theta_z 
/// @param theta_y 
void delta_robot::angle_rotation(double nx, double ny, double nz, double &theta_z, double &theta_y) {
    if (nz < 0)
        theta_z = (360 * dtr) + asin(nz);
    else
        theta_z = asin(nz);

    if (nx < 0) {
        theta_y = (180 * dtr) + (atan(ny / nx));
    }
    else if (nx == 0) {
        if (ny < 0)
            theta_y = (270 * dtr);
        else if (ny == 0)
            theta_y = 0;
        else
            theta_y = (90 * dtr);
    }
    else {
        if (ny < 0) {
            theta_y = (360 * dtr) + (atan(ny / nx));
        }
        else {
            theta_y = (atan(ny / nx));
        }
    }
}

/// @brief 
/// @param q0 
/// @param q1 
/// @param pas_1 total points for the acelerate and deacelerate
/// @param pas_2 total points during reach vmax
void delta_robot::trapezoidal_velocity_profile(double q0, double q1, int pas_1, int pas_2) {
    try {
        q0 *= mmtm;
        q1 *= mmtm;
        vmax *= mmtm;
        amax *= mmtm;

        if (pas_1 <= 0)
            pas_1 = 2;
        if (pas_2 <= 0)
            pas_2 = 2;

        double tau = vmax / amax;

        double T = 0;
        if ((q1 - q0) > 0)
            T = (q1 - q0) / vmax + tau;
        else if ((q1 - q0) < 0)
            T = (q0 - q1) / vmax + tau;

        double paso1 = tau / pas_1;
        double paso2 = (T - (2 * tau)) / pas_2;

        int pas_total = pas_1 + pas_2 + pas_1;

        double Tf = 2 * (sqrt((q1 - q0) / (amax)));
        double vmax_acel = amax * (Tf / 2);

        if (vmax_acel <= vmax) {
            cout << "case vmax_acel <= vmax we have" << endl;
            vmax = vmax_acel;
            tau = Tf / 2;
            T = Tf;
            paso1 = tau / pas_1;
            paso2 = 0;
            pas_2 = 0;
            pas_total = pas_1 + pas_2 + pas_1;
        }

        for (int i = 0; i < pas_total; i++) {
            data->pos = 0;
            data->position_val.init();
            data->theta_val.init();
            data->acel = 0;
            data->vel = 0;
            data->time_point = 0;
            m_data_delta.push_back(data);
        }

        double ti = -1 * paso1;
        double q_actual, v_actual, a_actual;

        for (int i = 0; i < pas_total; i++) {
            if (i >= 0 && i < pas_1)
                ti += paso1;
            else if (i >= pas_1 && i < (pas_1 + pas_2))
                ti += paso2;
            else if (i >= (pas_1 + pas_2) && i < (pas_total))
                ti += paso1;
            else
                ti = T;

            ls_v_a_puntual(q0, q1, ti, q_actual, v_actual, a_actual);
            m_data_delta[i]->pos = q_actual;
            m_data_delta[i]->vel = v_actual;
            m_data_delta[i]->acel = a_actual;
            m_data_delta[i]->time_point = ti;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}

/// @brief 
/// @param q0 
/// @param q1 
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
/// @param time_point 
/// @param rot_z 
/// @param rot_y 
/// @param theta_y 
/// @param theta_z 
/// @param m_trans 
void delta_robot::system_linear_matrix(
    double time_point, 
    double rot_z[3][3], 
    double rot_y[3][3], 
    double theta_y, 
    double theta_z, 
    double m_trans[4][4]
) {
    theta_y = theta_y * dtr;
    theta_z = theta_z * dtr;

    for (int i = 0; i < m_data_delta.size(); i++) {
        //   ######  posicion xyz  ##########
        // stem_linear_invese(m_data_delta[i]->pos, value_x, value_y, value_z);
        double xyz_res[4][1] = {0, 0, 0, 0};
        system_linear_invese(m_data_delta[i]->pos, rot_z, rot_y, theta_y, theta_z, m_trans, xyz_res);

        m_data_delta[i]->position_val.x = xyz_res[0][0];
        m_data_delta[i]->position_val.y = xyz_res[1][0];
        m_data_delta[i]->position_val.z = xyz_res[2][0];

        /*
        // ######  velocidad xyz  ##########
        system_linear_invese(m_data_delta[i]->vel, rot_z, rot_y, theta_y, theta_z, m_trans, xyz_res);
        m_data_delta[i]->vel_x = xyz_res[0][0];
        m_data_delta[i]->vel_y = xyz_res[1][0];
        m_data_delta[i]->vel_z = xyz_res[2][0];

        // ######  aceleracion xyz  ##########
        system_linear_invese(m_data_delta[i]->acel, rot_z, rot_y, theta_y, theta_z, m_trans, xyz_res);
        m_data_delta[i]->acel_x = xyz_res[0][0];
        m_data_delta[i]->acel_y = xyz_res[1][0];
        m_data_delta[i]->acel_z = xyz_res[2][0];
        */
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
void delta_robot::system_linear_invese(
    double xprima, double rot_z[3][3], 
    double rot_y[3][3], double theta_y, 
    double theta_z,
    double m_trans[4][4], 
    double (&xyz_res)[4][1]
) {
    double rot_tras_invese[4][4] = {};
    double pf_invese[3][1] = {};
    double pf_trans_invese[4][1] = {};
    double m_trans_invese[4][4] = {};
    // ######   Creacion de Matrices  ######

    // ######   Rotacion Y  ######
    pf_invese[0][0] = xprima;
    pf_invese[1][0] = 0;
    pf_invese[2][0] = 0;

    double rot_y_tras[3][3] = {};
    // rot_y_tras = rot_y.transpose()
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rot_y_tras[j][i] = rot_y[i][j];
        }
    }

    double m_rot1_invese[3][1] = {};
    // m_rot1_invese = rot_y_tras.dot(pf_invese)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m_rot1_invese[i][0] += rot_y_tras[i][j] * pf_invese[j][0];
        }
    }
    // ######   Rotacion Z  ######
    // rot_z_tras = rot_z.transpose()
    double rot_z_tras[3][3] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rot_z_tras[j][i] = rot_z[i][j];
        }
    }

    // m_rot2_invese = rot_z_tras.dot(m_rot1_invese)
    double m_rot2_invese[3][1] = {};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m_rot2_invese[i][0] += rot_z_tras[i][j] * m_rot1_invese[j][0];
        }
    }

    // ######   Traslacion ######
    pf_trans_invese[0][0] = m_rot2_invese[0][0];
    pf_trans_invese[1][0] = m_rot2_invese[1][0];
    pf_trans_invese[2][0] = m_rot2_invese[2][0];
    pf_trans_invese[3][0] = 1;

    m_trans_invese[0][3] = -1 * m_trans[0][3];
    m_trans_invese[1][3] = -1 * m_trans[1][3];
    m_trans_invese[2][3] = -1 * m_trans[2][3];
    m_trans_invese[0][0] = 1;
    m_trans_invese[1][1] = 1;
    m_trans_invese[2][2] = 1;
    m_trans_invese[3][3] = 1;

    // xyz_res = (m_trans_invese).dot(pf_trans_invese)
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            xyz_res[i][0] += m_trans_invese[i][j] * pf_trans_invese[j][0];
        }
    }
}

/// @brief 
/// @param point0 
/// @return 
double delta_robot::delta_calcAngleYZ(Point point0) {
    double y1 = -0.5 * 0.57735 * ff; // f/2 * tg 30
    point0.y -= 0.5 * 0.57735 * ee;        // shift center to edge
                                     // z = a + b*y
    double a = (point0.x * point0.x + point0.y * point0.y + point0.z * point0.z + rf * rf - re * re - y1 * y1) / (2 * point0.z);
    double b = (y1 - point0.y) / point0.z;

    // discriminant
    double d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);
    if (d < 0) {
        return 0; // non-existing point
    }
    double yj = (y1 - a * b - sqrt(d)) / (b * b + 1); // choosing outer point
    double zj = a + b * yj;
    double theta = 180.0 * atan(-zj / (y1 - yj)) / pi + ((yj > y1) ? 180.0 : 0.0);
    return theta;
}

/// @brief inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
/// @param point0 
/// @return 
Theta delta_robot::delta_calcInverse(Point point0) {
    Theta return_value;
    Point point_temp;
    return_value.angle1 = delta_calcAngleYZ(point0);

    point_temp.x = point0.x * cos120 + point0.y * sin120;
    point_temp.y = point0.y * cos120 - point0.x * sin120;
    point_temp.z = point0.z;
    return_value.angle2 = delta_calcAngleYZ(point_temp); // rotate  to +119 deg

    point_temp.x = point0.x * cos120 - point0.y * sin120;
    point_temp.y = point0.y * cos120 + point0.x * sin120;
    point_temp.z = point0.z;
    return_value.angle3 = delta_calcAngleYZ(point_temp); // rotate to -120 deg
}

/// @brief 
/// @param theta 
/// @return 
Point delta_robot::delta_calcForward(Theta theta) {
    Point return_point;
    double t = (ff - ee) * tan30 / 2;

    theta.angle1 *= dtr;
    theta.angle2 *= dtr;
    theta.angle3 *= dtr;

    double y1 = -(t + rf * cos(theta.angle1));
    double z1 = -rf * sin(theta.angle1);

    double y2 = (t + rf * cos(theta.angle2)) * sin30;
    double x2 = y2 * tan60;
    double z2 = -rf * sin(theta.angle2);

    double y3 = (t + rf * cos(theta.angle3)) * sin30;
    double x3 = -y3 * tan60;
    double z3 = -rf * sin(theta.angle3);

    double dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

    double w1 = y1 * y1 + z1 * z1;
    double w2 = x2 * x2 + y2 * y2 + z2 * z2;
    double w3 = x3 * x3 + y3 * y3 + z3 * z3;

    // x = (a1*z + b1)/dnm
    double a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
    double b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

    // y = (a2*z + b2)/dnm;
    double a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
    double b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

    // a*z^2 + b*z + c = 0
    double a = a1 * a1 + a2 * a2 + dnm * dnm;
    double b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
    double c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re);

    // discriminant
    double d = b * b - (double)4.0 * a * c;
    if (d < 0) {
        return; // non-existing point
    }

    return_point.z = -0.5 * (b + sqrt(d)) / a;
    return_point.x = (a1 * return_point.z + b1) / dnm;
    return_point.y = (a2 * return_point.z + b2) / dnm;

    return return_point;
}

/// @brief 
/// @param  
void delta_robot::inverse_m(void) {
    for (int i = 0; i < m_data_delta.size(); i++) {
        m_data_delta[i]->theta_val = inverse(m_data_delta[i]->position_val);
    }
}

/// @brief 
/// @param point0 
/// @return 
Theta delta_robot::inverse(Point point0) {
    Theta theta;
    Point point_temp;

    // Rotacion(0, 120, 240)
    theta.angle2 = angle_yz(point0);

    point_temp.x = point0.x * cos120 + point0.y * sin120;
    point_temp.y = point0.y * cos120 - point0.x * sin120;
    point_temp.z = point0.z;
    theta.angle3 = angle_yz(point_temp); // rotate  to +119 deg

    point_temp.x = point0.x * cos120 - point0.y * sin120;
    point_temp.y = point0.y * cos120 + point0.x * sin120;
    point_temp.z = point0.z;
    theta.angle1 = angle_yz(point_temp); // rotate to -120 deg

    return theta;
}

/// @brief 
/// @param x0 
/// @param y0 
/// @param z0 
/// @param theta 
/// @return 
double delta_robot::angle_yz(Point point0) {
    double theta = 0;

    double y1 = -0.5 * tan30 * ff * mmtm;
    point0.y = point0.y - 0.5 * tan30 * ee * mmtm;
    // z = a + b * y
    double a = (point0.x * point0.x + point0.y * point0.y + point0.z * point0.z + rf * mmtm * rf * mmtm - re * mmtm * re * mmtm - y1 * y1) / (2.0 * point0.z);
    double b = (y1 - point0.y) / point0.z;

    // discriminante
    // d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf)
    double d = ((-1) * (a + b * y1) * (a + b * y1)) + (rf * mmtm * rf * mmtm * (b * b + 1.0));

    if (d < 0)
    {
        cout << "discriminante - angle_yz" << endl;
        return 1; // error inversa
    }

    double yj = ((y1 - a * b) - sqrt(d)) / (b * b + 1.0);
    double zj = a + b * yj;

    if ((y1 - yj) != 0.0)
    {
        theta = atan(((-1) * (zj)) / (abs(y1 - yj)));
        if (yj < y1)
        {
            theta = theta * (rtd);
        }
        else
        {
            if (zj < 0)
            {
                theta = theta * (180.0 / pi);
                theta = 180.0 - theta;
            }
            else
            {
                theta = theta * (180.0 / pi);
                theta = (-1.0 * 180.0) + theta;
            }
        }
    }
    else
    {
        if (zj < 0)
        {
            theta = 90.0;
        }
        else
        {
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
void delta_robot::angulos_eulerianos(
    double ti, 
    Point pointi,
    Theta theta,
    int gripper, 
    double (&position)[13]
) {
    // Rviz interior angles in Radians
    double punto[3] = {-pointi.y, -pointi.x, -pointi.z};

    double c2[3] = {};
    double p2[3] = {};
    punto_codo(theta.angle2, c2);
    punto_ee(punto, 2, p2);

    double a2_a, a2_b;
    angulos_codo(c2, p2, 2, a2_a, a2_b);

    double c3[3] = {};
    double temp[3] = {};
    double p3[3] = {};
    punto_codo(theta.angle3, temp);
    rotacion120(temp, c3);
    punto_ee(punto, 3, p3);

    double a3_a, a3_b;
    angulos_codo(c3, p3, 3, a3_a, a3_b);

    double c1[3] = {};
    double p1[3] = {};
    punto_codo(theta.angle1, c1);
    rotacion120(c1, temp);
    rotacion120(temp, c1);
    punto_ee(punto, 1, p1);

    double a1_a, a1_b;
    angulos_codo(c1, p1, 1, a1_a, a1_b);

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
    position[9]  = pointi.x;
    position[10] = pointi.y;
    position[11] = pointi.z;
    position[12] = gripper;
}
