#include "common.h"

//Starting points
double A1[3] = {(hf * mmtm) / 3, 0, 0};
double A2[3] = {-A1[0] * cos(60 * dtr), -A1[0] * sin(60 * dtr), 0};
double A3[3] = {A2[0], -A2[1], 0};

//Plano XZ
/// @brief punto_codo  (J 1,2,3)
/// @param theta 
/// @param b1 
void punto_codo(double theta, double (&b1)[3]) {
    theta *= dtr;
    b1[0] = A1[0] + rf * mmtm * cos(theta);
    b1[1] = 0.0;
    b1[2] = rf * mmtm * sin(theta);
}

/// @brief rotation120
/// @param ent 
/// @param sal 
void rotation120(double ent[3], double(&sal)[3]) {
    sal[0] = cos120 * ent[0] + sin120 * ent[1];
    sal[1] = -sin120 * ent[0] + cos120 * ent[1];
    sal[2] = ent[2];
}

/// @brief rotation240
/// @param ent 
/// @param sal 
void rotation240(double ent[3], double(&sal)[3]) {
    sal[0] = cos240 * ent[0] + sin240 * ent[1];
    sal[1] = -sin240 * ent[0] + cos240 * ent[1];
    sal[2] = ent[2];
}

/// @brief punto_ee  EE1,2,3 ]
/// @param eee 
/// @param brazo 
/// @param sal 
void punto_ee(int brazo, double eee[3], double (&sal)[3]) {
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
void sum_vector(double v1[3], double v2[3], double (&s)[3]) {
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
void angulos_codo(int brazo, double codo[3], double eee[3], double &ang_a, double &ang_b) {
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
        ang_a = atan((_eee[2] - _codo[2]) / (_codo[0] - _eee[0]));
        if ((_codo[0] - _eee[0]) < 0) ang_a = ang_a + (180 * dtr);
    }
    else {
        cout<<"entra"<<endl;
        ang_a = 1.570796326794897;
    }

    //prep
    rotation_y(_codo, ang_a, _codo);
    rotation_y(_eee, ang_a, _eee);

    //calc
    if ((_codo[0] - _eee[0]) != 0) {
        ang_b = atan((_eee[1] - 0) / (_codo[0] - _eee[0]));
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
void rotation_y(double ent[3], double ang, double(&sal)[3]) {
    sal[0] = ent[0] * cos(ang) - ent[2] * sin(ang);
    sal[1] = ent[1];
    sal[2] = -ent[0] * sin(ang) + ent[2] * cos(ang);
}
