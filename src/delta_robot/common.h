#pragma once

inline constexpr double sqrt3 = 1.732050808;
inline constexpr double pi = 3.141592654;
inline constexpr double sin120 = 0.8660254038;
inline constexpr double cos120 = -0.5;
inline constexpr double sin240 = -0.8660254038;
inline constexpr double cos240 = -0.5;
inline constexpr double tan60 = 1.732050808;
inline constexpr double sin30 = 0.5;
inline constexpr double tan30 = 0.5773502692;

inline constexpr double ee = 86.5;  // End-effector side length [mm].
inline constexpr double ff = 346.4; // Base side length [mm].
inline constexpr double re = 465.0; // Forearm length [mm].
inline constexpr double rf = 200.0; // Upper-arm length [mm].
inline constexpr double hf = 299.99119987;
inline constexpr double he = 74.911197427;

inline constexpr double mmtm = 0.001;
inline constexpr double mtmm = 1000.0;
inline constexpr double dtr = pi / 180.0;
inline constexpr double rtd = 180.0 / pi;

class Point {
public:
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Point() = default;
  Point(double x_value, double y_value, double z_value)
      : x(x_value), y(y_value), z(z_value) {}
  Point(const Point &) = default;
  Point &operator=(const Point &) = default;

  bool operator==(const Point &other) const {
    return this->x == other.x && this->y == other.y && this->z == other.z;
  }

  bool operator!=(const Point &other) const { return !(*this == other); }

  void init() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }
};

/// @brief The Theta class represents the angles of the delta robot's arms.
class Theta {
public:
  double angle1{0.0};
  double angle2{0.0};
  double angle3{0.0};

  Theta() = default;
  Theta(const Theta &) = default;
  Theta &operator=(const Theta &) = default;

  void init(void) {
    angle1 = 0.0;
    angle2 = 0.0;
    angle3 = 0.0;
  }
};

/// @brief
typedef struct data_delta {
  double pos{0.0};
  Point position_val;

  double vel{0.0};
  double vel_x{0.0};
  double vel_y{0.0};
  double vel_z{0.0};

  double acel{0.0};
  double acel_x{0.0};
  double acel_y{0.0};
  double acel_z{0.0};

  Theta theta_val;

  double time_point{0.0};
} data_delta_t;
