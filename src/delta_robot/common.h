#pragma once

inline constexpr double pi = 3.141592654;
inline constexpr double sin120 = 0.8660254038;
inline constexpr double cos120 = -0.5;
inline constexpr double sin240 = -0.8660254038;
inline constexpr double cos240 = -0.5;
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

struct Point {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  constexpr Point() = default;
  constexpr Point(double x_value, double y_value, double z_value)
      : x(x_value), y(y_value), z(z_value) {}

  constexpr bool operator==(const Point &other) const {
    return x == other.x && y == other.y && z == other.z;
  }

  constexpr bool operator!=(const Point &other) const {
    return !(*this == other);
  }
};

struct Theta {
  double angle1{0.0};
  double angle2{0.0};
  double angle3{0.0};
};
