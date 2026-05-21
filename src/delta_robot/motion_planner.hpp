#pragma once

#include "common.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace delta_motion {

struct MotionLimits {
    double max_velocity_mps{1.5};
    double max_acceleration_mps2{200.0};
};

struct ProfileState {
    double position_m{0.0};
    double velocity_mps{0.0};
    double acceleration_mps2{0.0};
};

struct TrajectorySample {
    double time_s{0.0};
    Point position_m;
    Point velocity_mps;
    Point acceleration_mps2;
    double path_position_m{0.0};
    double path_velocity_mps{0.0};
    double path_acceleration_mps2{0.0};
};

struct PlanResult {
    bool ok{false};
    std::string error;
    double duration_s{0.0};
    bool triangular{false};
    std::vector<TrajectorySample> samples;
};

class CartesianTrajectoryGenerator {
public:
    PlanResult planLine(const Point& start_m,
                        const Point& target_m,
                        const MotionLimits& limits,
                        double sample_period_s) const {
        PlanResult result;

        if (sample_period_s <= 0.0) {
            result.error = "Sample period must be positive";
            return result;
        }
        if (limits.max_velocity_mps <= 0.0 || limits.max_acceleration_mps2 <= 0.0) {
            result.error = "Velocity and acceleration limits must be positive";
            return result;
        }

        const double dx = target_m.x - start_m.x;
        const double dy = target_m.y - start_m.y;
        const double dz = target_m.z - start_m.z;
        const double distance_m = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (distance_m <= kEpsilon) {
            result.error = "Start and target are identical";
            return result;
        }

        const Point unit{dx / distance_m, dy / distance_m, dz / distance_m};
        const ProfileTiming timing = computeTiming(distance_m, limits);
        result.duration_s = timing.total_time_s;
        result.triangular = timing.triangular;

        const std::size_t step_count =
            static_cast<std::size_t>(std::ceil(timing.total_time_s / sample_period_s));
        result.samples.reserve(step_count + 1U);

        for (std::size_t i = 0; i <= step_count; ++i) {
            const double t = std::min(static_cast<double>(i) * sample_period_s,
                                      timing.total_time_s);
            const ProfileState profile = sampleProfile(t, distance_m, limits, timing);

            TrajectorySample sample;
            sample.time_s = t;
            sample.path_position_m = profile.position_m;
            sample.path_velocity_mps = profile.velocity_mps;
            sample.path_acceleration_mps2 = profile.acceleration_mps2;
            sample.position_m = {
                start_m.x + unit.x * profile.position_m,
                start_m.y + unit.y * profile.position_m,
                start_m.z + unit.z * profile.position_m,
            };
            sample.velocity_mps = {
                unit.x * profile.velocity_mps,
                unit.y * profile.velocity_mps,
                unit.z * profile.velocity_mps,
            };
            sample.acceleration_mps2 = {
                unit.x * profile.acceleration_mps2,
                unit.y * profile.acceleration_mps2,
                unit.z * profile.acceleration_mps2,
            };
            result.samples.push_back(sample);
        }

        result.samples.back().time_s = timing.total_time_s;
        result.samples.back().position_m = target_m;
        result.samples.back().velocity_mps = {0.0, 0.0, 0.0};
        result.samples.back().acceleration_mps2 = {0.0, 0.0, 0.0};
        result.samples.back().path_position_m = distance_m;
        result.samples.back().path_velocity_mps = 0.0;
        result.samples.back().path_acceleration_mps2 = 0.0;
        result.ok = true;
        return result;
    }

private:
    static constexpr double kEpsilon = 1e-9;

    struct ProfileTiming {
        bool triangular{false};
        double accel_time_s{0.0};
        double cruise_time_s{0.0};
        double total_time_s{0.0};
        double peak_velocity_mps{0.0};
        double accel_distance_m{0.0};
    };

    static ProfileTiming computeTiming(double distance_m, const MotionLimits& limits) {
        ProfileTiming timing;
        const double accel_time_to_vmax = limits.max_velocity_mps / limits.max_acceleration_mps2;
        const double accel_distance =
            0.5 * limits.max_acceleration_mps2 * accel_time_to_vmax * accel_time_to_vmax;

        if ((2.0 * accel_distance) >= distance_m) {
            timing.triangular = true;
            timing.accel_time_s = std::sqrt(distance_m / limits.max_acceleration_mps2);
            timing.cruise_time_s = 0.0;
            timing.peak_velocity_mps = limits.max_acceleration_mps2 * timing.accel_time_s;
            timing.accel_distance_m = 0.5 * distance_m;
        } else {
            timing.triangular = false;
            timing.accel_time_s = accel_time_to_vmax;
            timing.cruise_time_s = (distance_m - 2.0 * accel_distance) / limits.max_velocity_mps;
            timing.peak_velocity_mps = limits.max_velocity_mps;
            timing.accel_distance_m = accel_distance;
        }

        timing.total_time_s = (2.0 * timing.accel_time_s) + timing.cruise_time_s;
        return timing;
    }

    static ProfileState sampleProfile(double t,
                                      double distance_m,
                                      const MotionLimits& limits,
                                      const ProfileTiming& timing) {
        ProfileState state;
        const double accel_end = timing.accel_time_s;
        const double cruise_end = timing.accel_time_s + timing.cruise_time_s;

        if (t <= accel_end) {
            state.position_m = 0.5 * limits.max_acceleration_mps2 * t * t;
            state.velocity_mps = limits.max_acceleration_mps2 * t;
            state.acceleration_mps2 = limits.max_acceleration_mps2;
            return state;
        }

        if (t <= cruise_end) {
            const double cruise_t = t - timing.accel_time_s;
            state.position_m = timing.accel_distance_m + timing.peak_velocity_mps * cruise_t;
            state.velocity_mps = timing.peak_velocity_mps;
            state.acceleration_mps2 = 0.0;
            return state;
        }

        const double decel_t = t - cruise_end;
        state.position_m = timing.accel_distance_m
            + timing.peak_velocity_mps * timing.cruise_time_s
            + timing.peak_velocity_mps * decel_t
            - 0.5 * limits.max_acceleration_mps2 * decel_t * decel_t;
        state.velocity_mps = timing.peak_velocity_mps
            - limits.max_acceleration_mps2 * decel_t;
        state.acceleration_mps2 = -limits.max_acceleration_mps2;

        if (t >= timing.total_time_s) {
            state.position_m = distance_m;
            state.velocity_mps = 0.0;
            state.acceleration_mps2 = 0.0;
        }
        return state;
    }
};

}  // namespace delta_motion
