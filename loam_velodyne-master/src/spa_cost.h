//
// Created by hl on 17-10-3.
//

#ifndef HELEI_WS_SPA_COST_H
#define HELEI_WS_SPA_COST_H
#include <array>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <cmath>
#include "constraint.h"
#include <vector>
double M_PI=3.1415926;

template <typename T>
T NormalizeAngleDifference(T difference) {
    while (difference > M_PI) {
        difference -= T(2. * M_PI);
    }
    while (difference < -M_PI) {
        difference += T(2. * M_PI);
    }
    return difference;
}
class Spa_cost
{
    explicit Spa_cost(const Constraint::Pose& pose) : pose_(pose) {}

    template <typename T>
    static std::array<T, 3> ComputeUnscaledError(
            const std::vector<double> c_relative_pose, const T* const c_i,
            const T* const c_j) {
        const T cos_theta_i = cos(c_i[2]);
        const T sin_theta_i = sin(c_i[2]);//角度误差
        const T delta_x = c_j[0] - c_i[0];//位移误差x
        const T delta_y = c_j[1] - c_i[1];//位移误差y
        const T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y,
                        -sin_theta_i * delta_x + cos_theta_i * delta_y,
                        c_j[2] - c_i[2]};
        return {{T(c_relative_pose[0]) - h[0],
                        T(c_relative_pose[1]) - h[1],
                        common::NormalizeAngleDifference(T(c_relative_pose[2]) -
                                                         h[2])}};
    }

    // Computes the error scaled by 'translation_weight' and 'rotation_weight',
    // storing it in 'e'.
    template <typename T>
    static void ComputeScaledError(const Constraint::Pose& pose,
                                   const T* const c_i, const T* const c_j,
                                   T* const e) {
        const std::array<T, 3> e_ij =
                ComputeUnscaledError(pose.c_relative_pose, c_i, c_j);
        e[0] = e_ij[0] * T(pose.c_translation_weight);
        e[1] = e_ij[1] * T(pose.c_translation_weight);
        e[2] = e_ij[2] * T(pose.c_rotation_weight);
    }

    template <typename T>
    bool operator()(const T* const c_i, const T* const c_j, T* e) const {
        ComputeScaledError(pose_, c_i, c_j, e);
        return true;
    }

private:
    Constraint::Pose pose_;

};
#endif //HELEI_WS_SPA_COST_H
