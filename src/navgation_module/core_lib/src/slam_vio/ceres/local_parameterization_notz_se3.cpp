#include "slam_vio/ceres/local_parameterization_notz_se3.h"
#include <sophus/se3.hpp>

LocalParameterizationNoTzSE3::LocalParameterizationNoTzSE3() {}
LocalParameterizationNoTzSE3::~LocalParameterizationNoTzSE3() {}

// Generalization of the addition operation,
//
//   x_plus_delta = Plus(x, delta)
//
// with the condition that Plus(x, 0) = x.
bool LocalParameterizationNoTzSE3::Plus(const double* x, const double* delta, double* x_plus_delta) const {
    Eigen::Map<const Eigen::Vector3d> p(x + 4), dp(delta), omega(delta + 3);
    Eigen::Map<const Sophus::SO3d> q(x);
    Eigen::Map<Eigen::Vector3d> p_(x_plus_delta + 4);
    Eigen::Map<Sophus::SO3d> q_(x_plus_delta);
    Sophus::SO3d dq = Sophus::SO3d::exp(omega);
    Eigen::Vector3d dp_remove_z(0,0,-dp.z());
    p_ = p + dp + dp_remove_z;
    q_ = q * dq;
    return true;
}

// The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
// jacobian is a row-major GlobalSize() x LocalSize() matrix.
bool LocalParameterizationNoTzSE3::ComputeJacobian(const double* x, double* jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(jacobian);
    J.setIdentity();
    // J.row(2).setZero(); //set Tz is constant
    J.bottomRows<1>().setZero();
    return true;
}

// Size of x.
int LocalParameterizationNoTzSE3::GlobalSize() const {
    return Sophus::SE3d::num_parameters;
}

// Size of delta.
int LocalParameterizationNoTzSE3::LocalSize() const {
    return Sophus::SE3d::DoF;
}

namespace autodiff {

LocalParameterizationNoTzSE3::LocalParameterizationNoTzSE3() {

}

LocalParameterizationNoTzSE3::~LocalParameterizationNoTzSE3() {

}

// Generalization of the addition operation,
//
//   x_plus_delta = Plus(x, delta)
//
// with the condition that Plus(x, 0) = x.
bool LocalParameterizationNoTzSE3::Plus(const double* T_raw, const double* delta_raw, double* T_plus_delta_raw) const {
    Eigen::Map<Sophus::SE3d const> const T(T_raw);
    Eigen::Map<Sophus::Vector6d const> const delta(delta_raw);
    Eigen::Map<Sophus::SE3d> T_plus_delta(T_plus_delta_raw);
    T_plus_delta = T * Sophus::SE3d::exp(delta);
    return true;
}

// The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
//
// jacobian is a row-major GlobalSize() x LocalSize() matrix.
bool LocalParameterizationNoTzSE3::ComputeJacobian(double const* T_raw, double* jacobian_raw) const {
    Eigen::Map<Sophus::SE3d const> T(T_raw);
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian(jacobian_raw);
    jacobian = T.Dx_this_mul_exp_x_at_0();
    return true;
}

// Size of x.
int LocalParameterizationNoTzSE3::GlobalSize() const {
    return Sophus::SE3d::num_parameters;
}

// Size of delta.
int LocalParameterizationNoTzSE3::LocalSize() const {
    return Sophus::SE3d::DoF;
}

};
