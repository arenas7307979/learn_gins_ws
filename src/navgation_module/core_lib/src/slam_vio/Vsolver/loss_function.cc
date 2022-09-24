//
// Created by gaoxiang19 on 11/10/18.
//
#include "slam_vio/Vsolver/loss_function.h"

namespace Vsolver {

    void HuberLoss::Compute(double e, Eigen::Vector3d& rho) const {
        double dsqr = delta_ * delta_;
        if (e <= dsqr) { // inlier
            rho[0] = e;
            rho[1] = 1.;
            rho[2] = 0.;
        } else { // outlier
            double sqrte = sqrt(e); // absolut value of the error
            rho[0] = 2*sqrte*delta_ - dsqr; // rho(e)   = 2 * delta * e^(1/2) - delta^2
            rho[1] = std::max(std::numeric_limits<double>::min(), delta_ / sqrte);// rho'(e)  = delta / sqrt(e)  
            rho[2] = - 0.5 * rho[1] / e;    // rho''(e) = -1 / (2*e^(3/2)) = -1/2 * (delta/e) / e
        }
    }

    void CauchyLoss::Compute(double err2, Eigen::Vector3d& rho) const {
        //delta_ = c; // lossfunction of ceres:a
        //dsqr = c^2 // lossfunction of ceres:b 
        //dsqrReci =1/c^2 // lossfunction of ceres:c
        double aux = dsqrReci * err2 + 1.0;  // 1 + e^2/c^
        double inv = 1.0 / aux;
        rho[0] = dsqr * log(aux);            // c^2 * log( 1 + e^2/c^2 )
        //rho[1] = 1. / aux;                   // rho'
        rho[1] = std::max( std::numeric_limits<double>::min(), inv);
        rho[2] = -dsqrReci * (inv * inv); // rho''
    }

    void TukeyLoss::Compute(double e2, Eigen::Vector3d& rho) const
    {
        const double e = sqrt(e2);
        const double delta2 = delta_ * delta_;
        if (e <= delta_) {
            const double aux = e2 / delta2;
            rho[0] = delta2 * (1. - std::pow((1. - aux), 3)) / 3.;
            rho[1] = std::pow((1. - aux), 2);
            rho[2] = -2. * (1. - aux) / delta2;
        } else {
            rho[0] = delta2 / 3.;
            rho[1] = 0;
            rho[2] = 0;
        }
    }
}
