#include "ekf_localization/ekf.h"

ExtendedKalmanFilter::ExtendedKalmanFilter(const Eigen::VectorXd &initState, //initState
                                            const Eigen::MatrixXd &initCov, //initCovariance
                                            const Eigen::MatrixXd &measurementNoise, //mesasurement noise
                                            const Eigen::MatrixXd &processNoise) //process noise
:state(initState), 
P(initCov), 
R(measurementNoise), 
Q(processNoise),     
H(Eigen::MatrixXd::Zero(state.size(), state.size())),
I(Eigen::MatrixXd::Identity(state.size(), state.size())),
dState(Eigen::VectorXd::Zero(state.size())),
RMatrix(Eigen::Matrix3d::Identity()),
JacobianF(Eigen::MatrixXd::Zero(state.size(), state.size())){}


Eigen::Matrix3d ExtendedKalmanFilter::updateRotationMatrix(const Eigen::Vector4d &quaternion) {

    double qw = quaternion(0);
    double qx = quaternion(1);
    double qy = quaternion(2);
    double qz = quaternion(3);

    this->RMatrix << 
        1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw),
        2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw),
        2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy);

    return this->RMatrix;
}

Eigen::VectorXd ExtendedKalmanFilter::computeStateDerivative(const Eigen::VectorXd &imuLinearAcceleration,
                                        const Eigen::VectorXd &imuAngularVelocity,
                                        double g) {

    Eigen::Vector3d linearAcceleration = this->RMatrix * imuLinearAcceleration;
    
    const double qw = state(6), qx = state(7), qy = state(8), qz = state(9);
    const double wx = imuAngularVelocity[0], wy = imuAngularVelocity[1], wz = imuAngularVelocity[2];

    this->dState.head<3>() = this->state.segment<3>(3); // dx/dt = vx, dy/dt = vy, dz/dt = vz
    this->dState.segment<3>(3) = linearAcceleration + Eigen::Vector3d(0, 0, g); // dvx/dt = ax, dvy/dt = ay, dvz/dt = az

    this->dState.segment<4>(6) <<
        -0.5 * (qx * wx + qy * wy + qz * wz), // dq_w/dt
        0.5 * (qw * wx - qz * wy + qy * wz), // dq_x/dt
        0.5 * (qz * wx + qw * wy - qx * wz), // dq_y/dt
        0.5 * (-qy * wx + qx * wy + qw * wz); // dq_z/dt
    
    return this->dState;
}

Eigen::MatrixXd ExtendedKalmanFilter::computeJacobianF(const Eigen::VectorXd &imuLinearAcceleration,
                                                        const Eigen::VectorXd &imuAngularVelocity) {

    this->JacobianF.setZero();

    this->JacobianF.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(); // Position derivatives w.r.t velocity
    
    const double qx = this->state[6], qy = this->state[7], qz = this->state[8], qw = this->state[9];
    double ax = imuLinearAcceleration[0], ay = imuLinearAcceleration[1], az = imuLinearAcceleration[2];
    
    this->JacobianF.block<3, 4>(3, 6) <<
        -2.0 * qz * ay + 2.0 * qy * az, 2.0 * qy * ay + 2.0 * qz * az, -4.0 * qy * ax + 2.0 * qx * ay + 2.0 * qw * az, -4.0 * qz * ax - 2.0 * qw * ay + 2.0 * qx * az,
        2.0 * qz * ax - 2.0 * qx * az, 2.0 * qy * ax - 4.0 * qx * ay - 2.0 * qw * az, 2.0 * qx * ax + 2.0 * qz * az, 2.0 * qw * ax - 2.0 * qx * ay + 2.0 * qy * az,
        -2.0 * qy * ax + 2.0 * qx * ay, 2.0 * qz * ax - 2.0 * qw * ay - 4.0 * qx * az, -2.0 * qw * ax + 2.0 * qz * ay - 4.0 * qy * az, 2.0 * qx * ax + 2.0 * qy * ay;
    
    const double wx = imuAngularVelocity[0], wy = imuAngularVelocity[1], wz = imuAngularVelocity[2];

    this->JacobianF.block<4, 3>(6, 6) <<
        -0.5 * wx, -0.5 * wy, -0.5 * wz,
        0.5 * wx,  0.5 * wz, -0.5 * wy,
        0.5 * wy, -0.5 * wz,  0.5 * wx,
        0.5 * wz,  0.5 * wy, -0.5 * wx;

    return this->JacobianF;
}

pair<Eigen::VectorXd, Eigen::MatrixXd> ExtendedKalmanFilter::predict(double dt,
                                                                    const Eigen::VectorXd &imuLinearAcceleration, //Linear Acceleration input from IMU
                                                                    const Eigen::VectorXd &imuAngularVelocity,  //Angular Velocity input from IMU
                                                                    double g) //Gravity 
{   
    this->RMatrix = this->updateRotationMatrix(this->state.segment<4>(6)); //Update Rotation Matrix
    this->dState = this->computeStateDerivative(imuLinearAcceleration, imuAngularVelocity, g); //Compute State Derivative


    this->JacobianF = this->computeJacobianF(imuLinearAcceleration, imuAngularVelocity); //Compute Jacobian of F
    this->F = this->I + this->JacobianF * dt;

    this->statePrior = this->state + this->dState * dt; // x' = x + dx * dt
    
    double norm_q = this->statePrior.segment<4>(6).norm();
    if (norm_q > 1e-6) { this->statePrior.segment<4>(6) /= norm_q; } 
    else { this->statePrior.segment<4>(6) = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0); } // Normalize Quaternion
    
    this->PPrior = this->F * this->P * this->F.transpose() + this->Q; // P' = FPF^T + Q

    this->state = this->statePrior;
    this->P = this->PPrior;

    return {this->state, this->P};
}

pair<Eigen::VectorXd, Eigen::MatrixXd> ExtendedKalmanFilter::update(const Eigen::VectorXd &measurement){
    
    this->H.setZero();
    this->H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // Position 
    //this->H.block<3, 3>(7, 7) = Eigen::Matrix3d::Identity(); // Quaternion

    this->S = this->R + this->H * this->P * this->H.transpose(); // S = HPH^T + R Measurement Covariance
    this->K = this->P * this->H.transpose() * this->S.inverse(); // K = PH^TS^-1 Kalman Gain

    this->statePosterior = this->state + this->K * (measurement - this->H * this->state); // x = x + K(z - Hx) Posterior State Estimate
    this->PPosterior = (this->I - this->K * this->H) * this->P; // P = (I - KH)P Posterior Covariance

    this->state = this->statePosterior;
    this->P = this->PPosterior;

    return {this->state, this->P};
}

const Eigen::VectorXd& ExtendedKalmanFilter::getState() const {
    return this->state;
}