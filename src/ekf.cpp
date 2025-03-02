#include "ekf.h"

ExtendedKalmanFilter::ExtendedKalmanFilter(const Eigen::VectorXd &initState, //initState
                                            const Eigen::MatrixXd &initCov, //initCovariance
                                            const Eigen::MatrixXd &measurementNoise, //mesasurement noise
                                            const Eigen::MatrixXd &processNoise) //process noise
:state(initState), P(initCov), R(measurementNoise), Q(processNoise) {
    H = Eigen::MatrixXd::Zero(state.size(), state.size());
    dState = Eigen::VectorXd::Zero(state.size());
    JacobianF = Eigen::MatrixXd::Zero(state.size(), state.size());
    I = Eigen::MatrixXd::Identity(state.size(), state.size());
    RMatrix = Eigen::Matrix3d::Zero();
}

Eigen::Matrix3d ExtendedKalmanFilter::updateRotationMatrix(const Eigen::Vector4d &quaternion) {

    double qx = quaternion(0);
    double qy = quaternion(1);
    double qz = quaternion(2);
    double qw = quaternion(3);
    
    RMatrix(0, 0) = 1.0 - 2.0 * qy * qy - 2.0 * qz * qz;
    RMatrix(0, 1) = 2.0 * qx * qy - 2.0 * qz * qw;
    RMatrix(0, 2) = 2.0 * qx * qz + 2.0 * qy * qw;
    RMatrix(1, 0) = 2.0 * qx * qy + 2.0 * qz * qw;
    RMatrix(1, 1) = 1.0 - 2.0 * qx * qx - 2.0 * qz * qz;
    RMatrix(1, 2) = 2.0 * qy * qz - 2.0 * qx * qw;
    RMatrix(2, 0) = 2.0 * qx * qz - 2.0 * qy * qw;
    RMatrix(2, 1) = 2.0 * qy * qz + 2.0 * qx * qw;
    RMatrix(2, 2) = 1.0 - 2.0 * qx * qx - 2.0 * qy * qy;

    return RMatrix;
}

Eigen::VectorXd ExtendedKalmanFilter::computeStateDerivative(const Eigen::VectorXd &imuLinearAcceleration,
                                        const Eigen::VectorXd &imuAngularVelocity,
                                        double g) {

    Eigen::Vector3d linearAcceleration = RMatrix * imuLinearAcceleration;
    double px = state(0), py = state(1), pz = state(2);
    double vx = state(3), vy = state(4), vz = state(5);
    double qw = state(6), qx = state(7), qy = state(8), qz = state(9);

    dState(0) = vx; // dpx/dt
    dState(1) = vy; // dpy/dt
    dState(2) = vz; // dpz/dt
    dState(3) = linearAcceleration(0); // dvx/dt
    dState(4) = linearAcceleration(1); // dvy/dt
    dState(5) = linearAcceleration(2) + g; // dvz/dt
    dState[6] = -0.5 * (qx * imuAngularVelocity[0] + qy * imuAngularVelocity[1] + qz * imuAngularVelocity[2]); // dq_w/dt
    dState[7] = 0.5 * (qw * imuAngularVelocity[0] - qz * imuAngularVelocity[1] + qy * imuAngularVelocity[2]); // dq_x/dt
    dState[8] = 0.5 * (qz * imuAngularVelocity[0] + qw * imuAngularVelocity[1] - qx * imuAngularVelocity[2]); // dq_y/dt
    dState[9] = 0.5 * (-qy * imuAngularVelocity[0] + qx * imuAngularVelocity[1] + qw * imuAngularVelocity[2]); // dq_z/dt
    
    return dState;
}

Eigen::MatrixXd ExtendedKalmanFilter::computeJacobianF(const Eigen::VectorXd &imuLinearAcceleration,
                                const Eigen::VectorXd &imuAngularVelocity) {
    
    double qx = state[6], qy = state[7], qz = state[8], qw = state[9];
    double ax = imuLinearAcceleration[0], ay = imuLinearAcceleration[1], az = imuLinearAcceleration[2];
    double wx = imuAngularVelocity[0], wy = imuAngularVelocity[1], wz = imuAngularVelocity[2];

    JacobianF(0, 3) = 1.0; 
    JacobianF(1, 4) = 1.0;
    JacobianF(2, 5) = 1.0;

    JacobianF(3, 6) = -2.0 * qz * ay + 2.0 * qy * az;
    JacobianF(3, 7) = 2.0 * qy * ay + 2.0 * qz * az;
    JacobianF(3, 8) = -4.0 * qy * ax + 2.0 * qx * ay + 2.0 * qw * az;
    JacobianF(3, 9) = -4.0 * qz * ax - 2.0 * qw * ay + 2.0 * qx * az;

    JacobianF(4, 6) = 2.0 * qz * ax - 2.0 * qx * az;
    JacobianF(4, 7) = 2.0 * qy * ax - 4.0 * qx * ay - 2.0 * qw * az;
    JacobianF(4, 8) = 2.0 * qx * ax + 2.0 * qz * az;
    JacobianF(4, 9) = 2.0 * qw * ax - 2.0 * qx * ay + 2.0 * qy * az;

    JacobianF(5, 6) = -2.0 * qy * ax + 2.0 * qx * ay;
    JacobianF(5, 7) = 2.0 * qz * ax - 2.0 * qw * ay - 4.0 * qx * az;
    JacobianF(5, 8) = -2.0 * qw * ax + 2.0 * qz * ay - 4.0 * qy * az;
    JacobianF(5, 9) = 2.0 * qx * ax + 2.0 * qy * ay;

    JacobianF(6, 7) = -0.5 * wx;
    JacobianF(6, 8) = -0.5 * wy;
    JacobianF(6, 9) = -0.5 * wz;

    JacobianF(7, 6) = 0.5 * wx;
    JacobianF(7, 8) = 0.5 * wz;
    JacobianF(7, 9) = -0.5 * wy;

    JacobianF(8, 6) = 0.5 * wy;
    JacobianF(8, 7) = -0.5 * wz;
    JacobianF(8, 9) = 0.5 * wx;

    JacobianF(9, 6) = 0.5 * wz;
    JacobianF(9, 7) = 0.5 * wy;
    JacobianF(9, 8) = -0.5 * wx;

    return JacobianF;
}

pair<Eigen::VectorXd, Eigen::MatrixXd> ExtendedKalmanFilter::predict(double dt,
                                                                    const Eigen::VectorXd &imuLinearAcceleration, //Linear Acceleration input from IMU
                                                                    const Eigen::VectorXd &imuAngularVelocity,  //Angular Velocity input from IMU
                                                                    const Eigen::Vector4d &quaternion, //Quaternion
                                                                    double g) //Gravity 
{   
    RMatrix = updateRotationMatrix(state.segment<4>(6)); //Update Rotation Matrix
    dState = computeStateDerivative(imuLinearAcceleration, imuAngularVelocity, g); //Compute State Derivative


    JacobianF = computeJacobianF(imuLinearAcceleration, imuAngularVelocity); //Compute Jacobian of F
    F = I + JacobianF * dt;

    statePrior = state + dState * dt; // x' = x + dx * dt
    statePrior.segment<4>(6) /= statePrior.segment<4>(6).norm(); //Normalize Quaternion
    PPrior = F * P * F.transpose() + Q; // P' = FPF^T + Q

    state = statePrior;
    P = PPrior;

    return {state, P};
}