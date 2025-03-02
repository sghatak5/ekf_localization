#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
using namespace std;

class ExtendedKalmanFilter{
    public:
    ExtendedKalmanFilter(const Eigen::VectorXd &initState, //initState
                            const Eigen::MatrixXd &initCov, //initCovariance
                            const Eigen::MatrixXd &measurementNoise, //mesasurement noise
                            const Eigen::MatrixXd &processNoise); //process noise

    pair<Eigen::VectorXd, Eigen::MatrixXd> predict(double dt, //Time Step
                                                const Eigen::VectorXd &imuLinearAcceleration, //Linear Acceleration input from IMU
                                                const Eigen::VectorXd &imuAngularVelocity, //Angular Velocity input from IMU
                                                double g); //Gravity

    pair<Eigen::VectorXd, Eigen::MatrixXd> update(const Eigen::VectorXd &measurement); //Measurement from GPS

    private:
    Eigen::VectorXd state; //State
    Eigen::MatrixXd P; //Covariance
    Eigen::MatrixXd R; //Measurement Noise
    Eigen::MatrixXd Q; //Process Noise
    Eigen::MatrixXd H; //Jacobian of H
    Eigen::MatrixXd F; //State Transition Matrix
    Eigen::MatrixXd I; //Identity Matrix
    Eigen::VectorXd dState; //State Derivative
    
    Eigen::Matrix3d RMatrix; //Rotation Matrix
    Eigen::MatrixXd statePrior; //State Prior
    Eigen::MatrixXd PPrior; //Covariance Prior
    Eigen::MatrixXd K; //Kalman Gain
    Eigen::MatrixXd S; //Innovation Covariance
    Eigen::MatrixXd y; //Innovation
    Eigen::MatrixXd statePosterior; //State Posterior
    Eigen::MatrixXd PPosterior; //Covariance Posterior
    Eigen::MatrixXd JacobianF; //Jacobian of F

    Eigen::VectorXd computeStateDerivative(const Eigen::VectorXd &imuLinearAcceleration,
                                        const Eigen::VectorXd &imuAngularVelocity,
                                        double g);

    Eigen::MatrixXd computeJacobianF(const Eigen::VectorXd &imuLinearAcceleration,
                                    const Eigen::VectorXd &imuAngularVelocity);

    Eigen::Matrix3d updateRotationMatrix(const Eigen::Vector4d &quaternion);
};

#endif 