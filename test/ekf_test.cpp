#include <iostream>
#include <random>
#include "ekf_localization/ekf.h"

Eigen::VectorXd generateImuLinearAcceleration(std::normal_distribution<double>& noiseDist, std::mt19937& gen) {
    Eigen::Vector3d trueAcc(0.1, 0.2, -9.81);  // Include gravity
    Eigen::Vector3d noise(noiseDist(gen), noiseDist(gen), noiseDist(gen));
    return trueAcc + noise;
}

Eigen::VectorXd generateImuAngularVelocity(std::normal_distribution<double>& noiseDist, std::mt19937& gen) {
    Eigen::Vector3d trueOmega(0.01, 0.02, 0.03);
    Eigen::Vector3d noise(noiseDist(gen), noiseDist(gen), noiseDist(gen));
    return trueOmega + noise;
}

Eigen::VectorXd generateMeasurement(const Eigen::VectorXd& trueState, std::normal_distribution<double>& noiseDist, std::mt19937& gen) {
    Eigen::VectorXd measurement(10);
    measurement.setZero();
    measurement.segment<3>(0) = trueState.segment<3>(0) + Eigen::Vector3d(noiseDist(gen), noiseDist(gen), noiseDist(gen)) * 0.01;
    measurement.segment<3>(3) = trueState.segment<3>(3) + Eigen::Vector3d(noiseDist(gen), noiseDist(gen), noiseDist(gen)) * 0.01;
    measurement.segment<4>(6) = trueState.segment<4>(6) + Eigen::Vector4d(noiseDist(gen), noiseDist(gen), noiseDist(gen), noiseDist(gen)) * 0.01;
    measurement.segment<4>(6).normalize();
    return measurement;
}

int main() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> noiseDist(0.0, 0.01);

    Eigen::VectorXd initState = Eigen::VectorXd::Zero(10);
    initState[6] = 1.0;
    Eigen::MatrixXd initCov = Eigen::MatrixXd::Identity(10, 10) * 0.1;
    Eigen::MatrixXd processNoise = Eigen::MatrixXd::Identity(10, 10) * 0.5;  // Increased Q
    processNoise(2, 2) = 3.5;
    Eigen::MatrixXd measurementNoise = Eigen::MatrixXd::Identity(10, 10) * 0.02;  // Adjusted R

    ExtendedKalmanFilter ekf(initState, initCov, measurementNoise, processNoise);

    double dt = 0.1;
    double g = 9.81;
    int numSteps = 100;
    Eigen::VectorXd trueState = initState;

    std::cout << "Starting EKF simulation...\n";

    for (int i = 0; i < numSteps; ++i) {
        Eigen::VectorXd imuLinearAcc = generateImuLinearAcceleration(noiseDist, gen);
        Eigen::VectorXd imuAngularVel = generateImuAngularVelocity(noiseDist, gen);

        auto [predictedState, predictedCov] = ekf.predict(dt, imuLinearAcc, imuAngularVel, g);
        Eigen::VectorXd measurement = generateMeasurement(trueState, noiseDist, gen);
        auto [updatedState, updatedCov] = ekf.update(measurement);

        Eigen::Vector3d gravityWorld(0, 0, g);
        trueState.segment<3>(0) += trueState.segment<3>(3) * dt;
        trueState.segment<3>(3) += (ekf.updateRotationMatrix(trueState.segment<4>(6)) * imuLinearAcc + gravityWorld) * dt;  // No extra -g
        Eigen::Vector4d quat = trueState.segment<4>(6);
        Eigen::Vector4d qDot(
            -0.5 * (quat[1] * imuAngularVel[0] + quat[2] * imuAngularVel[1] + quat[3] * imuAngularVel[2]),
             0.5 * (quat[0] * imuAngularVel[0] + quat[2] * imuAngularVel[2] - quat[3] * imuAngularVel[1]),
             0.5 * (quat[0] * imuAngularVel[1] + quat[3] * imuAngularVel[0] - quat[1] * imuAngularVel[2]),
             0.5 * (quat[0] * imuAngularVel[2] + quat[1] * imuAngularVel[1] - quat[2] * imuAngularVel[0])
        );
        trueState.segment<4>(6) += qDot * dt;
        trueState.segment<4>(6).normalize();

        if (i % 10 == 0) {
            std::cout << "\nStep " << i << ":\n";
            std::cout << "Predicted Position: " << predictedState.segment<3>(0).transpose() << "\n";
            std::cout << "Updated Position: " << updatedState.segment<3>(0).transpose() << "\n";
            std::cout << "True Position: " << trueState.segment<3>(0).transpose() << "\n";
            std::cout << "Position Error Norm: " << (updatedState.segment<3>(0) - trueState.segment<3>(0)).norm() << "\n";
        }
    }

    std::cout << "\nSimulation complete!\n";
    std::cout << "Final state: " << ekf.getState().transpose() << "\n";
    return 0;
}
