#include <iostream>
#include <Eigen/Dense>
// #include "kalman.hpp"

Eigen::VectorXd f(const Eigen::VectorXd &x) {
    Eigen::VectorXd x_new = x;
    x_new.head<3>() += x.tail<3>();
    return x_new;
}

Eigen::MatrixXd F_jacobian(const Eigen::VectorXd &x) {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 3) = 1;
    F(1, 4) = 1;
    F(2, 5) = 1;
    return F;
}

Eigen::VectorXd h(const Eigen::VectorXd &x) {
    return x.head<3>();
}

Eigen::MatrixXd H_jacobian(const Eigen::VectorXd &x) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    H(0, 0) = 1;
    H(1, 1) = 1;
    H(2, 2) = 1;
    return H;
}

void initializeKalmanFilter(Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::MatrixXd &R, Eigen::MatrixXd &Q) {
    // 状态向量 (位置和速度)
    x = Eigen::VectorXd(6);
    x << 0, 0, 0, 0, 0, 0;

    // 状态协方差矩阵
    P = Eigen::MatrixXd(6, 6);
    P.setIdentity();
    P=100*P;

    // 测量噪声协方差矩阵
    R = Eigen::MatrixXd(3, 3);
    R.setIdentity();
    R=100*R;


    // 过程噪声协方差矩阵
    Q = Eigen::MatrixXd(6, 6);
    Q.setIdentity();
    Q=100*Q;
}

void predict(Eigen::VectorXd &x, Eigen::MatrixXd &P, const Eigen::MatrixXd &Q) {
    Eigen::MatrixXd F = F_jacobian(x);
    x = f(x);
    P = F * P * F.transpose() + Q;
}

void update(Eigen::VectorXd &x, Eigen::MatrixXd &P, const Eigen::MatrixXd &R, const Eigen::VectorXd &z) {
    Eigen::MatrixXd H = H_jacobian(x);
    Eigen::VectorXd y = z - h(x);
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    x = x + K * y;
    P = (Eigen::MatrixXd::Identity(x.size(), x.size()) - K * H) * P;
}

void printState(const Eigen::VectorXd &x, const Eigen::MatrixXd &P) {
    std::cout << "Estimated state: \n" << x << std::endl;
    std::cout << "Estimated covariance: \n" << P << std::endl;
}

// #include "ekf.hpp"
// #include <iostream>
// #include <vector>

// int main() {
//     // 初始化卡尔曼滤波器参数
//     Eigen::VectorXd x;
//     Eigen::MatrixXd P, R, Q;
//     initializeKalmanFilter(x, P, R, Q);

//     // 假设有一系列的测量值 (其中一些是缺失的)
//     std::vector<Eigen::VectorXd> measurements = {
//         (Eigen::VectorXd(3) << 1, 1, 1).finished(),
//         Eigen::VectorXd::Zero(3),  // 缺失值
//         (Eigen::VectorXd(3) << 2, 2, 2).finished(),
//         Eigen::VectorXd::Zero(3),  // 缺失值
//         (Eigen::VectorXd(3) << 3, 3, 3).finished()
//     };

//     // 遍历每个测量值，执行预测和更新步骤
//     for (size_t i = 0; i < measurements.size(); ++i) {
//         // 预测步骤
//         predict(x, P, Q);

//         // 打印预测状态
//         std::cout << "After prediction step " << i + 1 << ":" << std::endl;
//         printState(x, P);

//         // 更新步骤 (只有在测量值不为零时才更新)
//         if (measurements[i].norm() != 0) {
//             update(x, P, R, measurements[i]);
//         }

//         // 打印更新后的状态
//         std::cout << "After update step " << i + 1 << ":" << std::endl;
//         printState(x, P);
//     }

//     return 0;
// }

