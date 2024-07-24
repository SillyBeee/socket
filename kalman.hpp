#include <iostream>
#include <Eigen/Dense>


using namespace Eigen;

// 初始化卡尔曼滤波器
void initializeKalmanFilter(VectorXd &x, MatrixXd &P, MatrixXd &F, MatrixXd &H, MatrixXd &R, MatrixXd &Q) {
    // 状态向量 (位置和速度)
    x = VectorXd(6);
    x << 0, 0, 0, 0, 0, 0;

    // 状态协方差矩阵
    P = MatrixXd(6, 6);
    P.setIdentity();

    // 状态转移矩阵
    F = MatrixXd(6, 6);
    F << 1, 0, 0, 1, 0, 0,
         0, 1, 0, 0, 1, 0,
         0, 0, 1, 0, 0, 1,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    // 测量矩阵
    H = MatrixXd(3, 6);
    H << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0;

    // 测量噪声协方差矩阵
    R = MatrixXd(3, 3);
    R.setIdentity();
    R=100.f*R;

    // 过程噪声协方差矩阵
    Q = MatrixXd(6, 6);
    Q.setIdentity();
    Q=100.f*Q;
}

// 卡尔曼滤波器预测步骤
void predict(VectorXd &x, MatrixXd &P, MatrixXd &F, MatrixXd &Q) {
    x = F * x;
    P = F * P * F.transpose() + Q;
}

// 卡尔曼滤波器更新步骤
void update(VectorXd &x, MatrixXd &P, MatrixXd &H, MatrixXd &R, const VectorXd &z) {
    VectorXd y = z - H * x;
    MatrixXd S = H * P * H.transpose() + R;
    MatrixXd K = P * H.transpose() * S.inverse();
    x = x + K * y;
    P = (MatrixXd::Identity(x.size(), x.size()) - K * H) * P;
}

// 打印当前状态和协方差矩阵
void printState(const VectorXd &x, const MatrixXd &P) {
    std::cout << "Estimated state: \n" << x << std::endl;
    // std::cout << "Estimated covariance: \n" << P << std::endl;
}


// int main() {
//     // 初始化卡尔曼滤波器参数
//     VectorXd x;
//     MatrixXd P, F, H, R, Q;
//     initializeKalmanFilter(x, P, F, H, R, Q);

//     // 假设有一系列的测量值 (其中一些是缺失的)
//     std::vector<VectorXd> measurements = {
//         (VectorXd(3) << 1, 1, 1).finished(),
//         VectorXd::Zero(3),  // 缺失值
//         (VectorXd(3) << 2, 2, 2).finished(),
//         VectorXd::Zero(3),  // 缺失值
//         (VectorXd(3) << 3, 3, 3).finished()
//     };

//     // 遍历每个测量值，执行预测和更新步骤
//     for (size_t i = 0; i < measurements.size(); ++i) {
//         // 预测步骤
//         predict(x, P, F, Q);

//         // 打印预测状态
//         std::cout << "After prediction step " << i + 1 << ":" << std::endl;
//         printState(x, P);

//         // 更新步骤 (只有在测量值不为零时才更新)
//         if (measurements[i].norm() != 0) {
//             update(x, P, H, R, measurements[i]);
//         }

//         // 打印更新后的状态
//         std::cout << "After update step " << i + 1 << ":" << std::endl;
//         printState(x, P);
//     }

//     return 0;
// }
