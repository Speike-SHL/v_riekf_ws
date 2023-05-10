/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   RobotState.h
 *  @author Ross Hartley
 *  @brief  Source file for RobotState (thread-safe)
 *  @date   September 25, 2018
 **/

#include "InEKF/RobotState.h"
#include "InEKF/LieGroup.h"

namespace inekf {

using namespace std;

/// 默认构造, X = [R v p] = 5x5, Theta = [bg; ba] = 6x1, P = 15x15
RobotState::RobotState() : 
    X_(Eigen::MatrixXd::Identity(5,5)), Theta_(Eigen::MatrixXd::Zero(6,1)), P_(Eigen::MatrixXd::Identity(15,15)) {}
/// Initialize with X
RobotState::RobotState(const Eigen::MatrixXd& X) : 
    X_(X), Theta_(Eigen::MatrixXd::Zero(6,1)) {
    P_ = Eigen::MatrixXd::Identity(3*this->dimX()+this->dimTheta()-6, 3*this->dimX()+this->dimTheta()-6);
}
/// Initialize with X and Theta
RobotState::RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta) : 
    X_(X), Theta_(Theta) {
    P_ = Eigen::MatrixXd::Identity(3*this->dimX()+this->dimTheta()-6, 3*this->dimX()+this->dimTheta()-6);
}
/// Initialize with X, Theta and P
RobotState::RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta, const Eigen::MatrixXd& P) : 
    X_(X), Theta_(Theta), P_(P) {}
// TODO: error checking to make sure dimensions are correct and supported


// // Move initialization
// RobotState::RobotState(RobotState&& other) {
//     lock_guard<mutex> lock(other.mutex_);
//     X_ = std::move(other.X_);
//     other.X_ = Eigen::MatrixXd;
// }

#if INEKF_USE_MUTEX
// Copy initialization
RobotState::RobotState(const RobotState& other) {
    lock_guard<mutex> other_lock(other.mutex_);
    X_ = other.X_;
    Theta_ = other.Theta_;
    P_ = other.P_;
}

// // Move assignment
// RobotState::RobotState& operator = (RobotState&& other) {
//     std::lock(mtx, other.mtx);
//     std::lock_guard<std::mutex> self_lock(mtx, std::adopt_lock);
//     std::lock_guard<std::mutex> other_lock(other.mtx, std::adopt_lock);
//     value = std::move(other.value);
//     other.value = 0;
//     return *this;
// }

// Copy assignment
RobotState& RobotState::operator = (const RobotState& other) {
    lock(mutex_, other.mutex_);
    lock_guard<mutex> self_lock(mutex_, adopt_lock);
    lock_guard<mutex> other_lock(other.mutex_, adopt_lock);
    X_ = other.X_;
    Theta_ = other.Theta_;
    P_ = other.P_;
    return *this;
}
#endif


const Eigen::MatrixXd RobotState::getX() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return X_; 
}
const Eigen::VectorXd RobotState::getTheta() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return Theta_; 
}
const Eigen::MatrixXd RobotState::getP() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return P_; 
}
const Eigen::Matrix3d RobotState::getRotation() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return X_.block<3,3>(0,0); 
}
const Eigen::Vector3d RobotState::getVelocity() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return X_.block<3,1>(0,3); 
}
const Eigen::Vector3d RobotState::getPosition() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return X_.block<3,1>(0,4); 
}
const Eigen::Vector3d RobotState::getGyroscopeBias() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return Theta_.head(3); 
}
const Eigen::Vector3d RobotState::getAccelerometerBias() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return Theta_.tail(3); 
}

/// 返回X的维度
const int RobotState::dimX() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return X_.cols(); 
}
/// 返回theta的行数
const int RobotState::dimTheta() {
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return Theta_.rows();
}
/// 返回P的维度
const int RobotState::dimP() {
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return P_.cols(); 
}

void RobotState::setX(const Eigen::MatrixXd& X) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    X_ = X; 
}
void RobotState::setTheta(const Eigen::VectorXd& Theta) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    Theta_ = Theta; 
}
void RobotState::setP(const Eigen::MatrixXd& P) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    P_ = P; 
}
void RobotState::setRotation(const Eigen::Matrix3d& R) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    X_.block<3,3>(0,0) = R; 
}
void RobotState::setVelocity(const Eigen::Vector3d& v) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    X_.block<3,1>(0,3) = v; 
}
void RobotState::setPosition(const Eigen::Vector3d& p) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    X_.block<3,1>(0,4) = p; 
}
void RobotState::setGyroscopeBias(const Eigen::Vector3d& bg) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    Theta_.head(3) = bg; 
}
void RobotState::setAccelerometerBias(const Eigen::Vector3d& ba) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    Theta_.tail(3) = ba; 
}

/// 在原有BigX基础上将状态X对角扩充n次, 稠密形式
void RobotState::copyDiagX(int n, Eigen::MatrixXd& BigX)
{
    int dimX = this->dimX();
    int totalSize = dimX * n;
    BigX.resize(totalSize, totalSize);
    BigX.setZero();
    for (int i = 0; i < n;++i)
    {
        int startIndex = i * dimX;
#if INEKF_USE_MUTEX
        unique_lock<mutex> mlock(mutex_);
#endif
        BigX.block(startIndex, startIndex, dimX, dimX) = X_;
    }
    // // 将X_以覆盖模式打印输出到文件/home/speike/X_.txt中,输出保留2位小数
    // ofstream fout("/home/speike/X_.txt");
    // fout << fixed << setprecision(1) << X_ << endl;
    // fout.close();
    return;
}


/// 在原有BigX基础上将状态X对角扩充n次, 稀疏形式
void RobotState::copyDiagX(int n, Eigen::SparseMatrix<double> &BigX)
{
    int dimX = this->dimX();
    BigX.resize(dimX * n, dimX * n);
    BigX.setZero();
    vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve((4 * dimX - 3) * n);
    for (int k = 0; k < n; k++)
        for (int i = 0; i < dimX; i++)
            for (int j = 0; j < dimX;j++)
            {
                if(i < 3)
                    tripletList.push_back(Eigen::Triplet<double>(k * dimX + i, k * dimX + j, X_(i, j)));
                else
                    if(i==j)
                        tripletList.push_back(Eigen::Triplet<double>(k * dimX + i, k * dimX + j, X_(i, j)));
            }
    BigX.setFromTriplets(tripletList.begin(), tripletList.end());
    return;
}



/// RobotState类重载<<输出当前状态, 打印X、theta、P的当前值
ostream& operator<<(ostream& os, const RobotState& s) {  
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(s.mutex_);
#endif
    //定义矩阵储存要输出的数据
    Eigen::Matrix<double, 3, 7> X_Theta;
    X_Theta.block<3, 5>(0, 0) = s.X_.block<3, 5>(0, 0);
    X_Theta.block<3, 1>(0, 5) = s.Theta_.head(3);
    X_Theta.block<3, 1>(0, 6) = s.Theta_.tail(3);

    //打印精度
    int precision = 2;

    os << "--------------------------- Robot State --------------------------" << endl;
    os << left << setprecision(precision) << fixed;
    os << setw(3*(precision+4)) << "R:" << setw(precision+4) << "v:" << setw(precision+4) << "p:" << setw(precision+4) << "bg:" << setw(precision+4) << "ba:" << endl;
    os << X_Theta << endl << endl;

    // os << "X:\n" << s.X_ << endl << endl;
    // os << "Theta:\n" << s.Theta_ << endl << endl;
    // os << "P:\n" << s.P_ << endl;
    os << "------------------------------------------------------------------";
    return os;  
} 

} // end inekf namespace
