#include <iostream>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <iomanip>
using namespace std;

int main()
{
    Eigen::Matrix<double, 6, 6> X = Eigen::Matrix<double, 6, 6>::Identity();
    X.topRows(3) = Eigen::Matrix<double, 3, 6>::Random();
    cout << "X = \n" << X << endl;

    Eigen::SparseMatrix<double> BigX;
    int n = 3;
    int dimX = X.rows();
    BigX.resize(n*dimX, n*dimX);
    vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve((4 * dimX - 3) * n);
    for (int k = 0; k < n; k++)
        for (int i = 0; i < dimX; i++)
            for (int j = 0; j < dimX;j++)
            {
                if(i < 3)
                    tripletList.push_back(Eigen::Triplet<double>(k * dimX + i, k * dimX + j, X(i, j)));
                else
                    if(i==j)
                        tripletList.push_back(Eigen::Triplet<double>(k * dimX + i, k * dimX + j, X(i, j)));
            }
    BigX.setFromTriplets(tripletList.begin(), tripletList.end());
    cout << BigX.rows() << endl;
    //将BigX转化为稠密矩阵
    Eigen::MatrixXd BigX_dense = Eigen::MatrixXd(BigX);
    cout << setprecision(2) << "BigX_dense = \n" << BigX_dense << endl;

    Eigen::MatrixXd H;
    int startIndex = 0;
    for (int k = 0; k < 3; k++)
    {
    startIndex = H.rows();
    H.conservativeResize(startIndex + 3, 15);
    H.block(startIndex, 0, 3, 10) = Eigen::MatrixXd::Zero(3, 10);
    H.block(startIndex, 6, 3, 3) = -Eigen::Matrix3d::Identity();   // -I
    H.block(startIndex, 3 * 6 - 6, 3, 3) = Eigen::Matrix3d::Identity(); // I
    }
    cout << H << endl;
    cout << "----------------------" << endl;

    Eigen::SparseMatrix<double> H_sparse;
    startIndex = 0;
    for (int k = 0; k < 3; k++)
    {
    startIndex = H_sparse.rows();
    H_sparse.conservativeResize(startIndex + 3, 15);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 15; ++j)
            H_sparse.insert(startIndex + i, j) = 0;
    H_sparse.coeffRef(startIndex, 6) = -1;
    H_sparse.coeffRef(startIndex + 1, 7) = -1;
    H_sparse.coeffRef(startIndex + 2, 8) = -1;
    H_sparse.coeffRef(startIndex, 3 * 6 - 6) = 1;
    H_sparse.coeffRef(startIndex + 1, 3 * 6 - 5) = 1;
    H_sparse.coeffRef(startIndex + 2, 3 * 6 - 4) = 1;
    }
    cout << Eigen::MatrixXd(H_sparse) << endl;
    return 0;
}
