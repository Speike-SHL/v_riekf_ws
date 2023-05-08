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
    BigX.setIdentity();
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
    cout << BigX.nonZeros() << endl;
    cout << setprecision(2) << "BigX = \n"
         << BigX << endl;
    //将BigX转化为稠密矩阵
    Eigen::MatrixXd BigX_dense = Eigen::MatrixXd(BigX);
    cout << setprecision(2) << "BigX_dense = \n" << BigX_dense << endl;

    return 0;
}
