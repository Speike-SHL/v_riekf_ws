cmake_minimum_required(VERSION 3.0.2)
project(eigen_sparse)

# 添加C++11支持
add_compile_options(-std=c++17)

# 寻找Eigen库并添加到包含路径
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 添加可执行文件
add_executable(eigen_sparse eigen_sparse.cpp)
target_link_libraries(eigen_sparse ${EIGEN3_LIBS})
