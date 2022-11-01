/**
 * @file utils.cpp
 * @author Michael
 * @brief 旋转矩阵与欧拉角相互转换
 * @version 0.1
 * @date 2022-09-19
 * @ref https://zhuanlan.zhihu.com/p/144032401
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace utils
{
    using namespace std;

    /**
     * @brief 欧拉角 -> 旋转矩阵
     * 
     * @param theta : 欧拉角（弧度单位，外旋 x -> y -> z）
     * @return Eigen::Matrix3d ：旋转矩阵（3X3）
     */
    Eigen::Matrix3d EulerXYZ2Matrix(const Eigen::Vector3d& theta)
    {
        Eigen::Matrix3d R_x;    // 计算旋转矩阵的X分量
        R_x <<
                1,              0,               0,
                0,  cos(theta[0]),  -sin(theta[0]),
                0,  sin(theta[0]),   cos(theta[0]);

        Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
        R_y <<
                cos(theta[1]),   0, sin(theta[1]),
                0,   1,             0,
                -sin(theta[1]),  0, cos(theta[1]);

        Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
        R_z <<
                cos(theta[2]), -sin(theta[2]), 0,
                sin(theta[2]),  cos(theta[2]), 0,
                0,              0,             1;
        Eigen::Matrix3d R = R_z * R_y * R_x;
        return R;
    }

    /**
     * @brief 欧拉角 -> 旋转矩阵
     * 
     * @param theta : 欧拉角（弧度单位，外旋 y -> x -> z）
     * @return Eigen::Matrix3d ：旋转矩阵（3X3）
     */
    Eigen::Matrix3d EulerYXZ2Matrix(const Eigen::Vector3d& theta)
    {
        Eigen::Matrix3d R_x;    // 计算旋转矩阵的X分量
        R_x <<
                1,              0,               0,
                0,  cos(theta[0]),  -sin(theta[0]),
                0,  sin(theta[0]),   cos(theta[0]);

        Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
        R_y <<
                cos(theta[1]),   0, sin(theta[1]),
                0,   1,             0,
                -sin(theta[1]),  0, cos(theta[1]);

        Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
        R_z <<
                cos(theta[2]), -sin(theta[2]), 0,
                sin(theta[2]),  cos(theta[2]), 0,
                0,              0,             1;
        Eigen::Matrix3d R = R_z * R_x * R_y;
        return R;
    }

    /**
     * @brief 判断旋转矩阵是否合理
     * 
     * @param R ：旋转矩阵（3X3）
     * @return true 
     * @return false 
     */
    bool isRotationMatirx(Eigen::Matrix3d R)
    {
        double err=1e-6;
        Eigen::Matrix3d shouldIdenity;
        shouldIdenity=R*R.transpose();
        Eigen::Matrix3d I=Eigen::Matrix3d::Identity();
        return (shouldIdenity - I).norm() < err;
    }

    /**
     * @brief 旋转矩阵 -> 欧拉角
     * 
     * @param R ：旋转矩阵（3X3）
     * @return Eigen::Vector3d ：欧拉角（弧度，外旋 x -> y -> z）
     */
    Eigen::Vector3d Matrix2EulerExternal(const Eigen::Matrix3d& R)
    {
        assert(isRotationMatirx(R));
        double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
        bool singular = sy < 1e-6;
        double x, y, z;

        if (!singular)
        {
            x = atan2( R(2,1), R(2,2));
            y = atan2(-R(2,0), sy);
            z = atan2( R(1,0), R(0,0));
        }
        else
        {
            x = atan2(-R(1,2), R(1,1));
            y = atan2(-R(2,0), sy);
            z = 0;
        }
        return {x, y, z};
    }

    /**
     * @brief 旋转矩阵 -> 欧拉角
     * 
     * @param R ：旋转矩阵（3X3）
     * @return Eigen::Vector3d ：欧拉角（弧度，内旋 x -> y -> z）
     */
    Eigen::Vector3d Matrix2EulerInternal(const Eigen::Matrix3d& mat)
    {
        Eigen::Vector3d vec;

        // The matrixR needs to be orthogonal, in addition the matrixR is pure rotation without reflection component.
        // Check if product of MatrixR and its transpose is identity matrix
        // assert(isRotationMatirx(R));
        double product[3][3] = { 0 };
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                float sum = 0;
                for (int k = 0; k < 3; k++){
                    sum += (mat(i,k) * mat(j,k));
                }
                product[i][j] = sum;
            }
        }

        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                if (i != j && abs(product[i][j] - 0 > 1e-3))
                {
                    printf("Input matrixR is not orthogonal!");
                    return { };
                }
                if (i == j && abs(product[i][j] - 1 > 1e-3))
                {
                    printf("Input matrixR is not orthogonal!");
                    return { };
                }
            }
        }

        //XYZ order < y p r >  ----  < z y x >
        double x, y, z;
        z = -atan2(mat(0, 1), mat(0, 0));
        y = asin(mat(0, 2));
        x = -atan2(mat(1, 2), mat(2, 2));

        return {x, y, z};
    }

    /**
     * @brief 四元数 -> 旋转矩阵
     * 
     * @param R ：四元数（4 * 1）
     * @return Eigen::Matrix3d ：旋转矩阵
     */
    Eigen::Matrix3d Quaternion2Matrix(const Eigen::Quaterniond quad)
    {
        Eigen::Matrix3d mat;
        mat(0, 0) = 2 * quad.w() * quad.w() + 2 * quad.x() * quad.x() - 1;
        mat(0, 1) = 2 * quad.x() * quad.y() - 2 * quad.z() * quad.w();
        mat(0, 2) = 2 * quad.x() * quad.z() + 2 * quad.y() * quad.w();
        mat(1, 0) = 2 * quad.x() * quad.y() + 2 * quad.z() * quad.w();
        mat(1, 1) = 2 * quad.w() * quad.w() + 2 * quad.y() * quad.y() - 1;
        mat(1, 2) = 2 * quad.y() * quad.z() - 2 * quad.x() * quad.w();
        mat(2, 0) = 2 * quad.x() * quad.z() - 2 * quad.y() * quad.w();;
        mat(2, 1) = 2 * quad.y() * quad.z() + 2 * quad.x() * quad.w();
        mat(2, 2) = 2 * quad.w() * quad.w() + 2 * quad.z() * quad.z() - 1;

        return mat;
    }
}




#endif /* UTILS_HPP_ */