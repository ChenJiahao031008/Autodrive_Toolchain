#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "../sensor_data/Infrastructure.hpp"
#include "../common/logger.hpp"


namespace interface
{

class DataConverter
{
public:
    // cv::Mat <- Eigen::Matrix
    static cv::Mat toCvMat(const Eigen::Matrix4d &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix4f &m);
    static cv::Mat toCvMat(const Eigen::Matrix3f &m);
    static cv::Mat toCvMat(const Eigen::Vector3d &m);
    static cv::Mat toCvMat(const Eigen::Vector3f &m);
    static cv::Mat toCvMat(const Eigen::Vector4d &m);
    static cv::Mat toCvMat(const Eigen::Vector4f &m);
    static cv::Mat toCvMat(const Eigen::MatrixXd &m);
    static cv::Mat toCvMat(const Eigen::MatrixXf &m);

    // cv::Mat -> Eigen::Matrix
    static Eigen::Matrix4d toEigenMatrix4d(const cv::Mat &m);
    static Eigen::Matrix4f toEigenMatrix4f(const cv::Mat &m);
    static Eigen::Matrix3d toEigenMatrix3d(const cv::Mat &m);
    static Eigen::Matrix3f toEigenMatrix3f(const cv::Mat &m);
    static Eigen::Vector4d toEigenVector4d(const cv::Mat &m);
    static Eigen::Vector4f toEigenVector4f(const cv::Mat &m);
    static Eigen::Vector3d toEigenVector3d(const cv::Mat &m);
    static Eigen::Vector3f toEigenVector3f(const cv::Mat &m);
    static Eigen::Vector3d toEigenVector3d(const cv::Point3d &cvPoint);
    static Eigen::Vector3f toEigenVector3f(const cv::Point3f &cvPoint);
    static Eigen::Vector2d toEigenVector2d(const cv::Point2d &cvPoint);
    static Eigen::Vector2f toEigenVector2f(const cv::Point2f &cvPoint);

    // 旋转矩阵转换为旋转向量
    static Eigen::Vector3f RotationtoVector(const Eigen::Matrix3f &R);
    // 旋转矩阵转换为欧拉角
    static Eigen::Vector3f RotationtoEuler(const Eigen::Matrix3f &R);
    // 旋转矩阵转换为四元数
    static Eigen::Quaternionf RotationtoQuaternion(const Eigen::Matrix3f &R);

    // 欧拉角转换为旋转矩阵
    static Eigen::Matrix3f EulertoRotation(const Eigen::Vector3f &euler);
    // 欧拉角转换为四元数
    static Eigen::Quaternionf EulertoQuaternion(const Eigen::Vector3f &euler);
    // 欧拉角转换为旋转向量
    static Eigen::Vector3f EulertoVector(const Eigen::Vector3f &euler);

    // 四元数转换为旋转矩阵
    static Eigen::Matrix3f QuaterniontoRotation(const Eigen::Quaternionf &q);
    // 四元数转换为欧拉角
    static Eigen::Vector3f QuaterniontoEuler(const Eigen::Quaternionf &q);
    // 四元数转换为旋转向量
    static Eigen::Vector3f QuaterniontoVector(const Eigen::Quaternionf &q);

    // 旋转向量转换为旋转矩阵（罗德里格斯公式）
    static Eigen::Matrix3f VectortoMatrix(const Eigen::Vector3f &theta);
    // 旋转向量转换为四元数
    static Eigen::Quaternionf VectortoQuaternion(const Eigen::Vector3f &theta);
    // 旋转向量转换为欧拉角
    static Eigen::Vector3f VectortoEuler(const Eigen::Vector3f &theta);
};

}
