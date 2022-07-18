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
    // cv::Mat(CV_64F) <- Eigen::Matrix
    template <typename _Scalar, int _Rows, int _Cols>
    static cv::Mat EigentoCvMat64F(const Eigen::Matrix<_Scalar, _Rows, _Cols> &m)
    {
        cv::Mat cv_mat(m.rows(), m.cols(), CV_64F);
        for (int i = 0; i < m.rows(); i++)
        {
            for (int j = 0; j < m.cols(); j++)
            {
                cv_mat.at<double>(i, j) = m(i, j);
            }
        }
        return cv_mat;
    }

    // cv::Mat(CV_32F) <- Eigen::Matrix
    template <typename _Scalar, int _Rows, int _Cols>
    static cv::Mat EigentoCvMat32F(const Eigen::Matrix<_Scalar, _Rows, _Cols> &m)
    {
        cv::Mat cv_mat(m.rows(), m.cols(), CV_32F);
        for (int i = 0; i < m.rows(); i++)
        {
            for (int j = 0; j < m.cols(); j++)
            {
                cv_mat.at<float>(i, j) = m(i, j);
            }
        }
        return cv_mat;
    }

    // cv::Mat(CV_64F) -> Eigen::Matrix
    template <typename _Scalar, int _Rows, int _Cols>
    static Eigen::Matrix4d CvMat64FtoEigen(const cv::Mat &m)
    {
        Eigen::Matrix<_Scalar, _Rows, _Cols> eigen_mat;
        for (int i = 0; i < m.rows(); i++)
        {
            for (int j = 0; j < m.cols(); j++)
            {
                eigen_mat(i, j) = m.at<double>(i, j);
            }
        }
        return eigen_mat;
    }

    // cv::Mat(CV_32F) -> Eigen::Matrix
    template <typename _Scalar, int _Rows, int _Cols>
    static Eigen::Matrix4d CvMat32FtoEigen(const cv::Mat &m)
    {
        Eigen::Matrix<_Scalar, _Rows, _Cols> eigen_mat;
        for (int i = 0; i < m.rows(); i++)
        {
            for (int j = 0; j < m.cols(); j++)
            {
                eigen_mat(i, j) = m.at<float>(i, j);
            }
        }
        return eigen_mat;
    }


    static Eigen::Vector3d toEigenVector3d(const cv::Point3d &cvPoint);
    static Eigen::Vector3f toEigenVector3f(const cv::Point3f &cvPoint);
    static Eigen::Vector2d toEigenVector2d(const cv::Point2d &cvPoint);
    static Eigen::Vector2f toEigenVector2f(const cv::Point2f &cvPoint);

    // 旋转矩阵转换为旋转向量
    static Eigen::Vector3f RotationtoVector(const Eigen::Matrix3f &R);
    static Eigen::Vector3d RotationtoVector(const Eigen::Matrix3d &R);
    // 旋转矩阵转换为欧拉角
    static Eigen::Vector3f RotationtoEuler(const Eigen::Matrix3f &R);
    static Eigen::Vector3d RotationtoEuler(const Eigen::Matrix3d &R);
    // 旋转矩阵转换为四元数
    static Eigen::Quaternionf RotationtoQuaternion(const Eigen::Matrix3f &R);
    static Eigen::Quaterniond RotationtoQuaternion(const Eigen::Matrix3d &R);

    // 欧拉角转换为旋转矩阵
    static Eigen::Matrix3f EulertoRotation(const Eigen::Vector3f &euler);
    static Eigen::Matrix3d EulertoRotation(const Eigen::Vector3d &euler);
    // 欧拉角转换为四元数
    static Eigen::Quaternionf EulertoQuaternion(const Eigen::Vector3f &euler);
    static Eigen::Quaterniond EulertoQuaternion(const Eigen::Vector3d &euler);
    // 欧拉角转换为旋转向量
    static Eigen::Vector3f EulertoVector(const Eigen::Vector3f &euler);
    static Eigen::Vector3d EulertoVector(const Eigen::Vector3d &euler);

    // 四元数转换为旋转矩阵
    static Eigen::Matrix3f QuaterniontoRotation(const Eigen::Quaternionf &q);
    static Eigen::Matrix3d QuaterniontoRotation(const Eigen::Quaterniond &q);
    // 四元数转换为欧拉角
    static Eigen::Vector3f QuaterniontoEuler(const Eigen::Quaternionf &q);
    static Eigen::Vector3d QuaterniontoEuler(const Eigen::Quaterniond &q);
    // 四元数转换为旋转向量
    static Eigen::Vector3f QuaterniontoVector(const Eigen::Quaternionf &q);
    static Eigen::Vector3d QuaterniontoVector(const Eigen::Quaterniond &q);

    // 旋转向量转换为旋转矩阵（罗德里格斯公式）
    static Eigen::Matrix3f VectortoMatrix(const Eigen::Vector3f &theta);
    static Eigen::Matrix3d VectortoMatrix(const Eigen::Vector3d &theta);
    // 旋转向量转换为四元数
    static Eigen::Quaternionf VectortoQuaternion(const Eigen::Vector3f &theta);
    static Eigen::Quaterniond VectortoQuaternion(const Eigen::Vector3d &theta);
    // 旋转向量转换为欧拉角
    static Eigen::Vector3f VectortoEuler(const Eigen::Vector3f &theta);
    static Eigen::Vector3d VectortoEuler(const Eigen::Vector3d &theta);
};

}
