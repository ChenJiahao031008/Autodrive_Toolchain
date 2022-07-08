#include "DataConverter.h"

namespace interface
{

cv::Mat DataConverter::toCvMat(const Eigen::Matrix4d &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat DataConverter::toCvMat(const Eigen::Matrix4f &m)
{
    cv::Mat cvMat(4, 4, CV_32F);
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            cvMat.at<float>(i, j) = m(i, j);

    return cvMat.clone();
}

cv::Mat DataConverter::toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat DataConverter::toCvMat(const Eigen::Matrix3f &m)
{
    cv::Mat cvMat(3, 3, CV_32F);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            cvMat.at<float>(i, j) = m(i, j);

    return cvMat.clone();
}

cv::Mat DataConverter::toCvMat(const Eigen::MatrixXd &m)
{
    cv::Mat cvMat(m.rows(),m.cols(),CV_32F);
    for(int i=0;i<m.rows();i++)
        for(int j=0; j<m.cols(); j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat DataConverter::toCvMat(const Eigen::MatrixXf &m)
{
    cv::Mat cvMat(m.rows(), m.cols(), CV_32F);
    for (int i = 0; i < m.rows(); i++)
        for (int j = 0; j < m.cols(); j++)
            cvMat.at<float>(i, j) = m(i, j);

    return cvMat.clone();
}

cv::Mat DataConverter::toCvMat(const Eigen::Vector3d &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

cv::Mat DataConverter::toCvMat(const Eigen::Vector3f &m)
{
    cv::Mat cvMat(3, 1, CV_32F);
    for (int i = 0; i < 3; i++)
        cvMat.at<float>(i) = m(i);

    return cvMat.clone();
}

cv::Mat DataConverter::toCvMat(const Eigen::Vector4d &m)
{
    cv::Mat cvMat(4, 1, CV_32F);
    for (int i = 0; i < 4; i++)
        cvMat.at<float>(i) = m(i);

    return cvMat.clone();
}

cv::Mat DataConverter::toCvMat(const Eigen::Vector4f &m)
{
    cv::Mat cvMat(4, 1, CV_32F);
    for (int i = 0; i < 4; i++)
        cvMat.at<float>(i) = m(i);

    return cvMat.clone();
}

Eigen::Matrix4d toEigenMatrix4d(const cv::Mat &m){
    Eigen::Matrix4d eigenMat;
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            eigenMat(i,j)=m.at<float>(i,j);
    return eigenMat;
}

Eigen::Matrix4f toEigenMatrix4f(const cv::Mat &m){
    Eigen::Matrix4f eigenMat;
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            eigenMat(i,j)=m.at<float>(i,j);
    return eigenMat;
}

Eigen::Matrix3d toEigenMatrix3d(const cv::Mat &m){
    Eigen::Matrix3d eigenMat;
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            eigenMat(i,j)=m.at<float>(i,j);
    return eigenMat;
}

Eigen::Matrix3f toEigenMatrix3f(const cv::Mat &m){
    Eigen::Matrix3f eigenMat;
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            eigenMat(i,j)=m.at<float>(i,j);
    return eigenMat;
}

Eigen::Vector4d toEigenVector4d(const cv::Mat &m){
    Eigen::Vector4d eigenVec;
    for(int i=0;i<4;i++)
        eigenVec(i)=m.at<float>(i);
    return eigenVec;
}

Eigen::Vector4f toEigenVector4f(const cv::Mat &m){
    Eigen::Vector4f eigenVec;
    for(int i=0;i<4;i++)
        eigenVec(i)=m.at<float>(i);
    return eigenVec;
}

Eigen::Vector3d toEigenVector3d(const cv::Mat &m){
    Eigen::Vector3d eigenVec;
    for(int i=0;i<3;i++)
        eigenVec(i)=m.at<float>(i);
    return eigenVec;
}

Eigen::Vector3f toEigenVector3f(const cv::Mat &m){
    Eigen::Vector3f eigenVec;
    for(int i=0;i<3;i++)
        eigenVec(i)=m.at<float>(i);
    return eigenVec;
}

Eigen::Vector3d toEigenVector3d(const cv::Point3d &cvPoint){
    Eigen::Vector3d eigenVec;
    eigenVec(0)=cvPoint.x;
    eigenVec(1)=cvPoint.y;
    eigenVec(2)=cvPoint.z;
    return eigenVec;
}

Eigen::Vector3f toEigenVector3f(const cv::Point3f &cvPoint){
    Eigen::Vector3f eigenVec;
    eigenVec(0)=cvPoint.x;
    eigenVec(1)=cvPoint.y;
    eigenVec(2)=cvPoint.z;
    return eigenVec;
}

Eigen::Vector2d toEigenVector2d(const cv::Point2d &cvPoint){
    Eigen::Vector2d eigenVec;
    eigenVec(0)=cvPoint.x;
    eigenVec(1)=cvPoint.y;
    return eigenVec;
}

Eigen::Vector2f toEigenVector2f(const cv::Point2f &cvPoint){
    Eigen::Vector2f eigenVec;
    eigenVec(0)=cvPoint.x;
    eigenVec(1)=cvPoint.y;
    return eigenVec;
}

Eigen::Vector3f RotationtoVector(const Eigen::Matrix3f &R){
    sensorData::SO3f so3(R);
    return so3.log();
}

Eigen::Vector3f RotationtoEuler(const Eigen::Matrix3f &R){
    Eigen::Vector3f euler;
    euler(0)=atan2(R(2,1),R(2,2));
    euler(1)=asin(-R(2,0));
    euler(2)=atan2(R(1,0),R(0,0));
    return euler;
}

Eigen::Quaternionf RotationtoQuaternion(const Eigen::Matrix3f &R){
    return Eigen::Quaternionf(R);
}

Eigen::Matrix3f EulertoRotation(const Eigen::Vector3f &euler){
    Eigen::Matrix3f R =(
        Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitZ())).matrix();
    return R;
}

Eigen::Quaternionf EulertoQuaternion(const Eigen::Vector3f &euler){
    Eigen::Matrix3f R = EulertoRotation(euler);
    return Eigen::Quaternionf(R);
}

Eigen::Vector3f EulertoVector(const Eigen::Vector3f &euler){
    Eigen::Matrix3f R = EulertoRotation(euler);
    return RotationtoVector(R);
}

Eigen::Matrix3f QuaterniontoRotation(const Eigen::Quaternionf &q){
    return q.toRotationMatrix();
}

Eigen::Vector3f QuaterniontoEuler(const Eigen::Quaternionf &q){
    Eigen::Matrix3f R = QuaterniontoRotation(q);
    return RotationtoEuler(R);
}

Eigen::Vector3f QuaterniontoVector(const Eigen::Quaternionf &q){
    sensorData::SO3f so3(q);
    return so3.log();
}

Eigen::Matrix3f VectortoMatrix(const Eigen::Vector3f &theta){
    sensorData::SO3f so3 = sensorData::SO3f::exp(theta);
    return so3.matrix();
}

Eigen::Quaternionf VectortoQuaternion(const Eigen::Vector3f &theta){
    sensorData::SO3f so3 = sensorData::SO3f::exp(theta);
    return so3.quaternion();
}

Eigen::Vector3f VectortoEuler(const Eigen::Vector3f &theta){
    auto rotation = sensorData::SO3f::exp(theta).matrix();
    Eigen::Vector3f euler = RotationtoEuler(rotation);
    return euler;
}

}
