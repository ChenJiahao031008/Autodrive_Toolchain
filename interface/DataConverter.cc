#include "DataConverter.h"

namespace interface
{

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

Eigen::Vector3f RotationtoVector(const Eigen::Matrix3f &R)
{
    sensorData::SO3f so3(R);
    return so3.log();
}
Eigen::Vector3d RotationtoVector(const Eigen::Matrix3d &R)
{
    sensorData::SO3d so3(R);
    return so3.log();
}

Eigen::Vector3f RotationtoEuler(const Eigen::Matrix3f &R){
    Eigen::Vector3f euler;
    euler(0)=atan2(R(2,1),R(2,2));
    euler(1)=asin(-R(2,0));
    euler(2)=atan2(R(1,0),R(0,0));
    return euler;
}
Eigen::Vector3d RotationtoEuler(const Eigen::Matrix3d &R)
{
    Eigen::Vector3d euler;
    euler(0) = atan2(R(2, 1), R(2, 2));
    euler(1) = asin(-R(2, 0));
    euler(2) = atan2(R(1, 0), R(0, 0));
    return euler;
}

Eigen::Quaternionf RotationtoQuaternion(const Eigen::Matrix3f &R){
    return Eigen::Quaternionf(R);
}

Eigen::Quaterniond RotationtoQuaternion(const Eigen::Matrix3d &R)
{
    return Eigen::Quaterniond(R);
}

Eigen::Matrix3f EulertoRotation(const Eigen::Vector3f &euler){
    Eigen::Matrix3f R =(
        Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitZ())).matrix();
    return R;
}

Eigen::Matrix3d EulertoRotation(const Eigen::Vector3d &euler)
{
    Eigen::Matrix3d R = (Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitZ()))
                            .matrix();
    return R;
}

Eigen::Quaternionf EulertoQuaternion(const Eigen::Vector3f &euler)
{
    Eigen::Matrix3f R = EulertoRotation(euler);
    return Eigen::Quaternionf(R);
}
Eigen::Quaterniond EulertoQuaternion(const Eigen::Vector3d &euler)
{
    Eigen::Matrix3d R = EulertoRotation(euler);
    return Eigen::Quaterniond(R);
}

Eigen::Vector3f EulertoVector(const Eigen::Vector3f &euler){
    Eigen::Matrix3f R = EulertoRotation(euler);
    return RotationtoVector(R);
}

Eigen::Vector3d EulertoVector(const Eigen::Vector3d &euler)
{
    Eigen::Matrix3d R = EulertoRotation(euler);
    return RotationtoVector(R);
}

Eigen::Matrix3f QuaterniontoRotation(const Eigen::Quaternionf &q){
    return q.toRotationMatrix();
}

Eigen::Matrix3d QuaterniontoRotation(const Eigen::Quaterniond &q)
{
    return q.toRotationMatrix();
}

Eigen::Vector3f QuaterniontoEuler(const Eigen::Quaternionf &q){
    Eigen::Matrix3f R = QuaterniontoRotation(q);
    return RotationtoEuler(R);
}
Eigen::Vector3d QuaterniontoEuler(const Eigen::Quaterniond &q)
{
    Eigen::Matrix3d R = QuaterniontoRotation(q);
    return RotationtoEuler(R);
}

Eigen::Vector3f QuaterniontoVector(const Eigen::Quaternionf &q){
    sensorData::SO3f so3(q);
    return so3.log();
}
Eigen::Vector3d QuaterniontoVector(const Eigen::Quaterniond &q)
{
    sensorData::SO3d so3(q);
    return so3.log();
}

Eigen::Matrix3f VectortoMatrix(const Eigen::Vector3f &theta){
    sensorData::SO3f so3 = sensorData::SO3f::exp(theta);
    return so3.matrix();
}
Eigen::Matrix3d VectortoMatrix(const Eigen::Vector3d &theta)
{
    sensorData::SO3d so3 = sensorData::SO3d::exp(theta);
    return so3.matrix();
}

Eigen::Quaternionf VectortoQuaternion(const Eigen::Vector3f &theta){
    sensorData::SO3f so3 = sensorData::SO3f::exp(theta);
    return so3.quaternion();
}
Eigen::Quaterniond VectortoQuaternion(const Eigen::Vector3d &theta)
{
    sensorData::SO3d so3 = sensorData::SO3d::exp(theta);
    return so3.quaternion();
}

Eigen::Vector3f VectortoEuler(const Eigen::Vector3f &theta){
    auto rotation = sensorData::SO3f::exp(theta).matrix();
    Eigen::Vector3f euler = RotationtoEuler(rotation);
    return euler;
}
Eigen::Vector3d VectortoEuler(const Eigen::Vector3d &theta)
{
    auto rotation = sensorData::SO3d::exp(theta).matrix();
    Eigen::Vector3d euler = RotationtoEuler(rotation);
    return euler;
}
}
