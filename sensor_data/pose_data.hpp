#pragma once
#include <Eigen/Dense>
#include <iomanip>
#include "../common/logger.hpp"
#include "Infrastructure.hpp"


namespace sensorData
{

enum class sort_options : unsigned char { timestampe = 0, eucl_dist = 1};

class PoseData
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp = 0.0;
    Eigen::Quaternionf rot = Eigen::Quaternionf(Eigen::Matrix3f::Identity());
    Eigen::Vector3f posi = Eigen::Vector3f::Zero();

    struct{
        Eigen::Vector3f v = Eigen::Vector3f::Zero();
        Eigen::Vector3f w = Eigen::Vector3f::Zero();
    } vel;

    struct{
        sort_options sort_by = sort_options::timestampe;
    } options;

public:
    // 默认构造函数
    PoseData(){};

    // 拷贝构造函数
    PoseData(const PoseData &p) : timestamp(p.timestamp), rot(p.rot), posi(p.posi), vel(p.vel), options(p.options){};

    // 移动构造函数
    PoseData(PoseData &&p) : timestamp(p.timestamp), rot(p.rot), posi(p.posi), vel(p.vel), options(p.options){};

    // 自定义构造函数
    PoseData(double &timestamp_, Eigen::Vector3f &posi_, Eigen::Quaternionf &rot_, Eigen::Vector3f &lin_velo, Eigen::Vector3f &ang_velo) : timestamp(timestamp_), rot(rot_), posi(posi_)
    {
        vel.v = lin_velo;
        vel.w = ang_velo;
    };

    // 通过函数的形式访问/写入成员变量，统一sensor_data的类型接口
    inline double GetTimestamp(){ return timestamp; };
    inline Eigen::Vector3f GetPosition(){ return posi; };
    inline Eigen::Quaternionf GetRotation(){ return rot; };
    inline Eigen::Matrix3f GetRotionMatrix() { return rot.toRotationMatrix(); };
    inline Eigen::Vector3f GetVelocity(){ return vel.v; };
    inline Eigen::Vector3f GetAngularVelocity(){ return vel.w; };
    inline sort_options GetSortOption(){ return options.sort_by; };

    inline void SetTimestamp(double timestamp){ this->timestamp = timestamp; };
    inline void SetPosition(Eigen::Vector3f posi){ this->posi = posi; };
    inline void SetRotation(Eigen::Quaternionf rot){ this->rot = rot; };
    inline void SetVelocity(Eigen::Vector3f vel){ this->vel.v = vel; };
    inline void SetAngularVelocity(Eigen::Vector3f w){ this->vel.w = w; };
    inline void SetSortOption(sort_options sort_by){ this->options.sort_by = sort_by; };

    // 获取位姿矩阵(Matrix4f类型)
    Eigen::Matrix4f GetPoseMatrixf(){
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3, 3>(0, 0) = rot.toRotationMatrix();
        pose.block<3, 1>(0, 3) = posi;
        return pose;
    };
    // 获取位姿矩阵(Matrix4d类型)
    Eigen::Matrix4d GetPoseMatrixd(){
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose.block<3, 3>(0, 0) = rot.matrix().cast<double>();
        pose.block<3, 1>(0, 3) = posi.cast<double>();
        return pose;
    };

    // 操作符重载：赋值构造函数
    PoseData& operator = (const PoseData &p){
        this->timestamp = p.timestamp;
        this->rot = p.rot;
        this->posi = p.posi;
        this->vel = p.vel;
        this->options = p.options;
        return *this;
    };

    // 操作符重载：比较操作符用于排序，sort_by枚举类作为排序的依据
    bool operator>(const PoseData &pose_data2) const {
        if (options.sort_by == sort_options::timestampe)
            return timestamp > pose_data2.timestamp;
        else if (options.sort_by == sort_options::eucl_dist)
            return (posi - pose_data2.posi).norm() > (posi - pose_data2.posi).norm();
        else
            return false;
    };

    bool operator < (const PoseData &pose_data2) const{
        if (options.sort_by == sort_options::timestampe)
            return timestamp < pose_data2.timestamp;
        else if (options.sort_by == sort_options::eucl_dist)
            return (posi - pose_data2.posi).norm() < (posi - pose_data2.posi).norm();
        else
            return false;
    };

    // 操作符重载：输出流重载，便于输出PoseData类的信息
    friend std::ostream& operator << (std::ostream &os, PoseData &pd){
        os << std::setprecision(15) << "[" << pd.timestamp << "]PoseData Info:\n\t position: " << pd.posi.transpose() << "\n\t rotation: " << pd.rot.coeffs().transpose() << "\n\t linear velocity: " << pd.vel.v.transpose() << "\n\t angular velocity: " << pd.vel.w.transpose() << std::endl;
        return os;
    };
};

} // namespace sensorData
