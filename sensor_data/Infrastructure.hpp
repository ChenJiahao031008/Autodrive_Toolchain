#pragma once

#include <iostream>
#include <memory>
#include <cassert>
#include "../common/logger.hpp"

namespace sensorData
{

template <typename T>
class CircularQueue
{
public:
    int d_front, d_rear, d_size, d_maxsize;
    std::unique_ptr<T[]> d_arr;

public:
    CircularQueue();                       // 默认构造函数
    CircularQueue(int size);               // 自定义构造函数
    CircularQueue(const CircularQueue &q); // 拷贝构造函数
    CircularQueue(CircularQueue &&q);      // 移动构造函数
    ~CircularQueue();                      // 析构函数

    T Deque();                  // 踢队操作
    void Enque(const T &value); // 入队操作
    T front() const;            // 查看队列首端元素
    T rear() const;             // 查看队列末端元素
    T DataPrev(int idx) const;  // 查看上一个元素
    T DataNext(int idx) const;  // 查看下一个元素

    bool isEmpty() const; // 队列是否非空
    bool isFull() const;  // 队列是否已满

    int GetMaxSize() const; // 获得最大尺寸
    const int size();       // 返回队列已有数量
    const T *buffer();      // 返回缓存（传统指针）

    //打印队列
    template <class R>
    friend std::ostream &operator<<(std::ostream &, CircularQueue<R> &);
};

template <typename T>
class SO3
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 静态成员变量初始化 对浮点型数据无法使用const进行修饰，只能采用constexpr修饰
    static constexpr float SMALL_EPS = 1e-10;
    static const int DoF = 3;

public:
    // 各种构造函数
    SO3() { unit_quaternion_.setIdentity(); };
    SO3(const SO3 &other) : unit_quaternion_(other.unit_quaternion_) {};
    SO3(SO3 &&other) : unit_quaternion_(other.unit_quaternion_){};
    SO3(const Eigen::Matrix<T, 3, 3> &R) : unit_quaternion_(R){};
    SO3(const Eigen::Quaternion<T> &quat) : unit_quaternion_(quat)
    {
        assert(unit_quaternion_.squaredNorm() > SMALL_EPS);
        unit_quaternion_.normalize();
    };

    // SO3 inverse
    SO3 inverse() const{ return SO3(unit_quaternion_.conjugate()); };

    // SO3 -> so3
    Eigen::Matrix<T, 3, 1> log() const;
    static Eigen::Matrix<T, 3, 1> log(const SO3 &so3);
    // so3 -> SO3
    SO3 exp() const;
    static SO3 exp(const Eigen::Matrix<T, 3, 1> &omega);

    // Vector -> Lie algebra
    static Eigen::Matrix<T, 3, 3> hat(const Eigen::Matrix<T, 3, 1> &omega);
    // Lie algebra -> Vector
    static Eigen::Matrix<T, 3, 1> vee(const Eigen::Matrix<T, 3, 3> &Omega);

    // SO3转换为旋转矩阵
    Eigen::Matrix<T, 3, 3> matrix() const {
        return unit_quaternion_.toRotationMatrix();
    };

    // SO3转换为四元数
    Eigen::Quaternion<T> quaternion() const {
        return unit_quaternion_;
    };

    // 重载 = 符号，进行赋值构造
    void operator = (const SO3 &so3){
        this->unit_quaternion_ = so3.unit_quaternion_;
    }
    SO3 operator = (const SO3 &&so3)
    {
        return SO3(so3.unit_quaternion_);
    }

    // 重载 *= 符号：使用 SO3 进行旋转变换
    void operator *= (const SO3 &so3){
        unit_quaternion_ *= so3.unit_quaternion_;
        unit_quaternion_.normalize();
    };

    // 重载 * 符号：使用 SO3 进行旋转变换
    SO3 operator * (const SO3 &so3) const{
        SO3 result(*this);
        result.unit_quaternion_ *= so3.unit_quaternion_;
        result.unit_quaternion_.normalize();
        return result;
    };

    // 重载 * 符号：使用 SO3/四元数 旋转一个点
    Eigen::Matrix<T, 3, 1> operator*(const Eigen::Matrix<T, 3, 1> &xyz) const {
            return unit_quaternion_._transformVector(xyz);
    };

    // 重载 << 符号：打印 SO3
    friend std::ostream &operator << (std::ostream &os, const SO3 &so3){
        os << "Stored in quaternion(xyzw): " << so3.unit_quaternion_.x() << " " << so3.unit_quaternion_.y() << " " << so3.unit_quaternion_.z() << " " << so3.unit_quaternion_.w() << std::endl;
        return os;
    };

protected:
    Eigen::Quaternion<T> unit_quaternion_;
};

using SO3d = SO3<double>;
using SO3f = SO3<float>;

}


#include "Infrastructure.inl"

