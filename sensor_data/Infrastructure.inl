#include "Infrastructure.hpp"

namespace sensorData
{

// #define USE_COUT

/* —————————————————— CircularQueue模板类的实现部分 —————————————————— */
template <class T>
CircularQueue<T>::CircularQueue()
    : d_front(0), d_rear(0), d_size(0), d_maxsize(20)
{
    d_arr = std::unique_ptr<T[]>(new T[d_maxsize](), std::default_delete<T[]>());
}

template <class T>
CircularQueue<T>::CircularQueue(int size) : d_front(0), d_rear(0), d_size(0), d_maxsize(size)
{
    d_arr = std::unique_ptr<T[]>(new T[d_maxsize](), std::default_delete<T[]>());
}

template <class T>
CircularQueue<T>::CircularQueue(const CircularQueue &q)
: d_front(q.d_front), d_rear(q.d_rear), d_size(q.d_size), d_maxsize(q.d_maxsize)
{
    d_arr = std::unique_ptr<T[]>(new T[d_maxsize], std::default_delete<T[]>());
    for (int i=0; i < d_maxsize; ++i){
        d_arr[i] = q.d_arr[i];
    }
}

template <class T>
CircularQueue<T>::CircularQueue(CircularQueue &&q)
    : d_front(q.d_front), d_rear(q.d_rear), d_size(q.d_size), d_maxsize(q.d_maxsize)
{
    d_arr.swap(q.d_arr);
}

template <class T>
CircularQueue<T>::~CircularQueue()
{
    d_arr.reset();
}

template <class T>
inline bool CircularQueue<T>::isFull() const
{
    if (d_rear == 0){
        return d_size == d_maxsize;
    }
    return (d_rear) % d_maxsize == d_front;
}

template <class T>
inline bool CircularQueue<T>::isEmpty() const
{
    return d_size == 0;
}

template <class T>
inline int CircularQueue<T>::GetMaxSize() const{
    return d_maxsize;
}

template <class T>
inline T CircularQueue<T>::Deque()
{
    if (isEmpty()){
        AERROR << "Circle Queue is Empty.";
        return T();
    }
    T ret = d_arr[d_front];
    d_front = (d_front + 1) % d_maxsize;
    d_size--;
    return ret;
}

template <class T>
inline void CircularQueue<T>::Enque(const T &value)
{
    if (isFull()){
        d_arr[d_rear] = value;
        d_rear = (d_rear + 1) % d_maxsize;
        d_front = (d_front + 1) % d_maxsize;
    }else{
        d_arr[d_rear] = value;
        d_rear = (d_rear + 1) % d_maxsize;
        d_size++;
    }

    // AINFO << "--> value: " << value << "; and --> d_size: " << d_size;
}

template <class T>
inline T CircularQueue<T>::front() const
{
    if (isEmpty()){
        AERROR << "Circle Queue is Empty.";
        return T();
    }
    return d_arr[d_front];
}

template <class T>
inline T CircularQueue<T>::rear() const
{
    if (isEmpty()){
        AERROR << "Circle Queue is Empty.";
        return T();
    }
    int tmp = (d_rear == 0 ? d_maxsize - 1 : d_rear - 1);
    return d_arr[tmp];
}

template <class T>
inline void CircularQueue<T>::clear(){
    d_front = 0;
    d_rear = 0;
    d_size = 0;
}

template <class T>
inline T CircularQueue<T>::DataPrev(int idx) const
{
    if (isEmpty())
    {
        AERROR << "Circle Queue is Empty.";
        return T();
    }
    if (idx == 0)
        return d_arr[d_maxsize - 1];
    else
        return d_arr[idx - 1];

}

template <class T>
inline T CircularQueue<T>::DataNext(int idx) const
{
    if (isEmpty())
    {
        AERROR << "Circle Queue is Empty.";
        return T();
    }
    if (idx == d_maxsize - 1)
        return d_arr[0];
    else
        return d_arr[idx + 1];
}

template <class T>
inline const int CircularQueue<T>::size()
{
    return d_size;
}

template <class T>
inline const T *CircularQueue<T>::buffer()
{
    return d_arr.get();
}

template <class R>
inline std::ostream &operator<<(std::ostream &os, CircularQueue<R> &q)
{
    if (q.isEmpty()){
#ifdef USE_COUT
        os << "Circle Queue is Empty.";
#endif
        return os;
    }else{
        auto i = q.d_front;
        auto end = (q.d_rear == 0 ? q.d_maxsize - 1 : q.d_rear - 1);
        while (i != end){
            os << q.d_arr[i] << " ";
            if (i == q.d_maxsize - 1)
                i = 0;
            else
                i++;
        }
        os << q.d_arr[i];
    }
    return os;
}

/* —————————————————— SO3模板类的实现部分 —————————————————— */

template <class T>
inline Eigen::Matrix<T, 3, 3> SO3<T>::hat(const Eigen::Matrix<T, 3, 1> &omega)
{
    Eigen::Matrix<T, 3, 3> ret;
    ret << 0, -omega(2, 0), omega(1, 0),
           omega(2, 0), 0, -omega(0, 0),
          -omega(1, 0), omega(0, 0), 0;
    return ret;
}

template <class T>
inline Eigen::Matrix<T, 3, 1> SO3<T>::vee(const Eigen::Matrix<T, 3, 3> &Omega)
{
    assert(fabs(Omega(2, 1) + Omega(1, 2)) < SMALL_EPS);
    assert(fabs(Omega(0, 2) + Omega(2, 0)) < SMALL_EPS);
    assert(fabs(Omega(1, 0) + Omega(0, 1)) < SMALL_EPS);
    Eigen::Matrix<T, 3, 1> ret;
    ret << Omega(2, 1), Omega(0, 2), Omega(1, 0);
    return ret;
}

template <class T>
inline SO3<T> SO3<T>::exp() const
{
    return SO3<T>(unit_quaternion_);
}

template <class T>
inline SO3<T> SO3<T>::exp(const Eigen::Matrix<T, 3, 1> &omega)
{
    // ***************************************************************** //
    // * 实现部分公式表述为：
    // * q = cos(|\omega| / 2) + sin(|\omega| / 2) / |\omega| * \omega
    // ***************************************************************** //
    T theta = omega.norm();
    T half_theta = 0.5 * theta;

    T imag_factor;
    T real_factor = cos(half_theta);
    if (theta < SMALL_EPS)
    {
        T theta_sq = theta * theta;
        T theta_po4 = theta_sq * theta_sq;
        imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4;
    }
    else
    {
        T sin_half_theta = sin(half_theta);
        imag_factor = sin_half_theta / theta;
    }

    return SO3<T>(Eigen::Quaternion<T>(real_factor,
                                       imag_factor * omega.x(),
                                       imag_factor * omega.y(),
                                       imag_factor * omega.z()));
}

template <class T>
inline Eigen::Matrix<T, 3, 1> SO3<T>::log() const
{
    return SO3<T>::log(*this);
}

template <class T>
inline Eigen::Matrix<T, 3, 1> SO3<T>::log(const SO3 &so3)
{
    // ***************************************************************** //
    // * 实现部分参考论文： Integrating Generic Sensor Fusion Algorithms withSound State Representations through Encapsulation of Manifolds, 2011. 这也是sophus库的实现方法
    // * 实现部分公式表述为：
    // * \theta = arctan(v / w) / v * V if v != 0, w != 0
    // *        = 0                     if v == 0
    // *        = (+-pi / 2) / v * V    if w == 0
    // ***************************************************************** //
    T v = so3.unit_quaternion_.vec().norm();
    T w = so3.unit_quaternion_.w();
    T squared_w = w * w;

    T two_atan_nbyw_by_v;

    if (v < SMALL_EPS && fabs(w) > SMALL_EPS)
    {
        two_atan_nbyw_by_v = 2.0 / w - 2.0 * (v * v) / (w * squared_w);
    }
    else if (v >= SMALL_EPS)
    {
        if (fabs(w) < SMALL_EPS)
            two_atan_nbyw_by_v = w > 0 ? M_PI / v : -M_PI / v;
        else
            two_atan_nbyw_by_v = 2 * atan(v / w) / v;
    }else{
        assert("SO3 Log Failed: v < SMALL_EPS && fabs(w) <= SMALL_EPS");
    }

    // T theta = two_atan_nbyw_by_v * v;
    return two_atan_nbyw_by_v * so3.unit_quaternion_.vec();
}

} // namespace sensorData
