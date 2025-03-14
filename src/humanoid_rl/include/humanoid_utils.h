/**
 * Copyright (c) [2025] XinChengYang <vertax@foxmail.com> <yaphetys@gmail.com>
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef HUMANOID_UTILS_H
#define HUMANOID_UTILS_H

#include <chrono>
#include <eigen3/Eigen/Dense>

using scalar_t = double;
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using quaternion_t = Eigen::Quaternion<scalar_t>;

template <typename T>
using feet_array_t = std::array<T, 4>;
using contact_flag_t = feet_array_t<bool>;

using namespace std::chrono;

bool Throttler(const time_point<high_resolution_clock> now, time_point<high_resolution_clock> &last,
               const milliseconds interval);

template <typename T>
class filter {
public:
    filter(void) {}
    virtual ~filter(void) {}
    virtual void input(T input_value) = 0;
    virtual T output(void) = 0;
    virtual void clear(void) = 0;
};

template <typename T>
class digital_lp_filter : public filter<T> {
public:
    digital_lp_filter(T w_c, T t_s);
    virtual ~digital_lp_filter(void);
    virtual void input(T input_value);
    virtual T output(void);
    virtual void update(void);
    virtual void set_wc(T w_c);
    virtual void set_ts(T t_s);
    virtual void clear(void);
    virtual void init(T init_data);

private:
    T Lpf_in_prev[2];
    T Lpf_out_prev[2];
    T Lpf_in1, Lpf_in2, Lpf_in3, Lpf_out1, Lpf_out2;
    T lpf_out;
    T wc;
    T ts;
};

/**
 * Compute the rotation matrix corresponding to euler angles zyx
 *
 * @param [in] eulerAnglesZyx
 * @return The corresponding rotation matrix
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> GetRotationMatrixFromZyxEulerAngles(const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles)
{
    const SCALAR_T z = eulerAngles(0);
    const SCALAR_T y = eulerAngles(1);
    const SCALAR_T x = eulerAngles(2);

    const SCALAR_T c1 = cos(z);
    const SCALAR_T c2 = cos(y);
    const SCALAR_T c3 = cos(x);
    const SCALAR_T s1 = sin(z);
    const SCALAR_T s2 = sin(y);
    const SCALAR_T s3 = sin(x);

    const SCALAR_T s2s3 = s2 * s3;
    const SCALAR_T s2c3 = s2 * c3;

    // clang-format off
  Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrix;
  rotationMatrix << c1 * c2,      c1 * s2s3 - s1 * c3,       c1 * s2c3 + s1 * s3,
                    s1 * c2,      s1 * s2s3 + c1 * c3,       s1 * s2c3 - c1 * s3,
                        -s2,                  c2 * s3,                   c2 * c3;
    // clang-format on
    return rotationMatrix;
}

template <typename T>
T Square(T a)
{
    return a * a;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> QuatToZyx(const Eigen::Quaternion<SCALAR_T> &q)
{
    Eigen::Matrix<SCALAR_T, 3, 1> zyx;

    SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
    zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                        Square(q.w()) + Square(q.x()) - Square(q.y()) - Square(q.z()));
    zyx(1) = std::asin(as);
    zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()),
                        Square(q.w()) - Square(q.x()) - Square(q.y()) + Square(q.z()));
    return zyx;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> QuatToXyz(const Eigen::Quaternion<SCALAR_T> &q)
{
    Eigen::Matrix<SCALAR_T, 3, 1> xyz;

    // Roll (X-axis rotation)
    SCALAR_T sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    SCALAR_T cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    xyz(0) = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (Y-axis rotation)
    SCALAR_T sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if(std::abs(sinp) >= 1)
        xyz(1) = std::copysign(M_PI / 2, sinp); // 使用copysign来处理极端情况
    else
        xyz(1) = std::asin(sinp);

    // Yaw (Z-axis rotation)
    SCALAR_T siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    SCALAR_T cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    xyz(2) = std::atan2(siny_cosp, cosy_cosp);

    return xyz;
}

bool isArmJoint(const std::string &joint_name);

#endif // HUMANOID_UTILS_H

