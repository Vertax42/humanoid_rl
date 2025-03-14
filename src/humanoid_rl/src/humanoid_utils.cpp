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

#include "humanoid_utils.h"

bool Throttler(const time_point<high_resolution_clock> now, time_point<high_resolution_clock> &last,
               const milliseconds interval)
{
    auto elapsed = now - last;

    if(elapsed >= interval)
    {
        last = now;
        return true;
    }
    return false;
}

template <typename T>
digital_lp_filter<T>::digital_lp_filter(T w_c, T t_s)
{
    Lpf_in_prev[0] = Lpf_in_prev[1] = 0;
    Lpf_out_prev[0] = Lpf_out_prev[1] = 0;
    Lpf_in1 = 0, Lpf_in2 = 0, Lpf_in3 = 0, Lpf_out1 = 0, Lpf_out2 = 0;
    wc = w_c;
    ts = t_s;
    update();
}

template <typename T>
void digital_lp_filter<T>::update(void)
{
    double den = 2500 * ts * ts * wc * wc + 7071 * ts * wc + 10000;

    Lpf_in1 = 2500 * ts * ts * wc * wc / den;
    Lpf_in2 = 5000 * ts * ts * wc * wc / den;
    Lpf_in3 = 2500 * ts * ts * wc * wc / den;
    Lpf_out1 = -(5000 * ts * ts * wc * wc - 20000) / den;
    Lpf_out2 = -(2500 * ts * ts * wc * wc - 7071 * ts * wc + 10000) / den;
}

template <typename T>
digital_lp_filter<T>::~digital_lp_filter(void)
{}

template <typename T>
void digital_lp_filter<T>::input(T lpf_in)
{
    lpf_out = Lpf_in1 * lpf_in + Lpf_in2 * Lpf_in_prev[0] + Lpf_in3 * Lpf_in_prev[1] + // input component
              Lpf_out1 * Lpf_out_prev[0] + Lpf_out2 * Lpf_out_prev[1];                 // output component
    Lpf_in_prev[1] = Lpf_in_prev[0];
    Lpf_in_prev[0] = lpf_in;
    Lpf_out_prev[1] = Lpf_out_prev[0];
    Lpf_out_prev[0] = lpf_out;
}

template <typename T>
T digital_lp_filter<T>::output(void)
{
    return lpf_out;
}

template <typename T>
void digital_lp_filter<T>::set_ts(T t_s)
{
    ts = t_s;
    update();
}

template <typename T>
void digital_lp_filter<T>::set_wc(T w_c)
{
    wc = w_c;
    update();
}

template <typename T>
void digital_lp_filter<T>::clear(void)
{
    Lpf_in_prev[1] = 0;
    Lpf_in_prev[0] = 0;
    Lpf_out_prev[1] = 0;
    Lpf_out_prev[0] = 0;
}

template <typename T>
void digital_lp_filter<T>::init(T init_data)
{
    Lpf_in_prev[1] = init_data;
    Lpf_in_prev[0] = init_data;
    Lpf_out_prev[1] = init_data;
    Lpf_out_prev[0] = init_data;
}

template class digital_lp_filter<double>;
template class digital_lp_filter<float>;

bool isArmJoint(const std::string &joint_name)
{
    // keywords for arm joints
    static const std::vector<std::string> ARM_KEYWORDS = { "shoulder", "arm", "elbow", "wrist" };

    // check if the joint name contains any of the keywords
    for(const auto &keyword : ARM_KEYWORDS)
    {
        if(joint_name.find(keyword) != std::string::npos)
        {
            return true;
        }
    }
    return false;
}