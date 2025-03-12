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

#ifndef JOY_STICK_IMPL_H
#define JOY_STICK_IMPL_H

#include "joy.h"
#include <fstream>
#include <future>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>

struct FloatPub {
    std::string topic_name;
    std::vector<int> buttons;
    ros::Publisher pub;
};

struct ResetPub {
    std::string topic_name;
    std::vector<int> buttons;
};

struct TwistPub {
    std::string topic_name;
    std::vector<int> buttons;
    std::map<std::string, int> axis;
};

struct ServiceClient {
    std::string service_name;
    std::string interface_type;
    std::vector<int> buttons;
};

class JoyStickImpl {
public:
    JoyStickImpl(ros::NodeHandle &nh);
    ~JoyStickImpl();

    bool Init();
    void Shutdown();
    ros::NodeHandle nh_;

    // ros::Publisher state_start_pub_;
    // ros::Publisher state_zero_pub_;
    // ros::Publisher state_stand_pub_;
    // ros::Publisher state_walk_pub_;
    ros::Publisher twist_pub_;
    // ros::ServiceServer service_server_;
    // ros::ServiceClient service_client_;

public:
    ros::NodeHandle private_nh_;

    std::shared_ptr<Joy> joy_;
    std::atomic_bool is_running_{ true };
    std::promise<void> stop_sig_;


    std::vector<FloatPub> float_pubs_;
    std::vector<ResetPub> reset_pubs_;
    std::vector<TwistPub> joy_twist_pubs_;
    std::vector<TwistPub> hat_twist_pubs_;
    // std::vector<ServiceClient> service_clients_;

    std::string command_type_;
    int32_t freq_;
    ros::Timer timer_;
};

#endif // JOY_STICK_IMPL_H