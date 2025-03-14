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

#ifndef HUMANOID_RL_INFERENCE_H
#define HUMANOID_RL_INFERENCE_H

#include "humanoid_rl_controller.h"
#include "humanoid_utils.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <map>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>
#include <unordered_map>
#include <vector>


template <typename T>
T clip(T value, T min, T max);

class HumanoidRLInference {
public:
    HumanoidRLInference(ros::NodeHandle &nh);
    ~HumanoidRLInference();

    bool Init();
    bool InitSubscribers(YAML::Node &cfg_node);
    bool PrintControlConfig(const ControlConfig &config);
    bool CheckRobotState(float limit_scale, float gravity_threshold);
    void Shutdown();

public:
    ros::NodeHandle &nh_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber joint_telep_state_sub_; // used for rosbag

    ros::Publisher policy_pub_;
    // state machine
    ros::Subscriber state_start_sub_;
    ros::Subscriber state_zero_sub_;
    ros::Subscriber state_stand_sub_;
    ros::Subscriber state_walk_sub_;
    ros::Subscriber state_move_to_fix_pos_sub_;
    ros::Subscriber state_bag_sub_;

public:
    // callback funcs
    void jointsCallback(const std_msgs::Float64MultiArray::ConstPtr &msg); // used for real robot state callback
    void bagJointsCallback(const sensor_msgs::JointState::ConstPtr &msg);  // used for rosbag callback
    void stateStartCallback(const std_msgs::Bool::ConstPtr &msg);          // damping
    void stateZeroCallback(const std_msgs::Bool::ConstPtr &msg);           // zero
    void stateStandCallback(const std_msgs::Bool::ConstPtr &msg);          // stand
    void stateWalkCallback(const std_msgs::Bool::ConstPtr &msg);           // walk
    void stateToFixPosCallback(const std_msgs::Bool::ConstPtr &msg);      // arm_motion
    void stateBagPlayCallback(const std_msgs::Bool::ConstPtr &msg);        // bag_play

    std::string stateToString(ControlState state);
    //   [zero]←--→[stand]
    //    ↑     /    ↑
    //    |    /     |
    //    ↓   ↓      ↓
    //  [damping]←—[walk]
    time_point<high_resolution_clock> last_set_start_time_;
    time_point<high_resolution_clock> last_set_zero_time_;
    time_point<high_resolution_clock> last_set_stand_time_;
    time_point<high_resolution_clock> last_set_walk_time_;


public:
    std::unique_ptr<HumanoidRLController> rl_controller_;
    // public for main loop
    int32_t freq_;
    std::atomic<bool> run_flag_{ true };
};
#endif // HUMANOID_RL_INFERENCE_H
