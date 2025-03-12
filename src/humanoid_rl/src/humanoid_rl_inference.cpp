
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

#include "humanoid_rl_inference.h"
#include "utils_logger.hpp"
#include <cstdlib>
#include <fcntl.h>
#include <mutex>
#include <stdlib.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

using namespace zsummer::log4z;

struct Command {
    // 1 for moving, 0 for standing used for yaw while stand
    double x, y, yaw, heading, move;
    Command(double _x = 0.0, double _y = 0.0, double _yaw = 0.0, double _heading = 0.64, double _move = 1.0)
        : x(_x), y(_y), yaw(_yaw), heading(_heading), move(_move)
    {}
} user_cmd;

template <typename T>
T clip(T value, T min, T max)
{
    return std::max(min, std::min(value, max));
}

HumanoidRLInference::HumanoidRLInference(ros::NodeHandle &nh) : nh_(nh) { Init(); }

HumanoidRLInference::~HumanoidRLInference() { LOGW("HumanoidRLInference object has been destroyed!"); }

void HumanoidRLInference::jointsCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    // LOGW("jointsCallback function called!");
    rl_controller_->SetBodyStateData(msg);
}

void HumanoidRLInference::bagJointsCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    rl_controller_->SetJointStateDataBag(msg);
}

std::string HumanoidRLInference::stateToString(ControlState state)
{
    switch(state)
    {
    case ControlState::DAMPING:
        return "DAMPING";
    case ControlState::ZERO:
        return "ZERO";
    case ControlState::STAND:
        return "STAND";
    case ControlState::WALK:
        return "WALK";
    default:
        return "UNKNOWN";
    }
}

void HumanoidRLInference::stateStartCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if(!Throttler(std::chrono::high_resolution_clock::now(), last_set_start_time_, 1000ms))
    {
        LOGE("State start change throttled, ignoring request!");
        return;
    }
    if(rl_controller_->GetMode() == ControlState::ZERO)
    {
        rl_controller_->SetMode(ControlState::DAMPING);
        LOGW("[ZERO] -> [DAMPING]");
    } else if(rl_controller_->GetMode() == ControlState::DAMPING)
    {
        LOGW("Is already in DAMPING state!");
        LOGW("[DAMPING] -> [DAMPING]");
    } else
    {
        std::string current_state = stateToString(rl_controller_->GetMode());
        LOGFMTW("Iilegal state transition: [%s] -> [DAMPING]", current_state.c_str());
        LOGW("Keep the current state!");
        LOGFMTW("[%s] -> [%s]", current_state.c_str(), current_state.c_str());
    }
}

void HumanoidRLInference::stateZeroCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if(!Throttler(std::chrono::high_resolution_clock::now(), last_set_zero_time_, 1000ms))
    {
        LOGE("State zero change throttled, ignoring request!");
        return;
    }
    if(rl_controller_->GetMode() == ControlState::STAND)
    {
        rl_controller_->SetMode(ControlState::ZERO);
        LOGW("[STAND] -> [ZERO]");
    } else if(rl_controller_->GetMode() == ControlState::DAMPING)
    {
        rl_controller_->SetMode(ControlState::ZERO);
        LOGW("[DAMPING] -> [ZERO]");
    } else if(rl_controller_->GetMode() == ControlState::ZERO)
    {
        LOGW("Is already in ZERO state!");
        LOGW("[ZERO] -> [ZERO]");
    } else
    {
        std::string current_state = stateToString(rl_controller_->GetMode());
        LOGFMTW("Iilegal state transition: [%s] -> [ZERO]", current_state.c_str());
        LOGW("Keep the current state!");
        LOGFMTW("[%s] -> [%s]", current_state.c_str(), current_state.c_str());
    }
}

void HumanoidRLInference::stateStandCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if(!Throttler(std::chrono::high_resolution_clock::now(), last_set_stand_time_, 1000ms))
    {
        // LOGE("State stand change throttled, ignoring request!");
        return;
    }
    if(rl_controller_->GetMode() == ControlState::ZERO)
    {
        rl_controller_->SetMode(ControlState::STAND);
        LOGW("[ZERO] -> [STAND]");
    } else if(rl_controller_->GetMode() == ControlState::WALK)
    {
        rl_controller_->SetMode(ControlState::STAND);
        LOGW("[WALK] -> [STAND]");
    } else if(rl_controller_->GetMode() == ControlState::STAND)
    {
        LOGW("Is already in STAND state!");
        LOGW("[STAND] -> [STAND]");
    } else
    {
        std::string current_state = stateToString(rl_controller_->GetMode());
        LOGFMTW("Iilegal state transition: [%s] -> [STAND]", current_state.c_str());
        LOGW("Keep the current state!");
        LOGFMTW("[%s] -> [%s]", current_state.c_str(), current_state.c_str());
    }
}

void HumanoidRLInference::stateWalkCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if(!Throttler(std::chrono::high_resolution_clock::now(), last_set_walk_time_, 1000ms))
    {
        // LOGE("State walk change throttled, ignoring request!");
        return;
    }
    if(rl_controller_->GetMode() == ControlState::STAND)
    {
        rl_controller_->SetMode(ControlState::WALK);
        LOGW("[STAND] -> [WALK]");
    } else if(rl_controller_->GetMode() == ControlState::WALK)
    {
        LOGW("Is already in WALK state!");
        LOGW("[WALK] -> [WALK]");
    } else
    {
        std::string current_state = stateToString(rl_controller_->GetMode());
        LOGFMTW("Iilegal state transition: [%s] -> [WALK]", current_state.c_str());
        LOGW("Keep the current state!");
        LOGFMTW("[%s] -> [%s]", current_state.c_str(), current_state.c_str());
    }
}

bool HumanoidRLInference::InitSubscribers(YAML::Node &cfg_node)
{
    try
    {
        // joint state subscriber
        joint_state_sub_
            = nh_.subscribe(cfg_node["sub_controllers_policy_output_name"].as<std::string>(), 1,
                            &HumanoidRLInference::jointsCallback, this, ros::TransportHints().tcpNoDelay());

        // joint teleported state subscriber
        joint_telep_state_sub_
            = nh_.subscribe(cfg_node["sub_controllers_joints_from_bag_name"].as<std::string>(), 1,
                            &HumanoidRLInference::bagJointsCallback, this, ros::TransportHints().tcpNoDelay());

        state_start_sub_ = nh_.subscribe(cfg_node["sub_state_machine_start_name"].as<std::string>(), 1,
                                         &HumanoidRLInference::stateStartCallback, this);

        // zero state machine subscriber
        state_zero_sub_ = nh_.subscribe(cfg_node["sub_state_machine_zero_name"].as<std::string>(), 1,
                                        &HumanoidRLInference::stateZeroCallback, this);

        // stand state machine subscriber
        state_stand_sub_ = nh_.subscribe(cfg_node["sub_state_machine_stand_name"].as<std::string>(), 1,
                                         &HumanoidRLInference::stateStandCallback, this);

        // walk state machine subscriber
        state_walk_sub_ = nh_.subscribe(cfg_node["sub_state_machine_walk_name"].as<std::string>(), 1,
                                        &HumanoidRLInference::stateWalkCallback, this);

        LOGD("Successfully initialized all subscribers!");
        return true;
    } catch(const std::exception &e)
    {
        LOGFMTE("Failed to initialize subscribers, %s", e.what());
        return false;
    }
}

bool HumanoidRLInference::Init()
{
    try
    {
        LOGI("Attempting to load YAML config file...");
        std::string package_path = ros::package::getPath("humanoid_rl");
        std::string config_path = package_path + "/config/rl_xbot.yaml";
        LOGFMTI("Config path: %s", config_path.c_str());

        // 检查文件是否存在
        std::ifstream f(config_path.c_str());
        if(!f.good())
        {
            LOGFMTE("Config file does not exist or cannot be accessed: %s", config_path.c_str());
            return false;
        }

        YAML::Node cfg_node = YAML::LoadFile(config_path);
        LOGD("Loaded yaml file!");
        freq_ = cfg_node["control_frequency"].as<int32_t>();
        LOGD("Loaded freq_");
        // ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ init controller ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

        // ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ joint_conf ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
        ControlConfig control_conf;
        control_conf.joint_conf["init_state"]
            = cfg_node["control_conf"]["joint_conf"]["init_state"].as<std::map<std::string, double> >();
        control_conf.joint_conf["stiffness"]
            = cfg_node["control_conf"]["joint_conf"]["stiffness"].as<std::map<std::string, double> >();
        control_conf.joint_conf["damping"]
            = cfg_node["control_conf"]["joint_conf"]["damping"].as<std::map<std::string, double> >();
        control_conf.joint_conf["upper_limit"]
            = cfg_node["control_conf"]["joint_conf"]["upper_limit"].as<std::map<std::string, double> >();
        control_conf.joint_conf["lower_limit"]
            = cfg_node["control_conf"]["joint_conf"]["lower_limit"].as<std::map<std::string, double> >();
        control_conf.joint_conf["torque_limit"]
            = cfg_node["control_conf"]["joint_conf"]["torque_limit"].as<std::map<std::string, double> >();
        LOGD("Loaded joint_conf");

        // ordered_obs_names
        control_conf.ordered_obs_names.clear();
        for(auto iter = cfg_node["control_conf"]["observations"].begin();
            iter != cfg_node["control_conf"]["observations"].end(); iter++)
        {
            control_conf.ordered_obs_names.push_back(iter->first.as<std::string>());
        }
        LOGD("Loaded ordered_obs_names");

        // ordered_action_names
        control_conf.ordered_action_names.clear();
        for(auto iter = cfg_node["control_conf"]["actions"].begin(); iter != cfg_node["control_conf"]["actions"].end();
            iter++)
        {
            control_conf.ordered_action_names.push_back(iter->first.as<std::string>());
        }
        LOGD("Loaded ordered_action_names");

        // ordered_joint_names
        control_conf.ordered_joint_names.clear();
        for(auto iter = cfg_node["control_conf"]["joint_conf"]["init_state"].begin();
            iter != cfg_node["control_conf"]["joint_conf"]["init_state"].end(); iter++)
        {
            control_conf.ordered_joint_names.push_back(iter->first.as<std::string>());
        }
        LOGD("Loaded ordered_joint_names");
        // ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ robot_conf ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
        control_conf.robot_config.total_joints_num
            = cfg_node["control_conf"]["robot_conf"]["total_joints_num"].as<int>();
        control_conf.robot_config.upper_body_joints_num
            = cfg_node["control_conf"]["robot_conf"]["upper_body_joints_num"].as<int>();
        control_conf.robot_config.leg_joints_num = cfg_node["control_conf"]["robot_conf"]["leg_joints_num"].as<int>();
        LOGD("Loaded robot_config");
        // ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ inference_conf ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
        std::string model_name = cfg_node["control_conf"]["inference_conf"]["model_name"].as<std::string>();
        if(model_name.find(".pt") != std::string::npos || model_name.find(".jit") != std::string::npos)
        {
            control_conf.inference_config.inference_type = InferenceType::LIBTORCH;
        } else if(model_name.find(".onnx") != std::string::npos)
        {
            control_conf.inference_config.inference_type = InferenceType::ONNX;
        } else
        {
            LOGFMTE("Unsupported model type: %s", model_name.c_str());
            return false;
        }
        std::string model_path = package_path + "/model/" + model_name;
        control_conf.inference_config.model_path = model_path;

        LOGFMTD("Loaded model_path: %s, inference_type: %s", control_conf.inference_config.model_path.c_str(),
                control_conf.inference_config.inference_type == InferenceType::LIBTORCH ? "LIBTORCH" : "ONNX");
        control_conf.inference_config.sw_mode = cfg_node["control_conf"]["inference_conf"]["sw_mode"].as<bool>();
        control_conf.inference_config.use_lpf = cfg_node["control_conf"]["inference_conf"]["use_lpf"].as<bool>();
        control_conf.inference_config.decimation = cfg_node["control_conf"]["inference_conf"]["decimation"].as<int>();
        control_conf.inference_config.obs_dim = cfg_node["control_conf"]["inference_conf"]["obs_dim"].as<int>();
        control_conf.inference_config.action_dim = cfg_node["control_conf"]["inference_conf"]["action_dim"].as<int>();
        control_conf.inference_config.yaw_history_length
            = cfg_node["control_conf"]["inference_conf"]["yaw_history_length"].as<int>();
        control_conf.inference_config.obs_history_length
            = cfg_node["control_conf"]["inference_conf"]["obs_history_length"].as<int>();
        control_conf.inference_config.lin_sensitivity
            = cfg_node["control_conf"]["inference_conf"]["lin_sensitivity"].as<double>();
        control_conf.inference_config.ang_sensitivity
            = cfg_node["control_conf"]["inference_conf"]["ang_sensitivity"].as<double>();
        control_conf.inference_config.cycle_time
            = cfg_node["control_conf"]["inference_conf"]["cycle_time"].as<double>();
        control_conf.inference_config.action_scale
            = cfg_node["control_conf"]["inference_conf"]["action_scale"].as<double>();
        control_conf.inference_config.max_action_clip
            = cfg_node["control_conf"]["inference_conf"]["max_action_clip"].as<double>();
        control_conf.inference_config.min_action_clip
            = cfg_node["control_conf"]["inference_conf"]["min_action_clip"].as<double>();
        control_conf.inference_config.cmd_threshold
            = cfg_node["control_conf"]["inference_conf"]["cmd_threshold"].as<double>();
        // ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ obs_conf ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
        control_conf.obs_config.lin_vel = cfg_node["control_conf"]["obs_scales"]["lin_vel"].as<double>();
        control_conf.obs_config.ang_vel = cfg_node["control_conf"]["obs_scales"]["ang_vel"].as<double>();
        control_conf.obs_config.dof_pos = cfg_node["control_conf"]["obs_scales"]["dof_pos"].as<double>();
        control_conf.obs_config.dof_vel = cfg_node["control_conf"]["obs_scales"]["dof_vel"].as<double>();
        control_conf.obs_config.quat = cfg_node["control_conf"]["obs_scales"]["quat"].as<double>();
        LOGD("Loaded obs_config");

        // ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ imu_conf ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
        control_conf.imu_config.bias_x = cfg_node["control_conf"]["imu_conf"]["bias_x"].as<double>();
        control_conf.imu_config.bias_y = cfg_node["control_conf"]["imu_conf"]["bias_y"].as<double>();
        control_conf.imu_config.bias_z = cfg_node["control_conf"]["imu_conf"]["bias_z"].as<double>();
        LOGD("Loaded imu_config");

        PrintControlConfig(control_conf);
        rl_controller_ = std::make_unique<HumanoidRLController>(nh_, control_conf);
        // ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

        // ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ init subscriber ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
        last_set_start_time_ = high_resolution_clock::now();
        last_set_zero_time_ = high_resolution_clock::now();
        last_set_stand_time_ = high_resolution_clock::now();
        last_set_walk_time_ = high_resolution_clock::now();

        // register subscribers
        if(!InitSubscribers(cfg_node))
        {
            LOGE("Failed to initialize subscribers!");
            return false;
        }

        // register joint cmd publishers
        policy_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
            cfg_node["pub_controllers_policy_input_name"].as<std::string>(), 1);
    } catch(const std::exception &e)
    {
        LOGFMTE("HumanoidRLInference init failed, %s", e.what());
        return false;
    }
    LOGI("HumanoidRLInference init successed!");
    return true;
}

bool HumanoidRLInference::PrintControlConfig(const ControlConfig &config)
{
    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, false);

    LOGD("=============================================================");
    LOGD("Loaded ordered observation names:");
    // print obs names and index
    for(size_t i = 0; i < config.ordered_obs_names.size(); ++i)
    {
        LOGFMTD("Observation name: %s, index: %ld", config.ordered_obs_names[i].c_str(), i);
    }
    LOGD("=============================================================");

    std::cout << ANSI_COLOR_RED_BOLD << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "Loaded ordered observation names:" << std::endl;
    for(size_t i = 0; i < config.ordered_obs_names.size(); ++i)
    {
        std::cout << "    Observation name: " << config.ordered_obs_names[i] << ", index: " << i << std::endl;
    }
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << ANSI_COLOR_RESET
              << std::endl;

    LOGD("=============================================================");
    LOGD("Loaded ordered action names:");
    // print action names and index
    for(size_t i = 0; i < config.ordered_action_names.size(); ++i)
    {
        LOGFMTD("Action name: %s, index: %ld", config.ordered_action_names[i].c_str(), i);
    }
    LOGD("=============================================================");

    std::cout << ANSI_COLOR_BLUE_BOLD << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "Loaded ordered action names:" << std::endl;
    for(size_t i = 0; i < config.ordered_action_names.size(); ++i)
    {
        std::cout << "    Action name: " << config.ordered_action_names[i] << ", index: " << i << std::endl;
    }
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << ANSI_COLOR_RESET
              << std::endl;

    LOGD("=============================================================");
    LOGD("Loaded ordered joint names:");
    // print joint names and index
    for(size_t i = 0; i < config.ordered_joint_names.size(); ++i)
    {
        LOGFMTD("Joint name: %s, index: %ld", config.ordered_joint_names[i].c_str(), i);
    }
    LOGD("=============================================================");

    std::cout << ANSI_COLOR_YELLOW_BOLD << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "Loaded ordered joint names:" << std::endl;
    for(size_t i = 0; i < config.ordered_joint_names.size(); ++i)
    {
        std::cout << "    Joint name: " << config.ordered_joint_names[i] << ", index: " << i << std::endl;
    }
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << ANSI_COLOR_RESET
              << std::endl;

    LOGD("=============================================================");
    LOGD("Loaded robot configuration:");
    LOGFMTD("  Total joints number: %d", config.robot_config.total_joints_num);
    LOGFMTD("  Upper body joints number: %d", config.robot_config.upper_body_joints_num);
    LOGFMTD("  Leg joints number: %d", config.robot_config.leg_joints_num);
    LOGD("=============================================================");

    std::cout << ANSI_COLOR_BLUE_BOLD << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "Loaded robot configuration:" << std::endl;
    std::cout << "    Total joints number: " << config.robot_config.total_joints_num << std::endl;
    std::cout << "    Upper body joints number: " << config.robot_config.upper_body_joints_num << std::endl;
    std::cout << "    Leg joints number: " << config.robot_config.leg_joints_num << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << ANSI_COLOR_RESET
              << std::endl;

    LOGD("=============================================================");
    LOGD("Loaded inference configuration:");
    LOGFMTD("  model_path: %s", config.inference_config.model_path.c_str());
    std::string sw_mode = config.inference_config.sw_mode ? "True" : "False";
    LOGFMTD("  sw_mode: %s", sw_mode.c_str());
    LOGFMTD("  decimation: %d", config.inference_config.decimation);
    LOGFMTD("  obs_dim: %d", config.inference_config.obs_dim);
    LOGFMTD("  action_dim: %d", config.inference_config.action_dim);
    LOGFMTD("  yaw_history_length: %d", config.inference_config.yaw_history_length);
    LOGFMTD("  obs_history_length: %d", config.inference_config.obs_history_length);
    LOGFMTD("  lin_sensitivity: %f", config.inference_config.lin_sensitivity);
    LOGFMTD("  ang_sensitivity: %f", config.inference_config.ang_sensitivity);
    LOGFMTD("  cycle_time: %f", config.inference_config.cycle_time);
    LOGFMTD("  action_scale: %f", config.inference_config.action_scale);
    LOGFMTD("  max_action_clip: %f", config.inference_config.max_action_clip);
    LOGFMTD("  min_action_clip: %f", config.inference_config.min_action_clip);
    LOGFMTD("  cmd_threshold: %f", config.inference_config.cmd_threshold);
    LOGD("=============================================================");

    std::cout << ANSI_COLOR_GREEN_BOLD << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "Loaded inference configuration:" << std::endl;
    std::cout << "    model_path: " << config.inference_config.model_path << std::endl;
    std::cout << "    sw_mode: " << sw_mode << std::endl;
    std::cout << "    decimation: " << config.inference_config.decimation << std::endl;
    std::cout << "    obs_dim: " << config.inference_config.obs_dim << std::endl;
    std::cout << "    action_dim: " << config.inference_config.action_dim << std::endl;
    std::cout << "    yaw_history_length: " << config.inference_config.yaw_history_length << std::endl;
    std::cout << "    obs_history_length: " << config.inference_config.obs_history_length << std::endl;
    std::cout << "    lin_sensitivity: " << config.inference_config.lin_sensitivity << std::endl;
    std::cout << "    ang_sensitivity: " << config.inference_config.ang_sensitivity << std::endl;
    std::cout << "    cycle_time: " << config.inference_config.cycle_time << std::endl;
    std::cout << "    action_scale: " << config.inference_config.action_scale << std::endl;
    std::cout << "    max_action_clip: " << config.inference_config.max_action_clip << std::endl;
    std::cout << "    min_action_clip: " << config.inference_config.min_action_clip << std::endl;
    std::cout << "    cmd_threshold: " << config.inference_config.cmd_threshold << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << ANSI_COLOR_RESET
              << std::endl;

    LOGD("=============================================================");
    LOGD("Loaded Obs_scale configuration:");
    LOGFMTD("  lin_vel_scale: %f", config.obs_config.lin_vel);
    LOGFMTD("  ang_vel_scale: %f", config.obs_config.ang_vel);
    LOGFMTD("  dof_pos_scale: %f", config.obs_config.dof_pos);
    LOGFMTD("  dof_vel_scale: %f", config.obs_config.dof_vel);
    LOGFMTD("  quat_scale: %f", config.obs_config.quat);
    LOGD("=============================================================");

    std::cout << ANSI_COLOR_GRAY_BOLD << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "Loaded Obs_scale configuration:" << std::endl;
    std::cout << "    lin_vel_scale: " << config.obs_config.lin_vel << std::endl;
    std::cout << "    ang_vel_scale: " << config.obs_config.ang_vel << std::endl;
    std::cout << "    dof_pos_scale: " << config.obs_config.dof_pos << std::endl;
    std::cout << "    dof_vel_scale: " << config.obs_config.dof_vel << std::endl;
    std::cout << "    quat_scale: " << config.obs_config.quat << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << ANSI_COLOR_RESET
              << std::endl;

    LOGD("=============================================================");
    LOGD("Loaded IMU configuration:");
    LOGFMTD("  bias_x: %f", config.imu_config.bias_x);
    LOGFMTD("  bias_y: %f", config.imu_config.bias_y);
    LOGFMTD("  bias_z: %f", config.imu_config.bias_z);
    LOGD("=============================================================");

    std::cout << ANSI_COLOR_RED_BOLD << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "Loaded IMU configuration:" << std::endl;
    std::cout << "    bias_x: " << config.imu_config.bias_x << std::endl;
    std::cout << "    bias_y: " << config.imu_config.bias_y << std::endl;
    std::cout << "    bias_z: " << config.imu_config.bias_z << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << ANSI_COLOR_RESET
              << std::endl;

    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, true);
    return true;
}

bool HumanoidRLInference::CheckRobotState(float limit_scale, float gravity_threshold)
{
    if(!rl_controller_->CheckJointLimit(limit_scale))
    {
        LOGE("Joint position out of limit!");
        return false;
    } else if(rl_controller_->propri_.projected_gravity(2) > gravity_threshold)
    {
        LOGE("Projected gravity is too small, robot is about to fall down!");
        return false;
    } else
    {
        return true;
    }
}

void HumanoidRLInference::Shutdown()
{
    LOGW("Shutting down HumanoidRLInference...");
    run_flag_.store(false);
    ros::shutdown();
}

int main(int argc, char **argv)
{
    // log4z
    ILog4zManager::getRef().start();
    ILog4zManager::getRef().setLoggerLevel(LOG4Z_MAIN_LOGGER_ID, LOG_LEVEL_DEBUG);

    printf_program("Humanoid Policy Inference: for humanoid robot velocity rl policy inference on Xbot-L robot.");
    common_tools::printf_software_version("Humanoid Policy Inference");
    common_tools::dump_program_info_log4z("Humanoid Policy Inference");
    common_tools::clean_log_files(6);

    ros::init(argc, argv, "humanoid_policy_inference");
    ros::NodeHandle nh;

    HumanoidRLInference rl_inference(nh);
    // ros::spinOnce();
    ros::Rate rate(rl_inference.freq_);
    while(ros::ok() && rl_inference.run_flag_)
    {
        if(rl_inference.rl_controller_->IsReady())
        {
            break;
        }
        LOGA("Waiting for the RL Controller to be ready...");
        rate.sleep();
        ros::spinOnce();
    }

    LOGA("******************Start RL Inference main loop*******************");

    // std::chrono::time_point<std::chrono::high_resolution_clock> cycle_start_time;
    // std::chrono::time_point<std::chrono::high_resolution_clock> cycle_end_time;
    // std::chrono::duration<double, std::milli> cycle_duration;

    while(ros::ok() && rl_inference.run_flag_)
    {
        // start time record
        auto cycle_start_time = std::chrono::high_resolution_clock::now();
        ros::spinOnce();

        std_msgs::Float64MultiArray msg;
        // unusual joint limit && projected gravity check
        if(!rl_inference.CheckRobotState(0.9, -0.8))
        {
            std::vector<double> pos_des_cmd(rl_inference.rl_controller_->control_config_.robot_config.total_joints_num,
                                            0.0);
            std::vector<double> vel_des_cmd(rl_inference.rl_controller_->control_config_.robot_config.total_joints_num,
                                            0.0);
            std::vector<double> kp_cmd(rl_inference.rl_controller_->control_config_.robot_config.total_joints_num, 0.0);
            std::vector<double> kd_cmd(rl_inference.rl_controller_->control_config_.robot_config.total_joints_num, 2.0);
            std::vector<double> torque_cmd(rl_inference.rl_controller_->control_config_.robot_config.total_joints_num,
                                           0.0);
            msg = rl_inference.rl_controller_->BodyJointCommandWraper(pos_des_cmd, vel_des_cmd, kp_cmd, kd_cmd,
                                                                      torque_cmd);
            rl_inference.policy_pub_.publish(msg);
            rl_inference.Shutdown();
            break;
        }

        rl_inference.rl_controller_->GetJointCmdData(msg);
        std::string current_state = rl_inference.rl_controller_->GetCurrentMode();

        rl_inference.policy_pub_.publish(msg);


        auto duration_time = std::chrono::high_resolution_clock::now() - cycle_start_time;
        // cycle_duration = cycle_end_time - cycle_start_time; // duration in milliseconds

        LOGFMTI("Cycle processing time: %.3f ms, humanoid robot is now in %s mode",
                std::chrono::duration_cast<std::chrono::microseconds>(duration_time).count() / 1000.0,
                current_state.c_str());
        // std::cout << "Cycle processing time: " << micro_sec / 1000.0 << " ms, humanoid robot is now in "
        //           << current_state.c_str() << " mode." << std::endl;
        // auto duration = std::chrono::high_resolution_clock::now() - cycle_start_time;
        auto sleep_time = std::chrono::microseconds(1000000 / rl_inference.freq_) - duration_time;
        if(sleep_time > std::chrono::microseconds::zero())
        {
            std::this_thread::sleep_for(sleep_time);
        }
    }
    LOGA("Exit RL Inference main loop!");
    LOGA("Ros ShutDown!");
    return 0;
}