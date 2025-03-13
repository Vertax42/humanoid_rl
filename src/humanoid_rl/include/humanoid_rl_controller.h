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

#ifndef HUMANOID_RL_CONTROLLER_H
#define HUMANOID_RL_CONTROLLER_H

#include "humanoid_rl_version.h"
#include "humanoid_utils.h"
#include "utils_common.h"
#include <core/session/onnxruntime_cxx_api.h>
#include <geometry_msgs/Twist.h>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <string>
#include <torch/script.h>
#include <torch/torch.h>
#include <yaml-cpp/yaml.h>

enum class ControlState : int8_t { DAMPING, ZERO, STAND, WALK }; // control state enum

enum class InferenceType : int8_t { ONNX, LIBTORCH }; // inference type enum

struct Proprioception {
    vector_t joint_pos;
    vector_t joint_vel;
    vector3_t base_ang_vel;
    vector3_t base_euler_xyz;
    vector3_t projected_gravity;
};

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngle {
    double roll, pitch, yaw;
};

struct ControlConfig {
    struct RobotConfig {
        int total_joints_num;
        int upper_body_joints_num;
        int leg_joints_num;
    };

    struct InfrenceConfig {
        InferenceType inference_type;
        std::string model_path;
        bool sw_mode;
        bool use_lpf;
        int decimation;
        int obs_dim;
        int action_dim;
        int yaw_history_length;
        int obs_history_length;
        double lin_sensitivity;
        double ang_sensitivity;
        double cycle_time;
        double action_scale;
        double max_action_clip;
        double min_action_clip;
        double cmd_threshold;
    };

    struct ObsConfig {
        double lin_vel;
        double ang_vel;
        double dof_pos;
        double dof_vel;
        double quat;
    };

    struct ImuConfig {
        double bias_x;
        double bias_y;
        double bias_z;
    };

    // joint_conf["init_state"/"stiffness"/"damping"][joint_name]
    std::vector<std::string> ordered_obs_names;                       // observation names in order
    std::vector<std::string> ordered_action_names;                    // action names in order
    std::vector<std::string> ordered_joint_names;                     // joint names in order
    std::map<std::string, std::map<std::string, double> > joint_conf; // joint configuration
    ObsConfig obs_config;                                             // observation configuration
    RobotConfig robot_config;                                         // robot configuration
    InfrenceConfig inference_config;                                  // inference configuration
    ImuConfig imu_config;                                             // imu configuration
};

namespace {
// Joint state callback constants
static const int MEANLESS_SIZE = 6;    // meaningless data size
static const int TOTAL_JOINTS = 36;    // total joints number
static const int QUATERNION_SIZE = 4;  // quaternion size
static const int ANGULAR_VEL_SIZE = 3; // angular velocity size
static const int IMU_ACCEL_SIZE = 3;   // imu acceleration size
}

class HumanoidRLController {
public:
    HumanoidRLController(ros::NodeHandle &nh, const ControlConfig &control_config);
    ~HumanoidRLController();

    void SetMode(const ControlState state);
    void SetBodyStateData(const std_msgs::Float64MultiArray::ConstPtr &msg); // for rl observation
    void SetJointStateDataBag(const sensor_msgs::JointState::ConstPtr &msg); // for whole body teleoperation from bag
    void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);        // for command input

    ControlState GetMode();
    std::string GetCurrentMode();
    bool IsReady();
    void GetJointCmdData(std_msgs::Float64MultiArray &cmd_msg);
    double GetJointLowerLimit(int index);
    double GetJointUpperLimit(int index);
    double GetJointTorqueLimit(int index);
    bool CheckJointLimit(double scale);

    std_msgs::Float64MultiArray BodyJointCommandWraper(std::vector<double> &pos_des, std::vector<double> &vel_des,
                                                       std::vector<double> &kp, std::vector<double> &kd,
                                                       std::vector<double> &torque);


private:
    void LoadModel();
    void PrintLibtorchModel(const torch::jit::script::Module &model);
    void PrintOnnxModel(const std::unique_ptr<Ort::Session> &session_ptr);
    void Update();

    void HandleDampingMode();
    void HandleZeroMode();
    void HandleStandMode();
    void HandleWalkMode();
    std::string StateToString(ControlState state);

    void UpdateStateEstimation();
    void ComputeObservation();
    void ComputeAction();
    void ComputeTorque();

    // legcay
    std::array<double, 3> quat_rotate_inverse(const std::array<double, 4> &quat, const std::array<double, 3> &vel);


public:
    ros::NodeHandle &nh_;

    mutable std::shared_mutex state_mutex_;
    std::vector<double> measured_q_;    // measured joint position 36
    std::vector<double> measured_v_;    // measured joint velocity 36
    std::vector<double> measured_tau_;  // measured joint torque 36
    std::array<double, 3> velocity_;    // linear velocity 3
    std::array<double, 4> quat_est_;    // estimated quaternion 4
    std::array<double, 3> angular_vel_; // angular velocity 3
    std::array<double, 3> imu_accel_;   // imu acceleration 3
    std::array<double, 3> imu_bias_;    // bias of the imu 3
    ros::Time last_time_{ 0, 0 };       // last time

    mutable std::shared_mutex cmd_mutex_;
    ros::Subscriber twist_sub_;
    ros::Subscriber bag_joints_sub_; // subscriber for bag data
    geometry_msgs::Twist cmd_data_;

    // bool is_update_{ false };
    mutable std::shared_mutex data_mutex_; // reserve for bag data
private:
    vector_t init_joint_pos_; // 30

    InferenceType inference_type_;
    // libtorch model
    torch::jit::script::Module model_;
    torch::Tensor torch_input_tensor_;
    torch::Tensor torch_action_tensor_;

    // onnx model
    std::unique_ptr<Ort::Session> session_ptr_;
    Ort::MemoryInfo memory_info_;
    std::vector<Ort::Value> onnx_input_tensor_;
    std::vector<Ort::Value> onnx_output_tensor_;

    std::vector<const char *> input_names_;
    std::vector<const char *> output_names_;
    std::vector<std::vector<int64_t> > input_shapes_;
    std::vector<std::vector<int64_t> > output_shapes_;

    // from rostopic
    ControlState current_state_;
    std::unordered_map<std::string, int32_t> obs_name_to_index_;    // 47
    std::unordered_map<std::string, int32_t> action_name_to_index_; // 12
    std::unordered_map<std::string, int32_t> joint_name_to_index_;  // 30

    // computed in algorithm
    std::vector<float> actions_;      // 12
    std::vector<float> observations_; // 47 * history_length
public:
    std::atomic<bool> data_ready_{ false };
    std::atomic<uint64_t> data_stamp_{ 0 };
    uint64_t last_processed_stamp_{ 0 };
    std_msgs::Float64MultiArray last_cmd_msg_;

public:
    Proprioception propri_;

private:
    vector_t last_actions_;
    Eigen::Matrix<float, Eigen::Dynamic, 1> propri_history_buffer_;
    bool is_first_observation_ = true;
    int64_t cycle_count_ = 0;
    std::vector<digital_lp_filter<double> > lpf_filters_;

    // wraped control msg
    // std_msgs::Float64MultiArray control_msg_;
public:
    // parameters from config yaml
    ControlConfig control_config_;
    std::vector<double> pos_des_cmd_;
    std::vector<double> vel_des_cmd_;
    std::vector<double> kp_cmd_;
    std::vector<double> kd_cmd_;
    std::vector<double> torque_cmd_;

    // Mode transition control
    double trans_mode_percentage_ = 0.0;
    double trans_mode_duration_cycle_ = 500.0; // 0.01 * 500 = 5s
    vector_t current_joint_pos_;               // 30 dof, only record when SetMode is called

    // 用于从rosbag获取上肢关节数据
    // std::unordered_map<int, double> upper_body_joints_from_bag_; // 索引->位置 的映射
    // bool has_bag_data_ = false;                                  // 标记是否有来自bag的数据
    // bool use_bag_for_upper_body_ = true;                         // 是否使用bag数据控制上肢
};


#endif // HUMANOID_RL_CONTROLLER_H