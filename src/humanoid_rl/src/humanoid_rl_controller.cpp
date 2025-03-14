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

#include "humanoid_rl_controller.h"


using namespace zsummer::log4z;

EulerAngle QuaternionToEuler(const Quaternion &q)
{
    EulerAngle angles;

    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if(std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

HumanoidRLController::HumanoidRLController(ros::NodeHandle &nh, const ControlConfig &control_config)
    : nh_(nh), memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
    control_config_ = control_config;
    // LoadModel
    LoadModel();

    actions_.resize(control_config_.inference_config.action_dim);
    observations_.resize(control_config_.inference_config.obs_dim
                         * control_config_.inference_config.obs_history_length);
    std::fill(observations_.begin(), observations_.end(), 0.0f); // init observations to zero

    last_actions_.resize(control_config_.inference_config.action_dim);
    last_actions_.setZero();
    propri_.projected_gravity(2) = -0.981; // init projected gravity vector
    propri_history_buffer_.resize(control_config_.inference_config.obs_dim
                                  * control_config_.inference_config.obs_history_length);

    init_joint_pos_.resize(control_config_.robot_config.total_joints_num);

    imu_bias_[0] = control_config_.imu_config.bias_x;
    imu_bias_[1] = control_config_.imu_config.bias_y;
    imu_bias_[2] = control_config_.imu_config.bias_z;

    for(int i = 0; i < control_config_.inference_config.action_dim; ++i)
    {
        init_joint_pos_(i) = control_config_.joint_conf["init_state"][control_config_.ordered_joint_names[i]];
    }

    current_joint_pos_.resize(control_config_.robot_config.total_joints_num); // curent joint position
    current_state_ = ControlState::DAMPING;                                   // init to damping mode

    obs_name_to_index_.clear();
    joint_name_to_index_.clear();
    action_name_to_index_.clear();
    cycle_count_ = 0;
    lpf_filters_.clear();
    twist_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &HumanoidRLController::TwistCmdCallback, this);
    for(int i = 0; i < control_config_.inference_config.action_dim; ++i)
    {
        lpf_filters_.emplace_back(100, 0.001);
    }

    pos_des_cmd_.resize(control_config_.robot_config.total_joints_num); // 30
    vel_des_cmd_.resize(control_config_.robot_config.total_joints_num); // 30
    kp_cmd_.resize(control_config_.robot_config.total_joints_num);      // 30
    kd_cmd_.resize(control_config_.robot_config.total_joints_num);      // 30
    torque_cmd_.resize(control_config_.robot_config.total_joints_num);  // 30
    if(control_config_.inference_config.inference_type == InferenceType::LIBTORCH)
    {
        torch_input_tensor_ = torch::zeros(
            { 1, control_config_.inference_config.obs_dim * control_config_.inference_config.obs_history_length },
            torch::kFloat32);
        for(int i = 0; i < 100; i++) // warmup
        {
            torch_action_tensor_ = model_.forward({ torch_input_tensor_ }).toTensor();
        }
        LOGFMTI("Libtorch model warmup completed");
    } else if(control_config_.inference_config.inference_type == InferenceType::ONNX)
    {
        onnx_input_tensor_.clear();
        onnx_input_tensor_.push_back(Ort::Value::CreateTensor<float>(memory_info_, observations_.data(),
                                                                     observations_.size(), input_shapes_[0].data(),
                                                                     input_shapes_[0].size()));
        for(int i = 0; i < 100; i++) // warmup
        {
            onnx_output_tensor_ = session_ptr_->Run(Ort::RunOptions{}, input_names_.data(), onnx_input_tensor_.data(),
                                                    1, output_names_.data(), 1);
        }
        LOGFMTI("Onnx model warmup completed");
    }

    bag_seq_
        = std::make_unique<HumanoidRLBag>(control_config_.bag_config.bag_name, control_config_.bag_config.bag_topic,
                                          control_config_.bag_config.bag_rate); // bag sequence object
}

HumanoidRLController::~HumanoidRLController() { LOGD("HumanoidRLController object has been destroyed!"); }

void HumanoidRLController::LoadModel()
{
    if(control_config_.inference_config.inference_type == InferenceType::LIBTORCH)
    {
        // Load libtorch model
        try
        {
            model_ = torch::jit::load(control_config_.inference_config.model_path);
            model_.eval();
        } catch(const c10::Error &e)
        {
            LOGFMTF("Failed to load model: %s", control_config_.inference_config.model_path.c_str());
            throw std::runtime_error("Failed to load model" + control_config_.inference_config.model_path);
        }
        PrintLibtorchModel(model_);
    } else if(control_config_.inference_config.inference_type == InferenceType::ONNX)
    {
        // Create env && Load onnx model
        try
        {
            std::shared_ptr<Ort::Env> onnxEnvPtr(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "HumanoidRLOnnxController"));
            Ort::SessionOptions session_options;
            session_options.SetInterOpNumThreads(1);
            session_ptr_ = std::make_unique<Ort::Session>(
                *onnxEnvPtr, control_config_.inference_config.model_path.c_str(), session_options);

            // Get input and output info
            input_names_.clear();
            output_names_.clear();
            input_shapes_.clear();
            output_shapes_.clear();

            // Get input info
            Ort::AllocatorWithDefaultOptions allocator;
            for(size_t i = 0; i < session_ptr_->GetInputCount(); ++i)
            {
                // onnxruntime v1.8.1 api GetInputName
                const char *input_name = session_ptr_->GetInputName(i, allocator);
                input_names_.push_back(input_name);
                input_shapes_.push_back(session_ptr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
                LOGFMTD("Onnx Input %zu: %s", i, input_names_[i]);
                LOGFMTD("Input shape: [%s]",
                        std::accumulate(std::next(input_shapes_[i].begin()), input_shapes_[i].end(),
                                        std::to_string(input_shapes_[i][0]),
                                        [](std::string a, int64_t b) { return a + ", " + std::to_string(b); })
                            .c_str());
            }

            // Get output info
            for(size_t i = 0; i < session_ptr_->GetOutputCount(); ++i)
            {
                // onnxruntime v1.8.1 api GetOutputName
                const char *output_name = session_ptr_->GetOutputName(i, allocator);
                output_names_.push_back(output_name);
                output_shapes_.push_back(session_ptr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
                LOGFMTD("Onnx Output %zu: %s", i, output_names_[i]);
                LOGFMTD("Output shape: [%s]",
                        std::accumulate(std::next(output_shapes_[i].begin()), output_shapes_[i].end(),
                                        std::to_string(output_shapes_[i][0]),
                                        [](std::string a, int64_t b) { return a + ", " + std::to_string(b); })
                            .c_str());
            }
        } catch(const Ort::Exception &e)
        {
            LOGFMTE("Failed to create onnx session: %s", e.what());
            throw std::runtime_error("Failed to create onnx session: " + std::string(e.what()));
        }
        PrintOnnxModel(session_ptr_);
    } else
    {
        LOGFMTE("Loaded model from: %s failed!", control_config_.inference_config.model_path.c_str());
        throw std::runtime_error("Unsupported model type: " + control_config_.inference_config.model_path);
    }
}

void HumanoidRLController::PrintOnnxModel(const std::unique_ptr<Ort::Session> &session_ptr)
{
    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, false);
    LOGFMTA("Use onnx model for inference!: %s", control_config_.inference_config.model_path.c_str());

    // Get model metadata
    Ort::ModelMetadata model_metadata = session_ptr->GetModelMetadata();
    Ort::AllocatorWithDefaultOptions allocator;

    // Get model information
    auto model_name = model_metadata.GetGraphName(allocator);
    auto description = model_metadata.GetDescription(allocator);
    auto domain = model_metadata.GetDomain(allocator);
    auto version = "v1.8.1";

    std::cout << ANSI_COLOR_PURPLE_BOLD << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    LOGD("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");

    // Print model information
    LOGFMTD("Model Name: %s", model_name);
    LOGFMTD("Description: %s", description);
    LOGFMTD("Domain: %s", domain);
    LOGFMTD("Version: %s", version);

    std::cout << "    Model Name: " << model_name << std::endl;
    std::cout << "    Description: " << description << std::endl;
    std::cout << "    Domain: " << domain << std::endl;
    std::cout << "    Version: " << version << std::endl;

    // Get producer information
    auto producer_name = model_metadata.GetProducerName(allocator);
    LOGFMTD("Producer Name: %s", producer_name);
    std::cout << "    Producer Name: " << producer_name << std::endl;

    // print model inputs
    std::cout << "\n    Model Inputs:" << std::endl;
    LOGD("Model Inputs:");
    for(size_t i = 0; i < input_names_.size(); i++)
    {
        if(i >= input_names_.size() || !input_names_[i])
        {
            LOGFMTE("Invalid input name at index %zu", i);
            continue;
        }

        std::cout << "        Input " << i << ": " << input_names_[i] << " (";
        LOGFMTD("    Input %zu: %s (", i, input_names_[i]);

        // print shape
        if(i < input_shapes_.size())
        {
            for(size_t j = 0; j < input_shapes_[i].size(); j++)
            {
                std::cout << input_shapes_[i][j];
                if(j < input_shapes_[i].size() - 1) std::cout << ", ";
            }

            // record to log
            std::string shape_str;
            for(size_t j = 0; j < input_shapes_[i].size(); j++)
            {
                shape_str += std::to_string(input_shapes_[i][j]);
                if(j < input_shapes_[i].size() - 1) shape_str += ", ";
            }
            LOGFMTD("%s)", shape_str.c_str());
        } else
        {
            std::cout << "unknown shape";
            LOGFMTD("unknown shape)");
        }
        std::cout << ")" << std::endl;
    }

    // print model outputs
    std::cout << "\n    Model Outputs:" << std::endl;
    LOGD("Model Outputs:");
    for(size_t i = 0; i < output_names_.size(); i++)
    {
        if(i >= output_names_.size() || !output_names_[i])
        {
            LOGFMTE("Invalid output name at index %zu", i);
            continue;
        }

        std::cout << "        Output " << i << ": " << output_names_[i] << " (";
        LOGFMTD("    Output %zu: %s (", i, output_names_[i]);

        // print shape
        if(i < output_shapes_.size())
        {
            for(size_t j = 0; j < output_shapes_[i].size(); j++)
            {
                std::cout << output_shapes_[i][j];
                if(j < output_shapes_[i].size() - 1) std::cout << ", ";
            }

            // record to log
            std::string shape_str;
            for(size_t j = 0; j < output_shapes_[i].size(); j++)
            {
                shape_str += std::to_string(output_shapes_[i][j]);
                if(j < output_shapes_[i].size() - 1) shape_str += ", ";
            }
            LOGFMTD("%s)", shape_str.c_str());
        } else
        {
            std::cout << "unknown shape";
            LOGFMTD("unknown shape)");
        }
        std::cout << ")" << std::endl;
    }

    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << ANSI_COLOR_RESET
              << std::endl;
    LOGD("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");

    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, true);
}

void HumanoidRLController::PrintLibtorchModel(const torch::jit::script::Module &model)
{
    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, false);
    LOGFMTA("Use libtorch model for inference!: %s", control_config_.inference_config.model_path.c_str());
    size_t num_params = 0;
    for(const auto &pair : model.named_parameters())
    {
        num_params += pair.value.numel();
    }
    LOGFMTI("Loaded policy model has %lu parameters", num_params);
    size_t num_layers = 0;
    std::cout << ANSI_COLOR_PURPLE_BOLD << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    LOGD("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    for(const auto &pair : model.named_parameters())
    {
        const std::string &name = pair.name;
        const torch::Tensor &param = pair.value;
        std::ostringstream oss;
        oss << "[";
        for(size_t i = 0; i < param.sizes().size(); ++i)
        {
            oss << param.sizes()[i];
            if(i < param.sizes().size() - 1)
            {
                oss << ", ";
            }
        }
        oss << "]";
        LOGFMTD("Layer %lu: %s, shape: %s", num_layers, name.c_str(), oss.str().c_str());
        std::cout << "    Layer " << num_layers << ": " << name << ", shape: " << oss.str() << std::endl;
    }
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << ANSI_COLOR_RESET
              << std::endl;
    LOGD("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    LOGFMTI("Total number of parameters: %lu", num_params);
    LOGFMTI("Total number of layers: %lu", num_layers);
    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, true);
}

ControlState HumanoidRLController::GetMode()
{
    std::shared_lock<std::shared_mutex> lock(state_mutex_);
    return current_state_;
}

std::string HumanoidRLController::GetCurrentMode()
{
    std::shared_lock<std::shared_mutex> lock(state_mutex_);
    return StateToString(current_state_);
}

void HumanoidRLController::SetMode(const ControlState state)
{
    std::unique_lock<std::shared_mutex> lock(state_mutex_);
    current_state_ = state;
    trans_mode_percentage_ = 0.0;

    LOGFMTD("State machine switch, start from:");
    for(size_t j = 0; j < control_config_.ordered_joint_names.size(); ++j)
    {
        current_joint_pos_(j)
            = measured_q_[joint_name_to_index_[control_config_.ordered_joint_names[j]] + MEANLESS_SIZE];
        LOGFMTD("current_joint_pos_[%ld]: %f", j, current_joint_pos_(j));
    }
}

double HumanoidRLController::GetJointLowerLimit(int index)
{
    return control_config_.joint_conf["lower_limit"][control_config_.ordered_joint_names[index]];
}

double HumanoidRLController::GetJointUpperLimit(int index)
{
    return control_config_.joint_conf["upper_limit"][control_config_.ordered_joint_names[index]];
}

double HumanoidRLController::GetJointTorqueLimit(int index)
{
    return control_config_.joint_conf["torque_limit"][control_config_.ordered_joint_names[index]];
}

void HumanoidRLController::TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    std::unique_lock<std::shared_mutex> lock(cmd_mutex_);
    cmd_data_ = *msg;
    // LOGFMTD("Received twist command: (%f, %f, %f, %f, %f, %f)", cmd_data_.linear.x, cmd_data_.linear.y,
    // cmd_data_.linear.z, cmd_data_.angular.x, cmd_data_.angular.y, cmd_data_.angular.z);
}

void HumanoidRLController::SetBodyStateData(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    //  zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, false);
    std::unique_lock<std::shared_mutex> lock(state_mutex_);
    // LOGFMTD("Callback function called!");

    if(static_cast<int>(msg->data.size()) < TOTAL_JOINTS * 3 + QUATERNION_SIZE + ANGULAR_VEL_SIZE + IMU_ACCEL_SIZE)
    {
        LOGE("Invalid data size in Float64MultiArray message!");
        ROS_ERROR("Invalid data size in Float64MultiArray message!");
        return;
    }

    measured_q_.clear();
    measured_v_.clear();
    measured_tau_.clear();

    // LOGFMTD("Received data size: %lu", msg->data.size());
    // position, velocity, torque
    measured_q_.assign(msg->data.begin(), msg->data.begin() + TOTAL_JOINTS);
    // LOGFMTD("Assigned measured_q_'s size: %ld", measured_q_.size());

    measured_v_.assign(msg->data.begin() + TOTAL_JOINTS, msg->data.begin() + 2 * TOTAL_JOINTS);
    // LOGFMTD("Assigned measured_v_'s size: %ld", measured_v_.size());

    measured_tau_.assign(msg->data.begin() + 2 * TOTAL_JOINTS, msg->data.begin() + 3 * TOTAL_JOINTS);
    // LOGFMTD("Assigned measured_tau_'s size: %ld", measured_q_.size());

    // decode quaternion
    std::copy(msg->data.begin() + 3 * TOTAL_JOINTS, msg->data.begin() + 3 * TOTAL_JOINTS + QUATERNION_SIZE,
              quat_est_.begin()); // 使用 quat_est_.begin() 作为输出迭代器

    // LOGFMTD("Received quaternion: (%f, %f, %f, %f)", quat_est_[0], quat_est_[1], quat_est_[2], quat_est_[3]);

    // decode angular velocity
    std::copy(msg->data.begin() + 3 * TOTAL_JOINTS + QUATERNION_SIZE,
              msg->data.begin() + 3 * TOTAL_JOINTS + QUATERNION_SIZE + ANGULAR_VEL_SIZE,
              angular_vel_.begin()); // use angular_vel_.begin() as output iterator

    // LOGFMTD("Received angular velocity: (%f, %f, %f)", angular_vel_[0], angular_vel_[1], angular_vel_[2]);

    // decode imu linear acceleration
    std::copy(msg->data.begin() + 3 * TOTAL_JOINTS + QUATERNION_SIZE + ANGULAR_VEL_SIZE,
              msg->data.begin() + 3 * TOTAL_JOINTS + QUATERNION_SIZE + ANGULAR_VEL_SIZE + IMU_ACCEL_SIZE,
              imu_accel_.begin()); // use imu_accel_.begin() as output iterator
    // LOGFMTD("Received imu_accel: (%f, %f, %f)", imu_accel_[0], imu_accel_[1], imu_accel_[2]);
    // LOGE("Callback function!");
    if(measured_q_.empty() || measured_v_.empty() || measured_tau_.empty())
    {
        LOGE("Callback function Error!!!!!!!!!!!!!");
        LOGE("+++++++++++++++++++++++++++++++++++++++");
    }
    // // get current ros time
    // ros::Time current_time = ros::Time::now();
    // if(last_time_.isZero())
    // { // check if the last time is zero
    //     last_time_ = current_time;
    //     // initialize the last_time
    // }
    // double dt = (current_time - last_time_).toSec(); // calculate the time diff
    // // LOGFMTD("Time diff aka (current_time - last_time): %f", dt);
    // last_time_ = current_time; // update the last time
    // // LOGFMTD("Received imu_accel: (%f, %f, %f)", imu_accel_[0], imu_accel_[1], imu_accel_[2]);
    // // gravity compensation for IMU acceleration
    // std::array<double, 3> acceleration_corrected
    //     = { imu_accel_[0] + imu_bias_[0], imu_accel_[1] + imu_bias_[1], imu_accel_[2] + imu_bias_[2] };
    // // calculate linear velocity via IMU acceleration
    // for(size_t i = 0; i < 3; ++i)
    // {
    //     velocity_[i] += acceleration_corrected[i] * dt;
    // }
    // LOGFMTD("After integration, get velocity_[0]: %f, velocity_[1]: %f, velocity_[2]: %f", velocity_[0],
    // velocity_[1],velocity_[2]);

    if(obs_name_to_index_.empty())
    {
        for(size_t i = 0; i < control_config_.ordered_obs_names.size(); ++i)
        {
            obs_name_to_index_[control_config_.ordered_obs_names[i]] = i;
        }
    }
    // LOGFMTD("obs_name_to_index_ size: %ld", obs_name_to_index_.size());

    if(joint_name_to_index_.empty())
    {
        for(size_t i = 0; i < control_config_.ordered_joint_names.size(); ++i)
        {
            joint_name_to_index_[control_config_.ordered_joint_names[i]] = i;
        }
    }
    // LOGFMTD("joint_name_to_index_ size: %ld", joint_name_to_index_.size());

    if(action_name_to_index_.empty())
    {
        for(size_t i = 0; i < control_config_.ordered_action_names.size(); ++i)
        {
            action_name_to_index_[control_config_.ordered_action_names[i]] = i;
        }
    }
    data_stamp_++; // 增加数据戳
    data_ready_.store(true);
}

void HumanoidRLController::SetJointStateDataBag(const sensor_msgs::JointState::ConstPtr &msg)
{
    std::unique_lock<std::shared_mutex> lock(data_mutex_);
}

void HumanoidRLController::GetJointCmdData(std_msgs::Float64MultiArray &cmd_msg)
{
    if(data_ready_.load() && last_processed_stamp_ != data_stamp_.load())
    {
        Update();
        last_processed_stamp_ = data_stamp_.load();
    } else
    {
        LOGA("data is not ready");
    }

    cmd_msg = BodyJointCommandWraper(pos_des_cmd_, vel_des_cmd_, kp_cmd_, kd_cmd_, torque_cmd_);
}

std::array<double, 3> HumanoidRLController::quat_rotate_inverse(const std::array<double, 4> &quat,
                                                                const std::array<double, 3> &vel)
{
    // quaternion: w, x, y, z: local to world
    // velocity: x, y, z: local frame

    constexpr double two = 2.0; // const

    const double w = quat[0];
    const double x = quat[1];
    const double y = quat[2];
    const double z = quat[3];

    const double vx = vel[0];
    const double vy = vel[1];
    const double vz = vel[2];

    // compute the tmp variables
    const double tmp1 = two * w * w - 1.0;
    const double tmp2 = two * w;
    const double tmp3 = two * (x * vx + y * vy + z * vz);

    // compute the projected velocity
    return { vx * tmp1 - (y * vz - z * vy) * tmp2 + x * tmp3, vy * tmp1 - (z * vx - x * vz) * tmp2 + y * tmp3,
             vz * tmp1 - (x * vy - y * vx) * tmp2 + z * tmp3 };
}

std_msgs::Float64MultiArray HumanoidRLController::BodyJointCommandWraper(std::vector<double> &pos_des,
                                                                         std::vector<double> &vel_des,
                                                                         std::vector<double> &kp,
                                                                         std::vector<double> &kd,
                                                                         std::vector<double> &torque)
{
    std_msgs::Float64MultiArray msg;
    // set layout
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "parameters";
    msg.layout.dim[0].size = 5; // 5 种参数：pos_des, vel_des, kp, kd, torque
    msg.layout.dim[0].stride = control_config_.robot_config.total_joints_num * 5;
    msg.layout.dim[1].label = "joints";
    msg.layout.dim[1].size = control_config_.robot_config.total_joints_num;
    msg.layout.dim[1].stride = 1;
    msg.layout.data_offset = 0;

    // fill in data
    msg.data.insert(msg.data.end(), pos_des.begin(), pos_des.end());
    msg.data.insert(msg.data.end(), vel_des.begin(), vel_des.end());
    msg.data.insert(msg.data.end(), kp.begin(), kp.end());
    msg.data.insert(msg.data.end(), kd.begin(), kd.end());
    msg.data.insert(msg.data.end(), torque.begin(), torque.end());

    // wraping msg
    return msg;
}

bool HumanoidRLController::IsReady()
{
    std::shared_lock<std::shared_mutex> lock(state_mutex_);
    if(measured_q_.empty() || measured_v_.empty() || measured_tau_.empty() || joint_name_to_index_.empty()
       || action_name_to_index_.empty() || obs_name_to_index_.empty())
    {
        LOGE("Data is not ready!");
        return false;
    }
    return true;
}

std::string HumanoidRLController::StateToString(ControlState state)
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

bool HumanoidRLController::CheckJointLimit(double scale)
{
    if(measured_q_.size() != 36)
    {
        LOGFMTE("Joint number mismatch: %ld, %d", measured_q_.size(), 36);
        return false;
    }

    for(int i = 0; i < control_config_.robot_config.total_joints_num; ++i)
    {
        double lower_limit = GetJointLowerLimit(i) * scale;
        double upper_limit = GetJointUpperLimit(i) * scale;
        // double torque_limit = GetJointTorqueLimit(i) * scale;
        if(measured_q_[i + MEANLESS_SIZE] < lower_limit || measured_q_[i + MEANLESS_SIZE] > upper_limit)
        {
            LOGFMTE("Joint[%d] pos out of %f * limits: %f, [%f, %f]", i, scale, measured_q_[i], lower_limit,
                    upper_limit);
            return false;
        }
        // if(measured_tau_[i + MEANLESS_SIZE] < -torque_limit || measured_tau_[i + MEANLESS_SIZE] > torque_limit)
        // {
        //     LOGFMTE("Joint[%d] torque out of %f * limits: %f, [%f, %f]", i, scale, measured_tau_[i],
        //     -torque_limit, torque_limit); return false;
        // }
    }
    // LOGA("++++++++++++++All joints within limits!++++++++++++++++");
    return true;
}

void HumanoidRLController::Update()
{
    // LOGD("Update function called!");
    UpdateStateEstimation();
    // LOGFMTW("Current state: %s", StateToString(current_state_).c_str());
    switch(current_state_)
    {
    case ControlState::DAMPING:
        HandleDampingMode();
        break;
    case ControlState::ZERO:
        HandleZeroMode();
        break;
    case ControlState::STAND:
        HandleStandMode();
        break;
    case ControlState::WALK:
        HandleWalkMode();
        break;
    default:
        break;
    }
}

void HumanoidRLController::UpdateStateEstimation()
{
    // update propri_ joint pos and vel angular vel quat_est_
    // LOGD("UpdateStateEstimation function called! Update 1: propri_ 2: joint_pos and 3: vel 4: angular_vel 5:
    // quat_est_");
    {
        std::shared_lock<std::shared_mutex> lock(state_mutex_);
        // compute the projected velocity
        propri_.joint_pos.resize(control_config_.inference_config.action_dim);
        propri_.joint_vel.resize(control_config_.inference_config.action_dim);
        for(int i = 0; i < control_config_.inference_config.action_dim; ++i)
        {
            std::string joint_name = control_config_.ordered_action_names[i];
            propri_.joint_pos(i) = measured_q_[joint_name_to_index_[joint_name] + MEANLESS_SIZE];
            propri_.joint_vel(i) = measured_v_[joint_name_to_index_[joint_name] + MEANLESS_SIZE];
        }
        vector3_t angular_vel;
        angular_vel(0) = angular_vel_[0];
        angular_vel(1) = -angular_vel_[1];
        angular_vel(2) = -angular_vel_[2];
        propri_.base_ang_vel = angular_vel;
        vector3_t gravity_vector(0.0, 0.0, -0.981);
        // EulerAngle euler = QuaternionToEuler(Quaternion{quat_est_[0], quat_est_[1], quat_est_[2], quat_est_[3]});
        quaternion_t quat;
        // LOGFMTW("Euler angle: roll: %f, pitch: %f, yaw: %f", euler.roll, euler.pitch, euler.yaw);
        quat.x() = quat_est_[1];
        quat.y() = quat_est_[2];
        quat.z() = quat_est_[3];
        quat.w() = quat_est_[0];
        matrix_t inverse_rot = GetRotationMatrixFromZyxEulerAngles(QuatToZyx(quat)).inverse();
        propri_.projected_gravity = inverse_rot * gravity_vector;
        propri_.projected_gravity(0) *= -1.0;
        // LOGFMTW("propri.projected_gravity: (%f, %f, %f)", propri_.projected_gravity(0),
        // propri_.projected_gravity(1), propri_.projected_gravity(2));
        propri_.base_euler_xyz = QuatToXyz(quat);
        propri_.base_euler_xyz(1) *= -1.0;
        propri_.base_euler_xyz(2) *= -1.0;
        // LOGFMTW("propri_.base_euler_xyz: (%f, %f, %f)", propri_.base_euler_xyz(0), propri_.base_euler_xyz(1),
        // propri_.base_euler_xyz(2));
    }
}

void HumanoidRLController::HandleDampingMode()
{
    // damping mode
    // LOGD("Damping mode");
    pos_des_cmd_ = std::vector<double>(control_config_.robot_config.total_joints_num, 0.0);
    vel_des_cmd_ = std::vector<double>(control_config_.robot_config.total_joints_num, 0.0);
    kp_cmd_ = std::vector<double>(control_config_.robot_config.total_joints_num, 0.0);
    kd_cmd_ = std::vector<double>(control_config_.robot_config.total_joints_num, 0.0);
    torque_cmd_ = std::vector<double>(control_config_.robot_config.total_joints_num, 0.0);
}

void HumanoidRLController::HandleZeroMode()
{
    // zero mode
    // LOGD("Zero mode++++++++++++++++++++++++");
    if(trans_mode_percentage_ <= 1.0)
    {
        for(int j = 0; j < control_config_.robot_config.total_joints_num; ++j)
        {
            int index = joint_name_to_index_[control_config_.ordered_joint_names[j]];
            double pos_des = current_joint_pos_(j) * (1 - trans_mode_percentage_);
            pos_des_cmd_[j] = pos_des;
            vel_des_cmd_[j] = 0.0;
            kp_cmd_[j] = control_config_.joint_conf["stiffness"][control_config_.ordered_joint_names[j]];
            kd_cmd_[j] = control_config_.joint_conf["damping"][control_config_.ordered_joint_names[j]];
            torque_cmd_[j] = 0.0;
            LOGFMTA("Joint name: %s, index: %d, pos_des: %f, kp: %f, kd: %f",
                    control_config_.ordered_joint_names[j].c_str(), index, pos_des, kp_cmd_[j], kd_cmd_[j]);
        }
        trans_mode_percentage_ += 1 / trans_mode_duration_cycle_;
        trans_mode_percentage_ = std::min(trans_mode_percentage_, 1.0);
    }
}

void HumanoidRLController::HandleStandMode()
{
    // stand mode
    // LOGD("Stand mode++++++++++++++++++++++++");
    if(trans_mode_percentage_ <= 1.0)
    {
        for(int j = 0; j < control_config_.robot_config.upper_body_joints_num; ++j)
        {
            double pos_des = current_joint_pos_(j) * (1 - trans_mode_percentage_)
                             + trans_mode_percentage_
                                   * control_config_.joint_conf["init_state"][control_config_.ordered_joint_names[j]];
            pos_des_cmd_[j] = pos_des;
            vel_des_cmd_[j] = 0.0;
            kp_cmd_[j] = 600;
            kd_cmd_[j] = 30;
            torque_cmd_[j] = 0.0;
        }

        for(int i = 0; i < control_config_.robot_config.leg_joints_num; ++i)
        {
            int index = control_config_.robot_config.upper_body_joints_num + i;
            double pos_des
                = current_joint_pos_(index) * (1 - trans_mode_percentage_)
                  + trans_mode_percentage_
                        * control_config_.joint_conf["init_state"][control_config_.ordered_joint_names[index]];
            pos_des_cmd_[index] = pos_des;
            vel_des_cmd_[index] = 0.0;
            kp_cmd_[index] = 300;
            kd_cmd_[index] = 10;
            torque_cmd_[index] = 0.0;
        }
        trans_mode_percentage_ += 1 / trans_mode_duration_cycle_;
        trans_mode_percentage_ = std::min(trans_mode_percentage_, 1.0);
    }
    cycle_count_ = 0;
}

void HumanoidRLController::HandleWalkMode()
{
    // LOGD("Walk mode++++++++++++++++++++++++");
    if(cycle_count_ % control_config_.inference_config.decimation == 0)
    {
        // LOGD("Compute Observations & Actions!");
        ComputeObservation();
        ComputeAction();
    }

    cycle_count_++;

    // walk mode

    for(int j = 0; j < control_config_.robot_config.total_joints_num; j++)
    {
        // int index = joint_name_to_index_[control_config_.ordered_joint_names[j]];
        double pos_des = control_config_.joint_conf["init_state"][control_config_.ordered_joint_names[j]];
        pos_des_cmd_[j] = pos_des;
        vel_des_cmd_[j] = 0.0;
        kp_cmd_[j] = control_config_.joint_conf["stiffness"][control_config_.ordered_joint_names[j]];
        kd_cmd_[j] = control_config_.joint_conf["damping"][control_config_.ordered_joint_names[j]];
        torque_cmd_[j] = 0.0;
    }

    if(control_config_.inference_config.use_lpf)
    {
        // LOGD("Use LPF!");
        for(int i = 0; i < control_config_.robot_config.leg_joints_num; i++)
        {
            if(i == 4 || i == 5 || i == 10 || i == 11)
            {
                double tau_des = kp_cmd_[control_config_.robot_config.upper_body_joints_num + i]
                                 * (actions_[i] * control_config_.inference_config.action_scale - propri_.joint_pos[i]
                                    + kd_cmd_[control_config_.robot_config.upper_body_joints_num + i]
                                          * (0 - propri_.joint_vel[i]));
                lpf_filters_[i].input(tau_des);
                double tau_des_lp = lpf_filters_[i].output();

                pos_des_cmd_[control_config_.robot_config.upper_body_joints_num + i] = 0.0;

                vel_des_cmd_[control_config_.robot_config.upper_body_joints_num + i] = 0.0;

                kp_cmd_[control_config_.robot_config.upper_body_joints_num + i] = 0.0;

                kd_cmd_[control_config_.robot_config.upper_body_joints_num + i] = 2.0;

                torque_cmd_[control_config_.robot_config.upper_body_joints_num + i] = tau_des_lp;
            } else
            {
                lpf_filters_[i].input(actions_[i] * control_config_.inference_config.action_scale);

                double pos_des_lp = lpf_filters_[i].output();

                pos_des_cmd_[control_config_.robot_config.upper_body_joints_num + i] = pos_des_lp;

                vel_des_cmd_[control_config_.robot_config.upper_body_joints_num + i] = 0.0;

                kp_cmd_[control_config_.robot_config.upper_body_joints_num + i]
                    = control_config_.joint_conf["stiffness"][control_config_.ordered_action_names[i]];

                kd_cmd_[control_config_.robot_config.upper_body_joints_num + i]
                    = control_config_.joint_conf["damping"][control_config_.ordered_action_names[i]];

                torque_cmd_[control_config_.robot_config.upper_body_joints_num + i] = 0.0;
            }
            last_actions_(i, 0) = actions_[i];
        }
    } else
    {
        for(int i = 0; i < control_config_.robot_config.leg_joints_num; i++)
        {
            pos_des_cmd_[control_config_.robot_config.upper_body_joints_num + i]
                = actions_[i] * control_config_.inference_config.action_scale;
            vel_des_cmd_[control_config_.robot_config.upper_body_joints_num + i] = 0.0;
            kp_cmd_[control_config_.robot_config.upper_body_joints_num + i]
                = control_config_.joint_conf["stiffness"][control_config_.ordered_action_names[i]];
            kd_cmd_[control_config_.robot_config.upper_body_joints_num + i]
                = control_config_.joint_conf["damping"][control_config_.ordered_action_names[i]];
            torque_cmd_[control_config_.robot_config.upper_body_joints_num + i] = 0.0;
            last_actions_(i, 0) = actions_[i];
        }
    }
}

void HumanoidRLController::ComputeObservation()
{
    double phase = cycle_count_ * (1.0 / 100) / control_config_.inference_config.cycle_time;

    if(control_config_.inference_config.sw_mode)
    {
        double cmd_norm
            = std::sqrt(Square(cmd_data_.linear.x) + Square(cmd_data_.linear.y) + Square(cmd_data_.angular.z));
        if(cmd_norm <= control_config_.inference_config.cmd_threshold)
        {
            cycle_count_ = 0;
            phase = 0.0;
        }
    }

    ControlConfig::ObsConfig obs_config = control_config_.obs_config;
    vector_t propri_obs(control_config_.inference_config.obs_dim);
    vector_t propri_obs_first(control_config_.inference_config.obs_dim);
    propri_obs_first.setZero();
    propri_obs << sin(2 * M_PI * phase), cos(2 * M_PI * phase), cmd_data_.linear.x * obs_config.lin_vel,
        cmd_data_.linear.y * obs_config.lin_vel, cmd_data_.angular.z * obs_config.ang_vel,
        propri_.joint_pos * obs_config.dof_pos, propri_.joint_vel * obs_config.dof_vel, last_actions_,
        propri_.base_ang_vel, propri_.base_euler_xyz * obs_config.quat;
    // LOGFMTA("Phase: %f", phase);
    LOGFMTA("Observation's command: (%f, %f, %f)", cmd_data_.linear.x, cmd_data_.linear.y, cmd_data_.angular.z);
    // for(int i = 0; i < propri_obs.size(); ++i)
    // {
    //     LOGFMTA("Observation[%d]: %f", i, propri_obs[i]);
    // }

    if(is_first_observation_)
    {
        if(control_config_.inference_config.use_lpf)
        {
            for(size_t i = 0; i < control_config_.ordered_action_names.size(); ++i)
            {
                if(i == 4 || i == 5 || i == 10 || i == 11)
                {
                    lpf_filters_[i].init(0);
                } else
                {
                    lpf_filters_[i].init(propri_.joint_pos[i]);
                }
            }
        }

        // set last action to zero
        for(int i = 29; i < 41; ++i)
        {
            propri_obs(i, 0) = 0.0;
        }
        for(int i = 0; i < control_config_.inference_config.obs_history_length; ++i)
        {
            propri_history_buffer_.segment(i * control_config_.inference_config.obs_dim,
                                           control_config_.inference_config.obs_dim)
                = propri_obs_first.cast<float>();
        }
        is_first_observation_ = false;
    }

    propri_history_buffer_.head(propri_history_buffer_.size() - control_config_.inference_config.obs_dim)
        = propri_history_buffer_.tail(propri_history_buffer_.size() - control_config_.inference_config.obs_dim);
    propri_history_buffer_.tail(control_config_.inference_config.obs_dim) = propri_obs.cast<float>();

    for(int i = 0; i < (control_config_.inference_config.obs_dim * control_config_.inference_config.obs_history_length);
        ++i)
    {
        observations_[i] = static_cast<float>(propri_history_buffer_[i]);
    }

    float obs_min = -100.0;
    float obs_max = 100.0;
    std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                   [obs_min, obs_max](float x) { return std::max(obs_min, std::min(obs_max, x)); });
}

void HumanoidRLController::ComputeAction()
{
    // create input tensor
    int64_t obs_dim = static_cast<int64_t>(control_config_.inference_config.obs_dim);
    int64_t history_length = static_cast<int64_t>(control_config_.inference_config.obs_history_length);

    // observations_
    if(observations_.size() != static_cast<size_t>(obs_dim * history_length))
    {
        LOGFMTE("Observation history buffer size mismatch: %ld vs %ld", observations_.size(), obs_dim * history_length);
        return;
    }

    // choose model type by model_path
    if(control_config_.inference_config.inference_type == InferenceType::LIBTORCH)
    {
        // use libtorch to inference
        try
        {
            // copy observations to input tensor
            std::memcpy(torch_input_tensor_.data_ptr<float>(), observations_.data(),
                        obs_dim * history_length * sizeof(float));

            // model inference
            torch_action_tensor_ = model_.forward({ torch_input_tensor_ }).toTensor();
            if(torch_action_tensor_.numel() != control_config_.inference_config.action_dim)
            {
                LOGFMTE("Unexpected model output size: %ld vs %d", torch_action_tensor_.numel(),
                        control_config_.inference_config.action_dim);
                return;
            }

            // copy output tensor to actions_
            std::memcpy(actions_.data(), torch_action_tensor_.data_ptr<float>(),
                        control_config_.inference_config.action_dim * sizeof(float));
        } catch(const c10::Error &e)
        {
            LOGFMTE("Libtorch model inference failed: %s", e.what());
            return;
        } catch(const std::exception &e)
        {
            LOGFMTE("Failed to process libtorch inference: %s", e.what());
            return;
        }
    } else if(control_config_.inference_config.inference_type == InferenceType::ONNX)
    {
        // use ONNX to inference
        try
        {
            // create input tensor

            onnx_input_tensor_.clear();
            onnx_output_tensor_.clear();

            onnx_input_tensor_.push_back(Ort::Value::CreateTensor<float>(memory_info_, observations_.data(),
                                                                         observations_.size(), input_shapes_[0].data(),
                                                                         input_shapes_[0].size()));

            // run inference
            onnx_output_tensor_ = session_ptr_->Run(Ort::RunOptions{}, input_names_.data(), onnx_input_tensor_.data(),
                                                    1, output_names_.data(), 1);

            for(int i = 0; i < control_config_.inference_config.action_dim; ++i)
            {
                actions_[i] = *(onnx_output_tensor_[0].GetTensorMutableData<float>() + i);
            }

        } catch(const Ort::Exception &e)
        {
            LOGFMTE("ONNX model inference failed: %s", e.what());
            return;
        } catch(const std::exception &e)
        {
            LOGFMTE("Failed to process ONNX inference: %s", e.what());
            return;
        }
    } else
    {
        LOGFMTE("Unsupported model type: %s", control_config_.inference_config.model_path.c_str());
        return;
    }

    // clip actions
    scalar_t action_min = control_config_.inference_config.min_action_clip;
    scalar_t action_max = control_config_.inference_config.max_action_clip;
    std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                   [action_min, action_max](scalar_t x) { return std::max(action_min, std::min(action_max, x)); });

    // log info
    LOGFMTW("Model output action size: %ld", actions_.size());
    for(int i = 0; i < control_config_.inference_config.action_dim; ++i)
    {
        LOGFMTW("Final joint_pos[%d]: %f", i, actions_[i] * control_config_.inference_config.action_scale);
    }

    // ComputeTorque();
}

void HumanoidRLController::ComputeTorque()
{
    std::vector<float> torque(control_config_.inference_config.action_dim, 0.0);
    for(int i = 0; i < control_config_.inference_config.action_dim; i++)
    {
        double kp = control_config_.joint_conf
                        ["stiffness"]
                        [control_config_.ordered_joint_names[control_config_.robot_config.upper_body_joints_num + i]];
        double kd = control_config_.joint_conf["damping"][control_config_.ordered_joint_names
                                                              [control_config_.robot_config.upper_body_joints_num + i]];
        double tau_des = kp * (actions_[i] * control_config_.inference_config.action_scale - propri_.joint_pos[i])
                         + kd * (0 - propri_.joint_vel[i]);
        LOGFMTD("Kp[%d]: %f, Kd[%d]: %f", i, kp, i, kd);
        LOGFMTD("Torque[%d]: %f", i, tau_des);
        torque[i] = tau_des;
    }
}