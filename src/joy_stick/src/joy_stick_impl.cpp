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

#include "joy_stick_impl.h"
#include "log4z.h"
#include <iostream>
#include <yaml-cpp/yaml.h>

using namespace zsummer::log4z;

JoyStickImpl::JoyStickImpl(ros::NodeHandle &nh) : nh_(nh) { Init(); }

JoyStickImpl::~JoyStickImpl() { LOGW("JoyStickImpl object has been destroyed!"); }

bool JoyStickImpl::Init()
{
    joy_ = std::make_shared<Joy>();
    try
    {
        // read yaml file
        LOGI("Attempting to load YAML config file...");
        const char *homeDir = getenv("HOME");
        std::string config_path = std::string(homeDir) + "/humanoid_rl/src/joy_stick/config/joy_xbot.yaml";
        LOGFMTI("Config path: %s", config_path.c_str());

        // check if file exists
        std::ifstream f(config_path.c_str());
        if(!f.good())
        {
            LOGFMTE("Config file does not exist or cannot be accessed: %s", config_path.c_str());
            return false;
        }

        YAML::Node cfg_node = YAML::LoadFile(config_path);
        LOGD("Loaded yaml file!");
        freq_ = cfg_node["freq"].as<int32_t>();
        LOGD("Loaded freq_");
        command_type_ = cfg_node["command_type"].as<std::string>();
        LOGD("Loaded command_type_");
        // ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ init publishers ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
        if(cfg_node["float_pubs"])
        {
            for(const auto &pub : cfg_node["float_pubs"])
            {
                FloatPub float_pub;
                float_pub.topic_name = pub["topic_name"].as<std::string>();
                float_pub.buttons = pub["buttons"].as<std::vector<int> >();
                float_pub.pub = nh_.advertise<std_msgs::Bool>(float_pub.topic_name, 1);
                float_pubs_.push_back(float_pub);
            }
        }

        if(cfg_node["reset_pubs"])
        {
            for(const auto &pub : cfg_node["reset_pubs"])
            {
                ResetPub reset_pub;
                reset_pub.buttons = pub["buttons"].as<std::vector<int> >();
                reset_pubs_.push_back(reset_pub);
            }
        }

        if(cfg_node["twist_pubs"])
        {
            for(const auto &pub : cfg_node["twist_pubs"])
            {
                TwistPub twist_pub;
                twist_pub.topic_name = pub["topic_name"].as<std::string>();
                twist_pub.buttons = pub["buttons"].as<std::vector<int> >();
                twist_pub.axis = pub["axis"].as<std::map<std::string, int> >();
                if(pub["type"].as<std::string>() == "joy")
                {
                    joy_twist_pubs_.push_back(twist_pub);
                } else if(pub["type"].as<std::string>() == "hat")
                {
                    hat_twist_pubs_.push_back(twist_pub);
                }
            }
        }

        // if(cfg_node["service_clients"])
        // {
        //     for(const auto &client : cfg_node["service_clients"])
        //     {
        //         ServiceClient service_client;
        //         service_client.service_name = client["service_name"].as<std::string>();
        //         service_client.interface_type = client["interface_type"].as<std::string>();
        //         service_client.buttons = client["buttons"].as<std::vector<uint8_t> >();
        //         service_clients_.push_back(service_client);
        //     }
        // }

        if(float_pubs_.size() != 4)
        {
            LOGFMTE("float_pubs size is not 4, %ld", float_pubs_.size());
            return false;
        }

        if(joy_twist_pubs_.size() != 1 || hat_twist_pubs_.size() != 1)
        {
            LOGFMTE("one of joy_twist_pubs_ or hat_twist_pubs_ size is not 1, %ld != 1", joy_twist_pubs_.size());
            return false;
        }

        if(joy_twist_pubs_[0].topic_name != hat_twist_pubs_[0].topic_name)
        {
            LOGFMTE("twist_pubs topic name is not the same, %s != %s", joy_twist_pubs_[0].topic_name.c_str(),
                    hat_twist_pubs_[0].topic_name.c_str());
            return false;
        }

        twist_pub_
            = nh_.advertise<geometry_msgs::Twist>(joy_twist_pubs_[0].topic_name, 1); // twist_pub_ is a ros::Publisher

        // if(service_clients_.size() != 1)
        // {
        //     LOGFMTE("service_clients size is not 1, %ld", service_clients_.size());
        //     return false;
        // }

        LOGI("JoyStickImpl init successed!");
        return true;

    } catch(const std::exception &e)
    {
        LOGFMTE("JoyStickImpl init failed, %s", e.what());
        return false;
    }
}

void JoyStickImpl::Shutdown()
{
    is_running_ = false;
    stop_sig_.get_future().wait();
    LOGW("JoyStickImpl shutdown succeeded!");
}

int main(int argc, char **argv)
{
    ILog4zManager::getRef().start();
    ILog4zManager::getRef().setLoggerLevel(LOG4Z_MAIN_LOGGER_ID, LOG_LEVEL_DEBUG);

    ros::init(argc, argv, "joy_stick_impl");
    ros::NodeHandle nh;
    JoyStickImpl joy_stick_impl(nh);
    ros::spinOnce();
    ros::Rate rate(joy_stick_impl.freq_);

    LOGA("******************Start JoyStickImpl main loop*******************");

    while(ros::ok() && joy_stick_impl.is_running_.load())
    {
        std_msgs::Bool button_msgs;
        geometry_msgs::Twist vel_msgs;
        // geometry_msgs::Twist zero_msgs;

        JoyStruct joy_data;
        joy_stick_impl.joy_->GetJoyData(joy_data);
        // LOGFMTD("Joy data received - buttons: %lu, axis: %lu", joy_data.buttons.size(), joy_data.axis.size());
        for(size_t i = 0; i < joy_data.buttons.size(); i++)
        {
            LOGFMTW("Button %lu: %d", i, joy_data.buttons[i]);
        }
        for(size_t i = 0; i < joy_data.axis.size(); i++)
        {
            LOGFMTW("Axis %lu: %f", i, joy_data.axis[i]);
        }

        for(auto float_pub : joy_stick_impl.float_pubs_)
        {
            bool ret = true;
            for(auto button : float_pub.buttons)
            {
                if(button < static_cast<int>(joy_data.buttons.size()))
                {
                    ret &= joy_data.buttons[button];
                } else
                {
                    // if button index is out of range, return false
                    LOGFMTE("Button index %d out of range (size: %zu)", button, joy_data.buttons.size());
                    ret = false;
                    break; // break the loop
                }
            }
            if(ret)
            {
                LOGD("**************Publishing float message!***************");
                float_pub.pub.publish(button_msgs);
            }
        }

        bool reset_ret = true;
        for(auto reset_pub : joy_stick_impl.reset_pubs_)
        {
            for(auto button : reset_pub.buttons)
            {
                // LOGFMTW("Reset button %d pressed", button);
                if(button < static_cast<int>(joy_data.buttons.size()))
                {
                    reset_ret &= joy_data.buttons[button];
                } else
                {
                    LOGFMTE("Button index %d out of range (size: %zu)", button, joy_data.buttons.size());
                    reset_ret = false;
                    break;
                }
            }
            if(reset_ret)
            {
                joy_stick_impl.joy_->ResetJoyData();
                for(size_t i = 0; i < joy_data.axis.size(); i++)
                {
                    if(i == 2 || i == 5)
                    {
                        joy_data.axis[i] = 1.0;
                    } else
                    {
                        joy_data.axis[i] = 0.0;
                    }
                }
                vel_msgs.linear.x = vel_msgs.linear.y = vel_msgs.linear.z = 0.0;
                vel_msgs.angular.x = vel_msgs.angular.y = vel_msgs.angular.z = 0.0;
                LOGD("**************Publishing zero twist message!***************");
                joy_stick_impl.twist_pub_.publish(vel_msgs);
                rate.sleep();
                ros::spinOnce();
                continue; // break the loop
            }
        }

        if(joy_stick_impl.command_type_ == "joy")
        {
            for(auto twist_pub : joy_stick_impl.joy_twist_pubs_)
            {
                bool ret = true;
                for(auto button : twist_pub.buttons)
                {
                    // check if button index is out of range
                    if(button < static_cast<int>(joy_data.buttons.size()))
                    {
                        ret &= joy_data.buttons[button];
                    } else
                    {
                        // if button index is out of range, return false
                        LOGFMTE("Button index %d out of range (size: %zu)", button, joy_data.buttons.size());
                        ret = false;
                        break; // break the loop
                    }
                }
                if(ret)
                {
                    // lambda function to check and set axis
                    auto checkAndSetAxis = [&joy_data](const std::map<std::string, int> &axisMap,
                                                       const std::string &key, double &value) {
                        auto it = axisMap.find(key);
                        if(it != axisMap.end())
                        {
                            int axisIndex = it->second;
                            // check if axis index is out of range
                            if(axisIndex < static_cast<int>(joy_data.axis.size()))
                            {
                                value = joy_data.axis[axisIndex];
                            } else
                            {
                                LOGFMTE("Axis index %d for %s out of range (size: %zu)", axisIndex, key.c_str(),
                                        joy_data.axis.size());
                            }
                        }
                    };

                    // use lambda function to check and set all axis
                    checkAndSetAxis(twist_pub.axis, "linear_x", vel_msgs.linear.x);
                    checkAndSetAxis(twist_pub.axis, "linear_y", vel_msgs.linear.y);
                    checkAndSetAxis(twist_pub.axis, "linear_z", vel_msgs.linear.z);
                    checkAndSetAxis(twist_pub.axis, "angular_x", vel_msgs.angular.x);
                    checkAndSetAxis(twist_pub.axis, "angular_y", vel_msgs.angular.y);
                    checkAndSetAxis(twist_pub.axis, "angular_z", vel_msgs.angular.z);

                    LOGD("**************Publishing joy vel twist message!***************");
                    joy_stick_impl.twist_pub_.publish(vel_msgs);
                }
            }
        } else if(joy_stick_impl.command_type_ == "hat")
        {
            for(auto twist_pub : joy_stick_impl.hat_twist_pubs_)
            {
                bool ret = true;
                for(auto button : twist_pub.buttons)
                {
                    // check if button index is out of range
                    if(button < static_cast<int>(joy_data.buttons.size()))
                    {
                        ret &= joy_data.buttons[button];
                    } else
                    {
                        // if button index is out of range, return false
                        LOGFMTE("Button index %d out of range (size: %zu)", button, joy_data.buttons.size());
                        ret = false;
                        break; // break the loop
                    }
                }
                if(ret)
                {
                    // lambda function to check and set axis
                    auto checkAndSetAxis = [&joy_data](const std::map<std::string, int> &axisMap,
                                                       const std::string &key, double &value) {
                        auto it = axisMap.find(key);
                        if(it != axisMap.end())
                        {
                            int axisIndex = it->second;
                            // check if axis index is out of range
                            if(axisIndex < static_cast<int>(joy_data.axis.size()))
                            {
                                value = joy_data.axis[axisIndex];
                            } else
                            {
                                LOGFMTE("Axis index %d for %s out of range (size: %zu)", axisIndex, key.c_str(),
                                        joy_data.axis.size());
                            }
                        }
                    };

                    // use lambda function to check and set all axis
                    checkAndSetAxis(twist_pub.axis, "linear_x", vel_msgs.linear.x);
                    checkAndSetAxis(twist_pub.axis, "linear_y", vel_msgs.linear.y);
                    checkAndSetAxis(twist_pub.axis, "linear_z", vel_msgs.linear.z);
                    checkAndSetAxis(twist_pub.axis, "angular_x", vel_msgs.angular.x);
                    checkAndSetAxis(twist_pub.axis, "angular_y", vel_msgs.angular.y);
                    checkAndSetAxis(twist_pub.axis, "angular_z", vel_msgs.angular.z);

                    LOGD("**************Publishing hat vel twist message!***************");
                    joy_stick_impl.twist_pub_.publish(vel_msgs);
                }
            }
        }

        rate.sleep();
        ros::spinOnce();
    }

    LOGA("******************Exit JoyStickImpl main loop*******************");
    LOGA("Ros ShutDown!");
    return 0;
}