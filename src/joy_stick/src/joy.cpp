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

#include <algorithm>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

#include "joy.h"
#include "log4z.h"

Joy::Joy()
{
    dev_id_ = 0;
    dev_name_ = "";
    scaled_deadzone_ = 0.05;
    unscaled_deadzone_ = 32767.0 * 0.05;
    // According to the SDL docs, this always returns a value between -32768 and 32767. However we want to report a
    // value between -1. and 1., so we scale it by devide by 32767. also note that SDL returns the axis with "forward"
    // and "left" as negative values, which is opposite ti the ROS convention of "forward" and "left" as positive
    // values. So we invert the axis here as well. Finally, we take into account the amount of deadzone so we truly do
    // get value between -1.0 and 1.0.
    scale_ = static_cast<float>(-1.0 / (1.0 - scaled_deadzone_) / 32767.0);
    autorepeat_rate_ = 20.0;
    if(autorepeat_rate_ < 0.0)
    {
        throw std::runtime_error("Autorepeat rate must be >= 0.0 !");
        LOGE("Autorepeat rate must be >= 0.0 !");
    } else if(autorepeat_rate_ > 1000.0)
    {
        throw std::runtime_error("Autorepeat rate must be <= 1000.0 !");
        LOGE("Autorepeat rate must be <= 1000.0 !");
    } else if(autorepeat_rate_ > 0.0)
    {
        autorepeat_interval_ms_ = static_cast<int>(1000.0 / autorepeat_rate_);
    } else
    {
        // If the autorepeate rate is set to 0, the user dose not want to publish unless an event happens. we still wake
        // up every 200 milliseconds to check if we need to quit.
        autorepeat_interval_ms_ = 200;
    }

    sticky_buttons_ = false;
    coalesce_interval_ms_ = 1;
    if(coalesce_interval_ms_ < 0)
    {
        throw std::runtime_error("Coalesce interval must be >= 0 !");
        LOGE("Coalesce interval must be >= 0 !");
    }
    // Make sure to initialize publish_soon_time_ regardless of whether we are going to use it, which ensures that we
    // are always using the correct time source.
    publish_soon_time_ = high_resolution_clock::now();

    if(SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC) < 0)
    {
        throw std::runtime_error("Failed to initialize SDL: " + std::string(SDL_GetError()));
        LOGFMTE("Failed to initialize SDL: %s", std::string(SDL_GetError()).c_str());
    }

    // In theory we could do this with just a timer, which would simplify the code a bit. But then we couldn't react to
    // "imeediate" events, so we stick whith the thread.

    event_thread_ = std::thread(&Joy::eventThread, this);
}

Joy::~Joy()
{
    is_runing_.store(false);
    if(event_thread_.joinable())
    {
        event_thread_.join();
    }
    if(haptic_ != nullptr)
    {
        SDL_HapticClose(haptic_);
    }
    if(joystick_ != nullptr)
    {
        SDL_JoystickClose(joystick_);
    }
    SDL_Quit();
}

void Joy::GetJoyData(JoyStruct &joy_data)
{
    while(!is_update_.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::lock_guard<std::mutex> lock(joy_msg_mutex_);
    joy_data = joy_msg_;
    is_update_.store(false);
    // LOGD("GetJoyData completed, is_update_ reset to false");
}

void Joy::ResetJoyData()
{
    std::lock_guard<std::mutex> lock(joy_msg_mutex_);
    for(size_t i = 0; i < joy_msg_.axis.size(); i++)
    {
        if(i == 2 || i == 5)
        {
            joy_msg_.axis.at(i) = 1.0;
        } else
        {
            joy_msg_.axis.at(i) = 0.0;
        }
    }
    for(size_t i = 0; i < joy_msg_.buttons.size(); i++)
    {
        joy_msg_.buttons.at(i) = 0;
    }
    is_update_.store(true);
}

float Joy::convertRawAxisValueToRos(int16_t val)
{
    if(val == -32768)
    {
        val = -32767;
    }
    // do all the math in double space
    double double_val = static_cast<double>(val);
    // apply deadzone segmantic
    if(double_val > unscaled_deadzone_)
    {
        double_val -= unscaled_deadzone_;
    } else if(double_val < -unscaled_deadzone_)
    {
        double_val += unscaled_deadzone_;
    } else
    {
        double_val = 0.0;
    }
    return static_cast<float>(double_val * scale_);
}

bool Joy::handleJoyAxis(const SDL_Event &e)
{
    bool publish = false;
    // LOGFMTD("Received axis event: which=%d, axis=%d, value=%d, our joystick_instance_id_=%d",
    //        e.jaxis.which, e.jaxis.axis, e.jaxis.value, joystick_instance_id_);
    if(e.jaxis.which != joystick_instance_id_)
    {
        LOGD("Ignoring axis event from different joystick");
        return publish;
    }
    if(e.jaxis.axis >= joy_msg_.axis.size())
    {
        std::cerr << "Saw axis too large for the device, ignoring" << std::endl;
        LOGE("Saw axis too large for the device, ignoring");
        return publish;
    }
    float last_axis_value = joy_msg_.axis.at(e.jaxis.axis);
    joy_msg_.axis.at(e.jaxis.axis) = convertRawAxisValueToRos(e.jaxis.value);
    if(last_axis_value != joy_msg_.axis.at(e.jaxis.axis))
    {
        if(coalesce_interval_ms_ > 0 && !publish_soon_)
        {
            publish_soon_ = true;
            publish_soon_time_ = high_resolution_clock::now();
        } else
        {
            auto time_since_publish_soon = high_resolution_clock::now() - publish_soon_time_;
            if(time_since_publish_soon >= milliseconds(coalesce_interval_ms_))
            {
                publish = true;
                publish_soon_ = false;
            }
        }
    }
    // Else no changes, no publish
    return publish;
}

bool Joy::handleJoyButtonDown(const SDL_Event &e)
{
    bool publish = false;
    if(e.jbutton.which != joystick_instance_id_)
    {
        return publish;
    }
    if(e.jbutton.button >= joy_msg_.buttons.size())
    {
        std::cerr << "Saw button too large for the device, ignoring" << std::endl;
        LOGE("Saw button too large for the device, ignoring");
        return publish;
    }

    if(sticky_buttons_)
    {
        // For sticky buttons, invert 0 -> 1 or 1 -> 0
        joy_msg_.buttons.at(e.jbutton.button) = 1 - joy_msg_.buttons.at(e.jbutton.button);
    } else
    {
        joy_msg_.buttons.at(e.jbutton.button) = 1;
    }
    publish = true;
    return publish;
}

bool Joy::handleJoyButtonUp(const SDL_Event &e)
{
    bool publish = false;
    if(e.jbutton.which != joystick_instance_id_)
    {
        return publish;
    }
    if(e.jbutton.button >= joy_msg_.buttons.size())
    {
        std::cerr << "Saw button too large for the device, ignoring" << std::endl;
        LOGE("Saw button too large for the device, ignoring");
        return publish;
    }

    if(!sticky_buttons_)
    {
        joy_msg_.buttons.at(e.jbutton.button) = 0;
        publish = true;
    }
    return publish;
}

bool Joy::handleJoyHatMotion(const SDL_Event &e)
{
    bool publish = false;
    // LOGFMTD("Received hat event: which=%d, hat=%d, value=%d, our joystick_instance_id_=%d",
    //        e.jhat.which, e.jhat.hat, e.jhat.value, joystick_instance_id_);
    if(e.jhat.which != joystick_instance_id_)
    {
        return publish;
    }
    // The hats are the last axis in the axis list. There are two axis per hat;
    // the first of the pair is for left (postive) and right (negative), while
    // the second of the pair is for up (positive) and down (negative).

    // Determine which pair we are based on e.jhat.hat
    int num_axes = SDL_JoystickNumAxes(joystick_);
    if(num_axes < 0)
    {
        std::cerr << "Failed to get number of axes: " << SDL_GetError() << std::endl;
        LOGFMTE("Failed to get number of axes: %s", SDL_GetError());
        return publish;
    }
    size_t axes_start_index = num_axes + e.jhat.hat * 2;
    // Note that we check axes_start_index + 1 here to ensure that we can write to
    // either the left/right or up/down axis that corresponds the this hat;
    if((axes_start_index + 1) >= joy_msg_.axis.size())
    {
        std::cerr << "Saw hat too large for the device, ignoring" << std::endl;
        LOGE("Saw hat too large for the device, ignoring");
        return publish;
    }

    if(e.jhat.value & SDL_HAT_LEFT)
    {
        joy_msg_.axis.at(axes_start_index) += 0.2;
    }
    if(e.jhat.value & SDL_HAT_RIGHT)
    {
        joy_msg_.axis.at(axes_start_index) -= 0.2;
    }
    if(e.jhat.value & SDL_HAT_UP)
    {
        joy_msg_.axis.at(axes_start_index + 1) += 0.2;
    }
    if(e.jhat.value & SDL_HAT_DOWN)
    {
        joy_msg_.axis.at(axes_start_index + 1) -= 0.2;
    }
    // if(e.jhat.value == SDL_HAT_CENTERED)
    // {
    //     joy_msg_.axis.at(axes_start_index) = 0.0;
    //     joy_msg_.axis.at(axes_start_index + 1) = 0.0;
    // }
    publish = true;

    return publish;
}

void Joy::handleJoyDeviceAdded(const SDL_Event &e)
{
    if(!dev_name_.empty())
    {
        int num_joysticks = SDL_NumJoysticks();
        if(num_joysticks < 0)
        {
            std::cerr << "Failed to get number of joysticks: " << SDL_GetError() << std::endl;
            LOGFMTE("Failed to get number of joysticks: %s", SDL_GetError());
            return;
        }
        bool matching_device_found = false;
        for(int i = 0; i < num_joysticks; ++i)
        {
            const char *name = SDL_JoystickNameForIndex(i);
            if(name == nullptr)
            {
                std::cerr << "Failed to get name for joystick " << i << ": " << SDL_GetError() << std::endl;
                LOGFMTE("Failed to get name for joystick %d: %s", i, SDL_GetError());
                continue;
            }
            if(std::string(name) == dev_name_)
            {
                // Found it!
                matching_device_found = true;
                dev_id_ = i;
                break;
            }
        }
        if(!matching_device_found)
        {
            std::cerr << "Failed to find device " << dev_name_ << ": " << SDL_GetError() << std::endl;
            LOGFMTE("Failed to find device %s: %s", dev_name_.c_str(), SDL_GetError());
            return;
        }
    }

    if(e.jdevice.which != dev_id_)
    {
        LOGFMTE("Device added but not our target device (got %d, want %d)", e.jdevice.which, dev_id_);
        return;
    }

    joystick_ = SDL_JoystickOpen(dev_id_);
    if(joystick_ == nullptr)
    {
        std::cerr << "Unable to open joystick " << dev_id_ << ": " << SDL_GetError() << std::endl;
        LOGFMTE("Unable to open joystick %d: %s", dev_id_, SDL_GetError());
        return;
    }

    // We need to hold onto this so we can properly remove it on a remove event
    joystick_instance_id_ = SDL_JoystickGetDeviceInstanceID(dev_id_);
    if(joystick_instance_id_ < 0)
    {
        std::cerr << "Failed to get instance id for joystick " << dev_id_ << ": " << SDL_GetError() << std::endl;
        LOGFMTE("Failed to get instance id for joystick %d: %s", dev_id_, SDL_GetError());
        SDL_JoystickClose(joystick_);
        joystick_ = nullptr;
        return;
    }

    int num_bottons = SDL_JoystickNumButtons(joystick_);
    if(num_bottons < 0)
    {
        std::cerr << "Failed to get number of buttons: " << SDL_GetError() << std::endl;
        LOGFMTE("Failed to get number of buttons: %s", SDL_GetError());
        SDL_JoystickClose(joystick_);
        joystick_ = nullptr;
        return;
    }
    joy_msg_.buttons.resize(num_bottons);

    int num_axes = SDL_JoystickNumAxes(joystick_);
    if(num_axes < 0)
    {
        std::cerr << "Failed to get number of axes: " << SDL_GetError() << std::endl;
        LOGFMTE("Failed to get number of axes: %s", SDL_GetError());
        SDL_JoystickClose(joystick_);
        joystick_ = nullptr;
        return;
    }

    int num_hats = SDL_JoystickNumHats(joystick_);
    if(num_hats < 0)
    {
        std::cerr << "Failed to get number of hats: " << SDL_GetError() << std::endl;
        LOGFMTE("Failed to get number of hats: %s", SDL_GetError());
        SDL_JoystickClose(joystick_);
        joystick_ = nullptr;
        return;
    }
    joy_msg_.axis.resize(num_axes + num_hats * 2);

    // Get the initial state for each of the axis
    for(int i = 0; i < num_axes; ++i)
    {
        int16_t state;
        if(SDL_JoystickGetAxisInitialState(joystick_, i, &state))
        {
            joy_msg_.axis.at(i) = convertRawAxisValueToRos(state);
        }
    }

    haptic_ = SDL_HapticOpenFromJoystick(joystick_);
    if(haptic_ != nullptr)
    {
        if(SDL_HapticRumbleInit(haptic_) < 0)
        {
            // Failed to init haptic. Close it and set to nullptr
            LOGFMTE("Failed to initialize rumble: %s", SDL_GetError());
            SDL_HapticClose(haptic_);
            haptic_ = nullptr;
        }
    } else
    {
        LOGI("No haptic(rumble) available, skipping initialization");
    }
    LOGFMTI("Opened joystick: %s. deadzone: %f", SDL_JoystickName(joystick_), scaled_deadzone_);
}

void Joy::handleJoyDeviceRemoved(const SDL_Event &e)
{
    if(e.jdevice.which != dev_id_)
    {
        return;
    }

    joy_msg_.buttons.resize(0);
    joy_msg_.axis.resize(0);
    if(haptic_ != nullptr)
    {
        SDL_HapticClose(haptic_);
        haptic_ = nullptr;
    }
    if(joystick_ != nullptr)
    {
        SDL_JoystickClose(joystick_);
        joystick_ = nullptr;
    }
    LOGFMTI("Joystick removed: %s", dev_name_.c_str());
}

void Joy::eventThread()
{
    LOGI("Starting event thread***********************");
    auto last_publish = high_resolution_clock::now();

    while(is_runing_)
    {
        bool should_publish = false;
        SDL_Event e;
        int wait_time_ms = autorepeat_interval_ms_;
        if(publish_soon_)
        {
            wait_time_ms = std::min(wait_time_ms, coalesce_interval_ms_);
        }
        int success = SDL_WaitEventTimeout(&e, wait_time_ms);
        std::lock_guard<std::mutex> lock(joy_msg_mutex_);
        if(success == 1)
        {
            // LOGFMTD("Received SDL event type: %d", e.type);
            switch(e.type)
            {
            case SDL_JOYAXISMOTION: // 1536
                should_publish = handleJoyAxis(e);
                break;
            case SDL_JOYBUTTONDOWN: // 1539
                should_publish = handleJoyButtonDown(e);
                break;
            case SDL_JOYBUTTONUP: // 1540
                should_publish = handleJoyButtonUp(e);
                break;
            case SDL_JOYHATMOTION: // 1538
                should_publish = handleJoyHatMotion(e);
                break;
            case SDL_JOYDEVICEADDED:
                handleJoyDeviceAdded(e);
                break;
            case SDL_JOYDEVICEREMOVED:
                handleJoyDeviceRemoved(e);
                break;
            default:
                LOGFMTE("Unhandled event type: %d", e.type);
                break;
            }
        } else
        {
            // didn't get an event, either because of a failure or because of a timeout.
            // if we are autorepeating and enough time has passed, we should publish
            auto now = high_resolution_clock::now();
            auto diff_since_last_publish = now - last_publish;
            if((autorepeat_rate_ > 0.0 && diff_since_last_publish >= milliseconds(autorepeat_interval_ms_))
               || publish_soon_)
            {
                last_publish = now;
                should_publish = true;
                publish_soon_ = false;
            }
        }

        if(joystick_ != nullptr && should_publish)
        {
            // LOGFMTD("Setting is_update_ to true, buttons size: %zu, axis size: %zu", joy_msg_.buttons.size(),
            // joy_msg_.axis.size());
            is_update_.store(true);
        }
    }
}