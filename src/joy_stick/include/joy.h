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

#ifndef JOY_H
#define JOY_H

#include <SDL2/SDL.h>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono;

struct JoyStruct {
    std::vector<double> axis;
    std::vector<int32_t> buttons;
};

class Joy {
public:
    explicit Joy();
    Joy(Joy &&c) = delete;
    Joy &operator=(Joy &&c) = delete;
    Joy(const Joy &c) = delete;
    Joy &operator=(const Joy &c) = delete;

    ~Joy();
    void GetJoyData(JoyStruct &joy_data);
    void ResetJoyData();

private:
    void eventThread();
    bool handleJoyAxis(const SDL_Event &e);
    bool handleJoyButtonDown(const SDL_Event &e);    // 按键下
    bool handleJoyButtonUp(const SDL_Event &e);      // 案件抬起
    bool handleJoyHatMotion(const SDL_Event &e);     // 摇杆
    void handleJoyDeviceAdded(const SDL_Event &e);   // 摇杆添加
    void handleJoyDeviceRemoved(const SDL_Event &e); // 摇杆移除
    float convertRawAxisValueToRos(int16_t val);

    int dev_id_{ 0 };

    SDL_Joystick *joystick_{ nullptr };                   // 手柄指针
    SDL_Haptic *haptic_{ nullptr };                       // 手柄震动指针
    int32_t joystick_instance_id_{ 0 };                   // 手柄实例id
    double scaled_deadzone_{ 0.0 };                       // 死区
    double unscaled_deadzone_{ 0.0 };                     // 未缩放的死区
    double scale_{ 0.0 };                                 // 缩放
    double autorepeat_rate_{ 0.0 };                       // 自动重复率
    int autorepeat_interval_ms_{ 0 };                     // 自动重复间隔
    bool sticky_buttons_{ false };                        // 粘性按钮
    bool publish_soon_{ false };                          // 发布
    time_point<high_resolution_clock> publish_soon_time_; // 发布时间
    int coalesce_interval_ms_{ 0 };                       // 合并间隔
    std::string dev_name_;                                // 设备名称
    std::thread event_thread_;                            // 事件线程
    std::atomic_bool is_runing_{ true };                  // 运行flag

    std::mutex joy_msg_mutex_;            // 互斥锁
    std::atomic_bool is_update_{ false }; // 更新flag
    JoyStruct joy_msg_;                   // 手柄数据
};

#endif // JOY_H