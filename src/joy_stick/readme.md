# joy_stick ros1 package for reading joystick inputs


Data Structure
--------------
- JoyStickImpl.joy_->joy_msg_:
  
JoyStruct
  --std::vector<double> axis; // 8 axis
  --std::vector<int32_t> buttons; // 11 buttons

axis & buttons sheet:
  --axis[0]: left stick left/right                | 1.0 / -1.0
  --axis[1]: left stick up/down                   | 1.0 / -1.0
  --axis[2]: left trigger press/unpress           | -1.0 / 1.0
  --axis[3]: right stick left/right               | 1.0 / -1.0
  --axis[4]: right stick up/down                  | 1.0 / -1.0
  --axis[5]: right trigger press/unpress          | -1.0 / 1.0
  --axis[6]: cross key left/right                 | 1.0 / -1.0
  --axis[7]: cross key up/down                    | 1.0 / -1.0

  --buttons[0]: Button A press/unpress            | 1.0 / 0.0
  --buttons[1]: Button B press/unpress            | 1.0 / 0.0
  --buttons[2]: Button X press/unpress            | 1.0 / 0.0
  --buttons[3]: Button y press/unpress            | 1.0 / 0.0
  --buttons[4]: Left Button press/unpress         | 1.0 / 0.0
  --buttons[5]: Right Button press/unpress        | 1.0 / 0.0
  --buttons[6]: Back Button press/unpress         | 1.0 / 0.0
  --buttons[7]: Start Button press/unpress        | 1.0 / 0.0
  --buttons[8]: PS Button press/unpress           | 1.0 / 0.0
  --buttons[9]: Left Stick Button press/unpress   | 1.0 / 0.0
  --buttons[10]: Right Stick Button press/unpress | 1.0 / 0.0