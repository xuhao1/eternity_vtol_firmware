//
// Created by xuhao on 2016/4/26.
//

#ifndef ETERNITY_FC_STATE_MACHINE_DEFINES_H
#define ETERNITY_FC_STATE_MACHINE_DEFINES_H

enum controller_mode {
    nothing = 0,
    disarm = 1,
    attitude = 2,
    manual = 3,
    debug_possess_control = 4,
    hover_attitude = 5,
    mode_end
};

enum mode_action {
    donothing = 0,
    try_arm = 1,
    try_disarm = 2,
    toManual = 3,
    toHoverAttitude = 4,
    toPossess = 5,
    toAttitude = 6,
    action_end
};

enum hover_modes
{
    hover_angular_yaw = 0,
    hover_absolute_yaw = 1,
    hover_ioc = 2
};

enum engine_modes
{
    engine_straight_forward = 0,
    engine_control_speed = 1,
    engine_lock = 3
};

#endif //ETERNITY_FC_STATE_MACHINE_DEFINES_H
