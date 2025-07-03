#include "BaseControl/Motor/Motor.hpp"
#include "Utils/Status.hpp"

Motor::Motor(Connectivity &connectivity, uint16_t send_id, uint16_t receive_id,
             uint8_t option, MotorOptionData optionData)
    : send_id(send_id)
    , receive_id(receive_id)
    , option(option)
    , optionData(optionData)
    , connectivity(connectivity)
{
}

bool Motor::ifInitialed()
{
    return state.status == STATUS_INITUALIZED;
}

Motor::MotorState &Motor::getState(void)
{
    return state;
}

Motor::MotorState &Motor::getTargetState(void)
{
    return targetState;
}

classicController *&Motor::getAngleLoopController()
{
    return this->angleLoop;
}
classicController *&Motor::getSpeedLoopController()
{
    return this->speedLoop;
}
classicController *&Motor::getCurrentLoopController()
{
    return this->currentLoop;
}

uint16_t Motor::getSendId()
{
    return this->send_id;
}

uint16_t Motor::getReceiveId()
{
    return this->receive_id;
}

float Motor::calculateControlData()
{
    if (state.status == STATUS_INITUALIZING && (option & MOTOR_SOFT_ZERO)) {
        // 如果正在初始化，并且有软零点选项，以恒定力矩倒转
        return -3.0;
    } else if (!ifInitialed()) {
        return 0.0f; // 如果未初始化，返回 0
    }

    // 限定位置范围
    if (option & MOTOR_SOFT_LIMIT) {
        getTargetState().position =
            getTargetState().position < optionData.soft_limit_min ?
                optionData.soft_limit_min :
            getTargetState().position > optionData.soft_limit_max ?
                optionData.soft_limit_max :
                getTargetState().position;
    }

    // 位置过零点处理
    if (getTargetState().position - getState().position > 180.0f)
        getTargetState().position -= 360.0f;
    else if (getTargetState().position - getState().position < -180.0f)
        getTargetState().position += 360.0f;
    // 计算控制量
    refState.position = getTargetState().position;
    refState.velocity = getTargetState().velocity;
    refState.toreque = getTargetState().toreque;
    if (angleLoop != nullptr) {
        refState.velocity +=
            angleLoop->calculate(refState.position, state.position);
    }
    if (speedLoop != nullptr) {
        refState.toreque +=
            speedLoop->calculate(refState.velocity, state.velocity);
    }

    return refState.toreque;
}