/*!
    \file lancetrobotstatemachine.cpp
    \brief LancetRobotStateMachine 状态转换表实现。
*/

#include "lancetrobotstatemachine.h"

// =====================================================================
// 构造
// =====================================================================

LancetRobotStateMachine::LancetRobotStateMachine(RobotState initial)
    : m_state(initial)
{
}

// =====================================================================
// 核心查询与转换
// =====================================================================

RobotState LancetRobotStateMachine::currentState() const
{
    return m_state;
}

bool LancetRobotStateMachine::canTransition(Event event) const
{
    return resolveTransition(m_state, event) != m_state;
}

RobotState LancetRobotStateMachine::transition(Event event, RobotState *oldState)
{
    RobotState prev = m_state;
    RobotState next = resolveTransition(m_state, event);
    m_state = next;
    if (oldState)
        *oldState = prev;
    return next;
}

void LancetRobotStateMachine::forceState(RobotState state)
{
    m_state = state;
}

// =====================================================================
// 便捷查询
// =====================================================================

bool LancetRobotStateMachine::isConnected() const
{
    return m_state != RobotState::Disconnected;
}

bool LancetRobotStateMachine::isIdle() const
{
    return m_state == RobotState::Idle;
}

bool LancetRobotStateMachine::isMoving() const
{
    return m_state == RobotState::Moving
        || m_state == RobotState::SpeedControl
        || m_state == RobotState::ServoControl;
}

bool LancetRobotStateMachine::inError() const
{
    return m_state == RobotState::Error;
}

bool LancetRobotStateMachine::canAcceptMotion() const
{
    return m_state == RobotState::Idle;
}

int LancetRobotStateMachine::motionConflictError() const
{
    switch (m_state) {
    case RobotState::Disconnected:
        return LancetRobotError::NotConnected;
    case RobotState::FreeDriving:
        return LancetRobotError::FreeDrivingBusy;
    case RobotState::Error:
        return LancetRobotError::FatalError;
    default:
        return LancetRobotError::CallingConflict;
    }
}

// =====================================================================
// 状态转换表
// =====================================================================

RobotState LancetRobotStateMachine::resolveTransition(RobotState current, Event event) const
{
    // ---- 全局转换（优先级最高） ----
    if (event == Event::ErrorOccurred && current != RobotState::Disconnected)
        return RobotState::Error;

    if (event == Event::Disconnected)
        return RobotState::Disconnected;

    // ---- 按当前状态分发 ----
    switch (current) {

    case RobotState::Disconnected:
        if (event == Event::Connected)
            return RobotState::Idle;
        break;

    case RobotState::Idle:
        switch (event) {
        case Event::MoveIssued:           return RobotState::Moving;
        case Event::SpeedIssued:          return RobotState::SpeedControl;
        case Event::ServoIssued:          return RobotState::ServoControl;
        case Event::FreeDrivingEnabled:   return RobotState::FreeDriving;
        case Event::ForceModeEntered:     return RobotState::ForceMode;
        case Event::ImpedanceModeEntered: return RobotState::ImpedanceMode;
        case Event::RescueModeEntered:    return RobotState::RescueMode;
        default: break;
        }
        break;

    case RobotState::Moving:
        switch (event) {
        case Event::MotionCompleted: return RobotState::Idle;
        case Event::Stopped:         return RobotState::Idle;
        case Event::Paused:          return RobotState::Paused;
        default: break;
        }
        break;

    case RobotState::Paused:
        switch (event) {
        case Event::Resumed: return RobotState::Moving;
        case Event::Stopped: return RobotState::Idle;
        default: break;
        }
        break;

    case RobotState::SpeedControl:
        if (event == Event::Stopped)
            return RobotState::Idle;
        break;

    case RobotState::ServoControl:
        if (event == Event::Stopped)
            return RobotState::Idle;
        break;

    case RobotState::FreeDriving:
        if (event == Event::FreeDrivingDisabled)
            return RobotState::Idle;
        break;

    case RobotState::ForceMode:
        if (event == Event::ForceModeLeft)
            return RobotState::Idle;
        break;

    case RobotState::ImpedanceMode:
        if (event == Event::ImpedanceModeLeft)
            return RobotState::Idle;
        break;

    case RobotState::Error:
        if (event == Event::ErrorCleared)
            return RobotState::Idle;
        break;

    case RobotState::RescueMode:
        if (event == Event::RescueModeLeft)
            return RobotState::Idle;
        break;
    }

    // 无合法转换，状态不变
    return current;
}

// =====================================================================
// 调试辅助
// =====================================================================

QString LancetRobotStateMachine::stateName(RobotState state)
{
    switch (state) {
    case RobotState::Disconnected:  return QStringLiteral("Disconnected");
    case RobotState::Idle:          return QStringLiteral("Idle");
    case RobotState::Moving:        return QStringLiteral("Moving");
    case RobotState::SpeedControl:  return QStringLiteral("SpeedControl");
    case RobotState::ServoControl:  return QStringLiteral("ServoControl");
    case RobotState::FreeDriving:   return QStringLiteral("FreeDriving");
    case RobotState::ForceMode:     return QStringLiteral("ForceMode");
    case RobotState::ImpedanceMode: return QStringLiteral("ImpedanceMode");
    case RobotState::Paused:        return QStringLiteral("Paused");
    case RobotState::Error:         return QStringLiteral("Error");
    case RobotState::RescueMode:    return QStringLiteral("RescueMode");
    }
    return QStringLiteral("Unknown");
}

QString LancetRobotStateMachine::eventName(Event event)
{
    switch (event) {
    case Event::Connected:            return QStringLiteral("Connected");
    case Event::Disconnected:         return QStringLiteral("Disconnected");
    case Event::MoveIssued:           return QStringLiteral("MoveIssued");
    case Event::SpeedIssued:          return QStringLiteral("SpeedIssued");
    case Event::ServoIssued:          return QStringLiteral("ServoIssued");
    case Event::FreeDrivingEnabled:   return QStringLiteral("FreeDrivingEnabled");
    case Event::FreeDrivingDisabled:  return QStringLiteral("FreeDrivingDisabled");
    case Event::ForceModeEntered:     return QStringLiteral("ForceModeEntered");
    case Event::ForceModeLeft:        return QStringLiteral("ForceModeLeft");
    case Event::ImpedanceModeEntered: return QStringLiteral("ImpedanceModeEntered");
    case Event::ImpedanceModeLeft:    return QStringLiteral("ImpedanceModeLeft");
    case Event::MotionCompleted:      return QStringLiteral("MotionCompleted");
    case Event::Paused:               return QStringLiteral("Paused");
    case Event::Resumed:              return QStringLiteral("Resumed");
    case Event::Stopped:              return QStringLiteral("Stopped");
    case Event::ErrorOccurred:        return QStringLiteral("ErrorOccurred");
    case Event::ErrorCleared:         return QStringLiteral("ErrorCleared");
    case Event::RescueModeEntered:    return QStringLiteral("RescueModeEntered");
    case Event::RescueModeLeft:       return QStringLiteral("RescueModeLeft");
    }
    return QStringLiteral("Unknown");
}
