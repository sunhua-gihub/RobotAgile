/*!
    \file lancetrobotstatemachine.h
    \brief 机械臂状态机声明。

    LancetRobotStateMachine 是一个纯 C++ 类（非 QObject），由 LancetRobotArm 内部持有。
    它负责维护当前状态、验证状态转换合法性以及执行转换。

    \note 本类自身不持有锁；调用方（LancetRobotArm）需在转换前获取互斥锁。
*/

#ifndef LANCETROBOTSTATEMACHINE_H
#define LANCETROBOTSTATEMACHINE_H

#include "lancetrobottypes.h"

/*!
    \class LancetRobotStateMachine
    \brief 机械臂状态机，管理 RobotState 之间的合法转换。

    状态转换由 Event 驱动。调用 transition() 前可先用 canTransition()
    查询是否允许，也可直接调用 transition() 并检查返回值。

    \sa RobotState
*/
class LancetRobotStateMachine
{
public:
    /*!
        \enum Event
        \brief 驱动状态转换的事件。
    */
    enum class Event {
        Connected,              ///< 连接建立
        Disconnected,           ///< 连接断开
        MoveIssued,             ///< 点到点 / 路径运动指令发出
        SpeedIssued,            ///< 速度控制指令发出
        ServoIssued,            ///< 伺服控制指令发出
        FreeDrivingEnabled,     ///< 进入拖动示教模式
        FreeDrivingDisabled,    ///< 退出拖动示教模式
        ForceModeEntered,       ///< 进入力控模式
        ForceModeLeft,          ///< 退出力控模式
        ImpedanceModeEntered,   ///< 进入阻抗模式
        ImpedanceModeLeft,      ///< 退出阻抗模式
        MotionCompleted,        ///< 运动完成（速度归零检测）
        Paused,                 ///< 暂停运动
        Resumed,                ///< 恢复运动
        Stopped,                ///< 停止指令
        ErrorOccurred,          ///< 错误回调触发
        ErrorCleared,           ///< 错误清除
        RescueModeEntered,      ///< 进入救援模式
        RescueModeLeft          ///< 退出救援模式
    };

    /*!
        \brief 构造状态机，初始状态为 \a initial。
    */
    explicit LancetRobotStateMachine(RobotState initial = RobotState::Disconnected);

    /*!
        \brief 获取当前状态。
    */
    RobotState currentState() const;

    /*!
        \brief 查询 \a event 在当前状态下是否允许转换。
    */
    bool canTransition(Event event) const;

    /*!
        \brief 执行状态转换。

        \param event 触发事件
        \param oldState 如果非 nullptr，写入转换前的旧状态
        \return 转换后的新状态；如果转换不合法则返回当前状态不变。
    */
    RobotState transition(Event event, RobotState *oldState = nullptr);

    /*!
        \brief 强制设置状态（仅用于 emergencyStop 等绕过正常转换的场景）。
    */
    void forceState(RobotState state);

    // ---- 便捷查询 ----

    bool isConnected() const;
    bool isIdle() const;
    bool isMoving() const;
    bool inError() const;

    /*!
        \brief 检查当前状态是否允许发出新的运动指令。

        仅 Idle 状态返回 true。SpeedControl / ServoControl 状态下
        允许同类型更新，但不被视为"可发新运动"。
    */
    bool canAcceptMotion() const;

    /*!
        \brief 返回运动冲突时应使用的错误码。
    */
    int motionConflictError() const;

    // ---- 调试 ----

    static QString stateName(RobotState state);
    static QString eventName(Event event);

private:
    RobotState resolveTransition(RobotState current, Event event) const;
    RobotState m_state;
};

#endif // LANCETROBOTSTATEMACHINE_H
