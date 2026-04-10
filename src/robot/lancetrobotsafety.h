/*!
    \file lancetrobotsafety.h
    \brief 医疗级安全防护层声明。

    LancetRobotSafety 提供两类安全保障：
    1. 指令预校验——在发出运动指令前验证速度、加速度、目标位置是否在安全限制内。
    2. 运行时监控——在每帧状态回调中检查力、力矩、工作空间边界。

    安全层由 LancetRobotArm 内部持有，调用方不直接操作本类。
*/

#ifndef LANCETROBOTSAFETY_H
#define LANCETROBOTSAFETY_H

#include <QLoggingCategory>
#include <QObject>

#include "lancetrobottypes.h"

Q_DECLARE_LOGGING_CATEGORY(lcRobotSafety)

/*!
    \class LancetRobotSafety
    \brief 医疗安全防护层。

    内部持有两份安全配置：
    - 外部单位版本（用于 config() 返回给调用方）
    - 内部单位版本（m / rad，用于实际校验，零额外开销）

    \warning 本类的校验方法均在内部单位（m / rad）下工作。
             调用方（LancetRobotArm）负责在调用前将参数转为内部单位。
*/
class LancetRobotSafety : public QObject
{
    Q_OBJECT

public:
    explicit LancetRobotSafety(QObject *parent = nullptr);

    /*!
        \brief 设置安全配置（外部单位）。

        \param cfg      外部单位下的安全配置
        \param unitCfg  当前单位配置，用于转换为内部单位缓存
    */
    void setConfig(const LancetSafetyConfig &cfg, const LancetUnitConfig &unitCfg);

    /*!
        \brief 获取安全配置（外部单位版本）。
    */
    LancetSafetyConfig config() const;

    // ================================================================
    // 指令预校验（内部单位）
    // ================================================================

    /*!
        \brief 校验关节运动参数。

        \param velocity  关节速度（rad/s）
        \param acceleration  关节加速度（rad/s²）
        \param[out] reason  失败原因
        \return true 为安全
    */
    bool validateJointMotion(double velocity, double acceleration,
                             QString *reason = nullptr) const;

    /*!
        \brief 校验笛卡尔平移运动参数。

        \param velocity  平移速度（m/s）
        \param acceleration  平移加速度（m/s²）
        \param[out] reason  失败原因
        \return true 为安全
    */
    bool validateCartMotion(double velocity, double acceleration,
                            QString *reason = nullptr) const;

    /*!
        \brief 校验目标位姿是否在工作空间内。

        \param pose  目标位姿（m / rad，内部单位）
        \param[out] reason  失败原因
        \return true 为安全；enableWorkspace == false 时始终返回 true
    */
    bool validatePoseInWorkspace(const LancetPose &pose,
                                 QString *reason = nullptr) const;

    // ================================================================
    // 运行时监控（内部单位）
    // ================================================================

    /*!
        \brief 检查一帧运行时状态是否安全。

        由 LancetRobotArm 的状态回调在每帧调用。
        如果检测到安全违规且需要急停，会通过 violationDetected 信号通知。

        \param status  当前状态快照（内部单位）
        \param[out] reason  违规原因
        \return true 为安全，false 表示需要急停
    */
    bool checkRuntimeStatus(const LancetRobotStatus &status,
                            QString *reason = nullptr);

Q_SIGNALS:
    /*!
        \brief 安全违规检测信号。

        \param reason  违规描述
        \param requireEmergencyStop  是否需要触发紧急停止
    */
    void violationDetected(const QString &reason, bool requireEmergencyStop);

private:
    LancetSafetyConfig m_externalConfig;  ///< 外部单位版本（原样保存）

    // 内部单位缓存（m / rad），运行时校验直接使用
    double m_maxJointVelRad     = 1.0;
    double m_maxCartVelMs       = 0.2;
    double m_maxCartRotVelRad   = 0.5;
    double m_maxJointAccRad     = 2.0;
    double m_maxCartAccMs       = 1.0;
    double m_maxTcpForceN       = 50.0;
    LancetJoints m_maxJointTorqueNm;

    bool       m_enableWorkspace = false;
    LancetPose m_workspaceMinM;   ///< 内部单位（m / rad）
    LancetPose m_workspaceMaxM;   ///< 内部单位（m / rad）
};

#endif // LANCETROBOTSAFETY_H
