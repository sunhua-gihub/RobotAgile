/*!
    \file lancetrobotarm.h
    \brief 思灵工业机械臂 Qt 封装类声明。

    LancetRobotArm 是本封装层的核心类，提供：
    - 状态机驱动的运动指令管理（冲突拒绝策略）
    - 医疗安全防护（指令预校验 + 运行时力/力矩监控）
    - 自动单位转换（外部 mm/度 ↔ 内部 m/rad）
    - 线程安全的状态查询与 Qt 信号通知

    \note 使用 PIMPL 模式，C API（DianaApi.h）依赖完全隐藏在 .cpp 中。
*/

#ifndef LANCETROBOTARM_H
#define LANCETROBOTARM_H

#include <QObject>
#include <QScopedPointer>

#include "lancetrobottypes.h"

class LancetRobotArmPrivate;

/*!
    \class LancetRobotArm
    \brief 思灵工业机械臂的 Qt 封装类（QObject 派生）。

    每个实例对应一台通过 IP 连接的物理机械臂。不可拷贝。
    所有公共方法的位置、速度、加速度参数均使用 unitConfig() 指定的外部单位，
    内部自动转换为机械臂原生单位后调用底层 C API。

    \section1 典型用法
    \code
    LancetRobotArm arm("192.168.1.10");
    arm.connectToRobot();
    arm.waitForIdle();

    LancetMotionParams p;
    p.velocity = 50.0;       // 50 mm/s
    p.acceleration = 300.0;  // 300 mm/s²
    arm.moveLinearToPose({300, 0, 400, 0, 180, 0}, p);
    arm.waitForIdle(10000);
    \endcode

    \sa LancetRobotStateMachine, LancetRobotSafety, LancetRobotPath
*/
class LancetRobotArm : public QObject
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(LancetRobotArm)

public:
    /*!
        \brief 构造机械臂控制实例。
        \param ip          机械臂 IP 地址
        \param jointCount  关节数（Diana = 7，Thor = 6）
        \param parent      Qt 父对象
    */
    explicit LancetRobotArm(const QString &ip, int jointCount = 7,
                            QObject *parent = nullptr);
    ~LancetRobotArm() override;

    // =================================================================
    // 连接管理
    // =================================================================

    /*!
        \brief 建立与机械臂的网络连接（调用 initSrvV2）。
        \return 0 成功，负数错误码
    */
    int connectToRobot();

    /*!
        \brief 断开连接（调用 destroySrv）。
    */
    void disconnectFromRobot();

    /*!
        \brief 是否已建立连接。
    */
    bool isConnected() const;

    /*!
        \brief 设置状态推送周期。
        \param ms 周期（毫秒），默认 10
    */
    int setStatePushPeriod(int ms);

    // =================================================================
    // 单位配置
    // =================================================================

    /*!
        \brief 设置外部单位系统。

        构造时默认 {Millimeter, Degree}。设置后立即生效，
        影响后续所有公共方法的输入/输出以及安全配置的解读。
    */
    void setUnitConfig(const LancetUnitConfig &cfg);

    /*!
        \brief 获取当前外部单位配置。
    */
    LancetUnitConfig unitConfig() const;

    // =================================================================
    // 状态机查询
    // =================================================================

    RobotState state() const;
    bool isIdle() const;
    bool isMoving() const;
    bool inError() const;

    /*!
        \brief 阻塞等待机械臂进入 Idle / Error / Disconnected 状态。

        \param timeoutMs 超时（毫秒），-1 表示永久等待
        \return 0 = Idle，LancetRobotError::Timeout = 超时，负数 = 进入了 Error 态
        \warning 不要在 Qt 主线程（GUI 线程）中使用 -1 超时。
    */
    int waitForIdle(int timeoutMs = -1);

    /*!
        \brief 清除错误状态，尝试恢复到 Idle。
        \return 0 成功
    */
    int clearError();

    // =================================================================
    // 安全配置
    // =================================================================

    /*!
        \brief 设置医疗安全配置。

        限值使用当前 unitConfig() 的外部单位填写。
        \warning 需要机械臂处于 Idle 或 Disconnected 状态。
    */
    void setSafetyConfig(const LancetSafetyConfig &cfg);

    /*!
        \brief 获取当前安全配置（外部单位）。
    */
    LancetSafetyConfig safetyConfig() const;

    // =================================================================
    // 点到点运动（Idle → Moving）
    // =================================================================

    /*!
        \brief 关节空间运动到目标关节角度。
        \param target 目标关节位置（外部角度单位），长度须等于 jointCount()
        \param params 运动参数（速度/加速度使用外部角度单位）
        \return 0 成功，负数错误码
    */
    int moveJoint(const LancetJoints &target, const LancetMotionParams &params);

    /*!
        \brief 关节空间运动到目标笛卡尔位姿（关节插补）。
        \param pose    目标位姿 [x,y,z,rx,ry,rz]（外部单位）
        \param params  运动参数（速度/加速度使用外部角度单位）
        \param tcpName 自定义 TCP 名称，空串使用默认
    */
    int moveJointToPose(const LancetPose &pose, const LancetMotionParams &params,
                        const QString &tcpName = QString());

    /*!
        \brief 笛卡尔直线运动到目标关节角度。
        \param target 目标关节位置（外部角度单位）
        \param params 运动参数（速度/加速度使用外部长度单位）
    */
    int moveLinear(const LancetJoints &target, const LancetMotionParams &params);

    /*!
        \brief 笛卡尔直线运动到目标位姿。
        \param pose    目标位姿 [x,y,z,rx,ry,rz]（外部单位）
        \param params  运动参数（速度/加速度使用外部长度单位）
        \param tcpName 自定义 TCP 名称
    */
    int moveLinearToPose(const LancetPose &pose, const LancetMotionParams &params,
                         const QString &tcpName = QString());

    /*!
        \brief 单关节点动（连续运动，需 stop() 停止）。
        \param jointIndex 关节索引（0-based）
        \param positive   正方向
        \param vel        速度（外部角度单位/s）
        \param acc        加速度（外部角度单位/s²）
    */
    int jogJoint(int jointIndex, bool positive, double vel, double acc);

    /*!
        \brief TCP 平移点动（连续运动，需 stop() 停止）。
        \param dir   方向
        \param vel   速度（外部长度单位/s）
        \param acc   加速度（外部长度单位/s²）
        \param frame 坐标系
        \param tcpName 自定义 TCP 名称
    */
    int jogTcp(MoveDirection dir, double vel, double acc,
               CoordinateFrame frame = CoordinateFrame::Base,
               const QString &tcpName = QString());

    /*!
        \brief TCP 旋转点动（连续运动，需 stop() 停止）。
        \param axis  旋转轴方向
        \param vel   角速度（外部角度单位/s）
        \param acc   角加速度（外部角度单位/s²）
        \param frame 坐标系
        \param tcpName 自定义 TCP 名称
    */
    int rotateTcp(MoveDirection axis, double vel, double acc,
                  CoordinateFrame frame = CoordinateFrame::Base,
                  const QString &tcpName = QString());

    // =================================================================
    // 速度控制（Idle → SpeedControl，SpeedControl 下可更新）
    // =================================================================

    /*!
        \brief 关节速度控制。
        \param vel         各关节速度（外部角度单位/s）
        \param acc         加速度（外部角度单位/s²）
        \param durationSec 持续时间（秒），0 = 连续直到 stop()
    */
    int setJointVelocity(const LancetJoints &vel, double acc,
                         double durationSec = 0.0);

    /*!
        \brief 笛卡尔速度控制。
        \param vel         6 维速度 [vx,vy,vz,wx,wy,wz]（外部单位/s）
        \param transAcc    平移加速度（外部长度单位/s²）
        \param rotAcc      旋转加速度（外部角度单位/s²）
        \param durationSec 持续时间，0 = 连续
        \param tcpName     自定义 TCP 名称
    */
    int setCartVelocity(const LancetPose &vel, double transAcc, double rotAcc,
                        double durationSec = 0.0,
                        const QString &tcpName = QString());

    // =================================================================
    // 伺服控制（Idle → ServoControl，ServoControl 下可更新）
    // =================================================================

    /*!
        \brief 关节伺服（高频位置更新，通常 ≥ 500Hz）。
        \param target 目标关节位置（外部角度单位）
    */
    int servoJoint(const LancetJoints &target);

    /*!
        \brief 笛卡尔伺服（高频位置更新）。
        \param target 目标位姿 [x,y,z,rx,ry,rz]（外部单位）
    */
    int servoCartesian(const LancetPose &target);

    // =================================================================
    // 停止 / 暂停
    // =================================================================

    /*!
        \brief 停止所有运动，恢复到 Idle。

        可在任何已连接状态下调用。
    */
    int stop();

    /*!
        \brief 暂停当前运动（仅 Moving 状态有效）。
    */
    int pause();

    /*!
        \brief 恢复暂停的运动（仅 Paused 状态有效）。
    */
    int resume();

    /*!
        \brief 紧急停止。

        绕过状态机检查，立即停止并强制进入 Error 状态。
        由安全层自动触发，也可手动调用。
    */
    int emergencyStop();

    // =================================================================
    // 特殊模式
    // =================================================================

    int enableFreeDriving(bool enable,
                          FreeDriveMode mode = FreeDriveMode::Normal);
    int releaseBrake();
    int holdBrake();

    int enterForceMode(const LancetForceParams &params);
    int leaveForceMode(int exitMode = 0);
    int updateForceTarget(double forceN);

    int setControlMode(ControlMode mode);
    int setJointImpedance(const LancetJoints &params);
    int setCartImpedance(const QVector<double> &params6);

    int enterRescueMode();
    int leaveRescueMode();

    // =================================================================
    // 状态查询（线程安全，返回外部单位）
    // =================================================================

    /*!
        \brief 获取完整状态快照（外部单位）。
    */
    LancetRobotStatus status() const;

    LancetJoints jointPositions() const;   ///< 关节位置（外部角度单位）
    LancetJoints jointVelocities() const;  ///< 关节速度（外部角度单位/s）
    LancetJoints jointTorques() const;     ///< 关节力矩（N·m，不转换）
    LancetPose   tcpPose() const;          ///< TCP 位姿（外部单位）
    double       tcpExternalForce() const; ///< TCP 外力（N，不转换）
    bool         isCollision() const;      ///< 碰撞标志
    int          lastError() const;        ///< 最近错误码

    // =================================================================
    // 运动学
    // =================================================================

    int forwardKinematics(const LancetJoints &joints, LancetPose &pose,
                          const QString &tcpName = QString()) const;
    int inverseKinematics(const LancetPose &pose, LancetJoints &joints,
                          const QString &tcpName = QString()) const;
    int jacobianMatrix(LancetJacobian &jacobian) const;

    static LancetPose      homogeneousToPose(const LancetMatrix4x4 &m);
    static LancetMatrix4x4 poseToHomogeneous(const LancetPose &p);

    // =================================================================
    // 配置（require Idle）
    // =================================================================

    int setTcpOffset(const LancetPose &pose);
    int setPayload(const QVector<double> &payload10);
    int setVelocityPercent(int pct);
    int setJointPositionLimits(const LancetJoints &minPos, const LancetJoints &maxPos);
    int setMaxJointVelocity(const LancetJoints &maxVel);
    int setCollisionLevel(int level);
    int setCollisionTorqueThreshold(const LancetJoints &threshold);
    int enableCollisionDetection(bool enable);
    int enableSingularityAvoidance(bool enable);
    int setGravity(const QVector<double> &grav3);
    int saveConfig();

    // =================================================================
    // I/O
    // =================================================================

    int    readDI(const QString &group, const QString &name) const;
    int    readDO(const QString &group, const QString &name) const;
    int    writeDO(const QString &group, const QString &name, int value);
    double readAI(const QString &group, const QString &name) const;
    int    writeAO(const QString &group, const QString &name, int mode, double value);

    // =================================================================
    // 信息
    // =================================================================

    static QString errorString(int errorCode);
    int     jointCount() const;
    QString ipAddress() const;

Q_SIGNALS:
    /*!
        \brief 状态机发生转换时发射。
        \note 可能从 C API 回调线程发射，槽函数的连接类型建议 Qt::AutoConnection。
    */
    void stateChanged(RobotState newState, RobotState oldState);

    /*!
        \brief 错误回调触发时发射。机械臂进入 Error 状态。
    */
    void errorOccurred(int errorCode, const QString &description);

    /*!
        \brief 警告回调触发时发射。不影响状态机。
    */
    void warningOccurred(int warningCode, const QString &description);

    /*!
        \brief 每帧状态推送时发射（数据为外部单位）。
    */
    void statusUpdated(const LancetRobotStatus &status);

    /*!
        \brief 点到点运动或路径执行完成时发射。
    */
    void motionCompleted();

    /*!
        \brief 安全层检测到违规时发射。
        \param description          违规描述
        \param emergencyStopTriggered 是否已触发紧急停止
    */
    void safetyViolation(const QString &description, bool emergencyStopTriggered);

private:
    Q_DISABLE_COPY(LancetRobotArm)
    QScopedPointer<LancetRobotArmPrivate> d_ptr;

    friend class LancetRobotPath;

    /*!
        \internal
        \brief 供 LancetRobotPath 在路径执行成功后推进状态机到 Moving。
    */
    void notifyPathStarted();
};

#endif // LANCETROBOTARM_H
