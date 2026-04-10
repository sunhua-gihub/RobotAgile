/*!
    \file lancetrobotarm.cpp
    \brief LancetRobotArm 实现（含 PIMPL 私有类、C API 回调分发、单位转换）。
*/

#include "lancetrobotarm.h"
#include "lancetrobotsafety.h"
#include "lancetrobotstatemachine.h"

#include <QDateTime>
#include <QElapsedTimer>
#include <QLoggingCategory>
#include <QMap>
#include <QMutex>
#include <QMutexLocker>
#include <QReadWriteLock>
#include <QWaitCondition>

#include <cmath>
#include <cstring>

// ---- 思灵机械臂 C API 头文件 ----
// 请确保项目 .pro 中配置了 DianaApi 的 INCLUDEPATH 和 LIBS
#include "DianaApi.h"

Q_LOGGING_CATEGORY(lcRobotArm, "lancet.robot.arm")

// =====================================================================
// 静态回调注册表
// =====================================================================

namespace {

QMutex s_registryMutex;
QMap<QString, LancetRobotArmPrivate *> s_registry;

void registerInstance(const QString &ip, LancetRobotArmPrivate *d)
{
    QMutexLocker lock(&s_registryMutex);
    s_registry[ip] = d;
}

void unregisterInstance(const QString &ip)
{
    QMutexLocker lock(&s_registryMutex);
    s_registry.remove(ip);
}

LancetRobotArmPrivate *findInstance(const char *ip)
{
    QMutexLocker lock(&s_registryMutex);
    return s_registry.value(QString::fromLatin1(ip), nullptr);
}

LancetRobotArmPrivate *findAnyInstance()
{
    QMutexLocker lock(&s_registryMutex);
    return s_registry.isEmpty() ? nullptr : s_registry.first();
}

} // anonymous namespace

// =====================================================================
// 私有实现类
// =====================================================================

class LancetRobotArmPrivate
{
    Q_DECLARE_PUBLIC(LancetRobotArm)

public:
    LancetRobotArmPrivate(LancetRobotArm *q, const QString &ip, int jointCount)
        : q_ptr(q)
        , m_ip(ip)
        , m_jointCount(jointCount)
        , m_stateMachine(RobotState::Disconnected)
        , m_safety(new LancetRobotSafety(q))
    {
        m_cachedStatus.jointPositions.resize(jointCount, 0.0);
        m_cachedStatus.jointVelocities.resize(jointCount, 0.0);
        m_cachedStatus.jointTorques.resize(jointCount, 0.0);
        m_cachedStatus.jointCurrents.resize(jointCount, 0.0);
        m_cachedStatus.tcpPose.resize(6, 0.0);

        QObject::connect(m_safety, &LancetRobotSafety::violationDetected,
                         q, [this](const QString &reason, bool eStop) {
            if (eStop) {
                qCCritical(lcRobotArm) << "Safety violation → emergency stop:" << reason;
                Q_Q(LancetRobotArm);
                q->emergencyStop();
                emit q->safetyViolation(reason, true);
            } else {
                Q_Q(LancetRobotArm);
                emit q->safetyViolation(reason, false);
            }
        });
    }

    // ---- 单位转换工具 ----

    inline double lengthToInternal(double v) const { return v * m_unitConfig.lengthToInternal(); }
    inline double lengthToExternal(double v) const { return v * m_unitConfig.lengthToExternal(); }
    inline double angleToInternal(double v)  const { return v * m_unitConfig.angleToInternal(); }
    inline double angleToExternal(double v)  const { return v * m_unitConfig.angleToExternal(); }

    LancetJoints jointsToInternal(const LancetJoints &ext) const
    {
        const double af = m_unitConfig.angleToInternal();
        LancetJoints result(ext.size());
        for (int i = 0; i < ext.size(); ++i)
            result[i] = ext[i] * af;
        return result;
    }

    LancetJoints jointsToExternal(const LancetJoints &internal) const
    {
        const double af = m_unitConfig.angleToExternal();
        LancetJoints result(internal.size());
        for (int i = 0; i < internal.size(); ++i)
            result[i] = internal[i] * af;
        return result;
    }

    LancetPose poseToInternal(const LancetPose &ext) const
    {
        if (ext.size() < 6)
            return ext;
        const double lf = m_unitConfig.lengthToInternal();
        const double af = m_unitConfig.angleToInternal();
        return { ext[0]*lf, ext[1]*lf, ext[2]*lf,
                 ext[3]*af, ext[4]*af, ext[5]*af };
    }

    LancetPose poseToExternal(const LancetPose &internal) const
    {
        if (internal.size() < 6)
            return internal;
        const double lf = m_unitConfig.lengthToExternal();
        const double af = m_unitConfig.angleToExternal();
        return { internal[0]*lf, internal[1]*lf, internal[2]*lf,
                 internal[3]*af, internal[4]*af, internal[5]*af };
    }

    // ---- 状态转换辅助 ----

    RobotState doTransition(LancetRobotStateMachine::Event event)
    {
        QMutexLocker lock(&m_stateMutex);
        RobotState oldState;
        RobotState newState = m_stateMachine.transition(event, &oldState);
        if (newState != oldState) {
            qCInfo(lcRobotArm) << "State transition:"
                               << LancetRobotStateMachine::stateName(oldState)
                               << "→" << LancetRobotStateMachine::stateName(newState);
            Q_Q(LancetRobotArm);
            emit q->stateChanged(newState, oldState);

            // 唤醒 waitForIdle
            if (newState == RobotState::Idle
                || newState == RobotState::Error
                || newState == RobotState::Disconnected) {
                m_waitCondition.wakeAll();
            }
        }
        return newState;
    }

    bool checkCanIssueMotion(int *errorCode) const
    {
        QMutexLocker lock(&m_stateMutex);
        if (m_stateMachine.canAcceptMotion())
            return true;
        if (errorCode)
            *errorCode = m_stateMachine.motionConflictError();
        return false;
    }

    bool checkJointCount(const LancetJoints &joints) const
    {
        return joints.size() == m_jointCount;
    }

    bool checkPoseSize(const LancetPose &pose) const
    {
        return pose.size() == 6;
    }

    // ---- C API 回调处理 ----

    void onErrorCallback(int errorCode)
    {
        m_lastError = errorCode;
        qCCritical(lcRobotArm) << "Robot error:" << errorCode
                               << LancetRobotArm::errorString(errorCode);
        doTransition(LancetRobotStateMachine::Event::ErrorOccurred);
        Q_Q(LancetRobotArm);
        emit q->errorOccurred(errorCode, LancetRobotArm::errorString(errorCode));
    }

    void onWarningCallback(int warningCode)
    {
        qCWarning(lcRobotArm) << "Robot warning:" << warningCode;
        Q_Q(LancetRobotArm);
        emit q->warningOccurred(warningCode, QString("Warning %1").arg(warningCode));
    }

    void onStateCallback(StrRobotStateInfo *info)
    {
        if (!info)
            return;

        // 更新缓存状态（内部单位）
        {
            QWriteLocker lock(&m_statusLock);
            for (int i = 0; i < m_jointCount; ++i) {
                m_cachedStatus.jointPositions[i]  = info->jointPos[i];
                m_cachedStatus.jointVelocities[i] = info->jointAngularVel[i];
                m_cachedStatus.jointTorques[i]    = info->jointTorque[i];
                m_cachedStatus.jointCurrents[i]   = info->jointCurrent[i];
            }
            for (int i = 0; i < 6; ++i)
                m_cachedStatus.tcpPose[i] = info->tcpPos[i];
            m_cachedStatus.tcpExternalForce = info->tcpExternalForce;
            m_cachedStatus.collision        = info->bCollision;
            m_cachedStatus.timestamp        = QDateTime::currentMSecsSinceEpoch();
            m_cachedStatus.state            = m_stateMachine.currentState();
        }

        // 运行时安全监控
        m_safety->checkRuntimeStatus(m_cachedStatus);

        // 运动完成检测
        checkMotionCompletion();

        // 发射 statusUpdated 信号（转换为外部单位）
        Q_Q(LancetRobotArm);
        LancetRobotStatus extStatus = statusToExternal(m_cachedStatus);
        emit q->statusUpdated(extStatus);
    }

    void checkMotionCompletion()
    {
        QMutexLocker lock(&m_stateMutex);
        if (m_stateMachine.currentState() != RobotState::Moving) {
            m_slowFrameCount = 0;
            return;
        }

        bool allSlow = true;
        {
            QReadLocker rlock(&m_statusLock);
            for (int i = 0; i < m_jointCount; ++i) {
                if (std::abs(m_cachedStatus.jointVelocities[i]) > kVelocityThreshold) {
                    allSlow = false;
                    break;
                }
            }
        }

        if (allSlow) {
            if (++m_slowFrameCount >= kSlowFrameThreshold) {
                m_slowFrameCount = 0;
                RobotState oldState;
                RobotState newState = m_stateMachine.transition(
                    LancetRobotStateMachine::Event::MotionCompleted, &oldState);
                if (newState != oldState) {
                    qCInfo(lcRobotArm) << "Motion completed, velocity settled";
                    Q_Q(LancetRobotArm);
                    emit q->stateChanged(newState, oldState);
                    emit q->motionCompleted();
                    m_waitCondition.wakeAll();
                }
            }
        } else {
            m_slowFrameCount = 0;
        }
    }

    LancetRobotStatus statusToExternal(const LancetRobotStatus &s) const
    {
        LancetRobotStatus ext;
        ext.state             = s.state;
        ext.jointPositions    = jointsToExternal(s.jointPositions);
        ext.jointVelocities   = jointsToExternal(s.jointVelocities);
        ext.jointTorques      = s.jointTorques;   // N·m 不转换
        ext.jointCurrents     = s.jointCurrents;  // A 不转换
        ext.tcpPose           = poseToExternal(s.tcpPose);
        ext.tcpExternalForce  = s.tcpExternalForce;  // N 不转换
        ext.collision         = s.collision;
        ext.timestamp         = s.timestamp;
        return ext;
    }

    // ---- 成员 ----

    LancetRobotArm *q_ptr;
    QString          m_ip;
    int              m_jointCount;
    LancetUnitConfig m_unitConfig;

    mutable QMutex              m_stateMutex;
    LancetRobotStateMachine     m_stateMachine;

    mutable QReadWriteLock      m_statusLock;
    LancetRobotStatus           m_cachedStatus;

    QMutex                      m_waitMutex;
    QWaitCondition              m_waitCondition;

    LancetRobotSafety          *m_safety;
    int                         m_lastError = 0;

    // 运动完成检测
    int m_slowFrameCount = 0;
    static constexpr int    kSlowFrameThreshold = 3;
    static constexpr double kVelocityThreshold  = 1e-3;  // rad/s
};

// =====================================================================
// C API 静态回调函数
// =====================================================================

static void s_errorCallback(int e)
{
    auto *d = findAnyInstance();
    if (d)
        d->onErrorCallback(e);
}

static void s_warningCallback(int w, const char *ip)
{
    auto *d = findInstance(ip);
    if (!d)
        d = findAnyInstance();
    if (d)
        d->onWarningCallback(w);
}

static void s_stateCallback(StrRobotStateInfo *info, const char *ip)
{
    auto *d = findInstance(ip);
    if (d)
        d->onStateCallback(info);
}

// =====================================================================
// 构造 / 析构
// =====================================================================

LancetRobotArm::LancetRobotArm(const QString &ip, int jointCount, QObject *parent)
    : QObject(parent)
    , d_ptr(new LancetRobotArmPrivate(this, ip, jointCount))
{
    qRegisterMetaType<RobotState>("RobotState");
    qRegisterMetaType<LancetRobotStatus>("LancetRobotStatus");
}

LancetRobotArm::~LancetRobotArm()
{
    disconnectFromRobot();
}

// =====================================================================
// 连接管理
// =====================================================================

int LancetRobotArm::connectToRobot()
{
    Q_D(LancetRobotArm);

    if (isConnected())
        return LancetRobotError::Success;

    srv_net_st netInfo;
    initSrvNetInfo(&netInfo);
    std::string ipStd = d->m_ip.toStdString();
    std::strncpy(netInfo.SrvIp, ipStd.c_str(), sizeof(netInfo.SrvIp) - 1);

    registerInstance(d->m_ip, d);

    int ret = initSrvV2(s_errorCallback, s_warningCallback, s_stateCallback, &netInfo);
    if (ret != 0) {
        qCCritical(lcRobotArm) << "Failed to connect to" << d->m_ip << ", error:" << ret;
        unregisterInstance(d->m_ip);
        return ret;
    }

    // 配置心跳
    const auto &safety = d->m_safety->config();
    if (safety.heartbeatTimeoutMs > 0) {
        setHeartbeatParam(safety.heartbeatTimeoutMs,
                          safety.heartbeatTimeoutMs,
                          ipStd.c_str());
    }

    d->doTransition(LancetRobotStateMachine::Event::Connected);
    qCInfo(lcRobotArm) << "Connected to robot at" << d->m_ip;
    return LancetRobotError::Success;
}

void LancetRobotArm::disconnectFromRobot()
{
    Q_D(LancetRobotArm);

    if (!isConnected())
        return;

    std::string ipStd = d->m_ip.toStdString();
    destroySrv(ipStd.c_str());
    unregisterInstance(d->m_ip);
    d->doTransition(LancetRobotStateMachine::Event::Disconnected);
    qCInfo(lcRobotArm) << "Disconnected from" << d->m_ip;
}

bool LancetRobotArm::isConnected() const
{
    Q_D(const LancetRobotArm);
    QMutexLocker lock(&d->m_stateMutex);
    return d->m_stateMachine.isConnected();
}

int LancetRobotArm::setStatePushPeriod(int ms)
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return setPushPeriod(ms, ipStd.c_str());
}

// =====================================================================
// 单位配置
// =====================================================================

void LancetRobotArm::setUnitConfig(const LancetUnitConfig &cfg)
{
    Q_D(LancetRobotArm);
    d->m_unitConfig = cfg;
    // 重新转换安全配置
    d->m_safety->setConfig(d->m_safety->config(), cfg);
}

LancetUnitConfig LancetRobotArm::unitConfig() const
{
    Q_D(const LancetRobotArm);
    return d->m_unitConfig;
}

// =====================================================================
// 状态机查询
// =====================================================================

RobotState LancetRobotArm::state() const
{
    Q_D(const LancetRobotArm);
    QMutexLocker lock(&d->m_stateMutex);
    return d->m_stateMachine.currentState();
}

bool LancetRobotArm::isIdle() const
{
    return state() == RobotState::Idle;
}

bool LancetRobotArm::isMoving() const
{
    Q_D(const LancetRobotArm);
    QMutexLocker lock(&d->m_stateMutex);
    return d->m_stateMachine.isMoving();
}

bool LancetRobotArm::inError() const
{
    return state() == RobotState::Error;
}

int LancetRobotArm::waitForIdle(int timeoutMs)
{
    Q_D(LancetRobotArm);
    QMutexLocker lock(&d->m_waitMutex);

    auto isTerminal = [this]() {
        RobotState s = state();
        return s == RobotState::Idle
            || s == RobotState::Error
            || s == RobotState::Disconnected;
    };

    if (isTerminal())
        return (state() == RobotState::Idle) ? 0 : d->m_lastError;

    QElapsedTimer timer;
    timer.start();

    while (!isTerminal()) {
        if (timeoutMs >= 0) {
            qint64 remaining = timeoutMs - timer.elapsed();
            if (remaining <= 0)
                return LancetRobotError::Timeout;
            d->m_waitCondition.wait(&d->m_waitMutex,
                                    static_cast<unsigned long>(remaining));
        } else {
            d->m_waitCondition.wait(&d->m_waitMutex);
        }
    }

    return (state() == RobotState::Idle) ? 0 : d->m_lastError;
}

int LancetRobotArm::clearError()
{
    Q_D(LancetRobotArm);
    if (state() != RobotState::Error)
        return LancetRobotError::Success;

    std::string ipStd = d->m_ip.toStdString();
    cleanErrorInfo(ipStd.c_str());
    d->m_lastError = 0;
    d->doTransition(LancetRobotStateMachine::Event::ErrorCleared);
    return LancetRobotError::Success;
}

// =====================================================================
// 安全配置
// =====================================================================

void LancetRobotArm::setSafetyConfig(const LancetSafetyConfig &cfg)
{
    Q_D(LancetRobotArm);
    d->m_safety->setConfig(cfg, d->m_unitConfig);
}

LancetSafetyConfig LancetRobotArm::safetyConfig() const
{
    Q_D(const LancetRobotArm);
    return d->m_safety->config();
}

// =====================================================================
// 点到点运动
// =====================================================================

int LancetRobotArm::moveJoint(const LancetJoints &target, const LancetMotionParams &params)
{
    Q_D(LancetRobotArm);
    if (!d->checkJointCount(target))
        return LancetRobotError::JointCountMismatch;

    int err;
    if (!d->checkCanIssueMotion(&err))
        return err;

    double velRad = d->angleToInternal(params.velocity);
    double accRad = d->angleToInternal(params.acceleration);

    QString reason;
    if (!d->m_safety->validateJointMotion(velRad, accRad, &reason))
        return LancetRobotError::SafetyRejected;

    LancetJoints jointsRad = d->jointsToInternal(target);
    std::string ipStd = d->m_ip.toStdString();

    qCInfo(lcRobotArm) << "moveJoint: vel=" << velRad << "rad/s, acc=" << accRad << "rad/s^2";

    int ret = moveJToTarget(jointsRad.data(), velRad, accRad,
                            params.zvOrder, params.zvFrequency, params.zvDamping,
                            ipStd.c_str());
    if (ret == 0)
        d->doTransition(LancetRobotStateMachine::Event::MoveIssued);
    return ret;
}

int LancetRobotArm::moveJointToPose(const LancetPose &pose, const LancetMotionParams &params,
                                      const QString &tcpName)
{
    Q_D(LancetRobotArm);
    if (!d->checkPoseSize(pose))
        return LancetRobotError::InvalidParam;

    int err;
    if (!d->checkCanIssueMotion(&err))
        return err;

    double velRad = d->angleToInternal(params.velocity);
    double accRad = d->angleToInternal(params.acceleration);

    QString reason;
    if (!d->m_safety->validateJointMotion(velRad, accRad, &reason))
        return LancetRobotError::SafetyRejected;

    LancetPose poseInternal = d->poseToInternal(pose);

    if (!d->m_safety->validatePoseInWorkspace(poseInternal, &reason))
        return LancetRobotError::SafetyRejected;

    std::string ipStd = d->m_ip.toStdString();
    QByteArray tcpBa = tcpName.toLatin1();
    const char *tcp = tcpName.isEmpty() ? nullptr : tcpBa.constData();

    int ret = moveJToPose(poseInternal.data(), velRad, accRad, tcp,
                          params.zvOrder, params.zvFrequency, params.zvDamping,
                          ipStd.c_str());
    if (ret == 0)
        d->doTransition(LancetRobotStateMachine::Event::MoveIssued);
    return ret;
}

int LancetRobotArm::moveLinear(const LancetJoints &target, const LancetMotionParams &params)
{
    Q_D(LancetRobotArm);
    if (!d->checkJointCount(target))
        return LancetRobotError::JointCountMismatch;

    int err;
    if (!d->checkCanIssueMotion(&err))
        return err;

    double velMs  = d->lengthToInternal(params.velocity);
    double accMs  = d->lengthToInternal(params.acceleration);

    QString reason;
    if (!d->m_safety->validateCartMotion(velMs, accMs, &reason))
        return LancetRobotError::SafetyRejected;

    LancetJoints jointsRad = d->jointsToInternal(target);
    std::string ipStd = d->m_ip.toStdString();

    int ret = moveLToTarget(jointsRad.data(), velMs, accMs,
                            params.zvOrder, params.zvFrequency, params.zvDamping,
                            params.avoidSingular, ipStd.c_str());
    if (ret == 0)
        d->doTransition(LancetRobotStateMachine::Event::MoveIssued);
    return ret;
}

int LancetRobotArm::moveLinearToPose(const LancetPose &pose, const LancetMotionParams &params,
                                       const QString &tcpName)
{
    Q_D(LancetRobotArm);
    if (!d->checkPoseSize(pose))
        return LancetRobotError::InvalidParam;

    int err;
    if (!d->checkCanIssueMotion(&err))
        return err;

    double velMs  = d->lengthToInternal(params.velocity);
    double accMs  = d->lengthToInternal(params.acceleration);

    QString reason;
    if (!d->m_safety->validateCartMotion(velMs, accMs, &reason))
        return LancetRobotError::SafetyRejected;

    LancetPose poseInternal = d->poseToInternal(pose);

    if (!d->m_safety->validatePoseInWorkspace(poseInternal, &reason))
        return LancetRobotError::SafetyRejected;

    std::string ipStd = d->m_ip.toStdString();
    QByteArray tcpBa = tcpName.toLatin1();
    const char *tcp = tcpName.isEmpty() ? nullptr : tcpBa.constData();

    int ret = moveLToPose(poseInternal.data(), velMs, accMs, tcp,
                          params.zvOrder, params.zvFrequency, params.zvDamping,
                          params.avoidSingular, ipStd.c_str());
    if (ret == 0)
        d->doTransition(LancetRobotStateMachine::Event::MoveIssued);
    return ret;
}

int LancetRobotArm::jogJoint(int jointIndex, bool positive, double vel, double acc)
{
    Q_D(LancetRobotArm);
    if (jointIndex < 0 || jointIndex >= d->m_jointCount)
        return LancetRobotError::InvalidParam;

    int err;
    if (!d->checkCanIssueMotion(&err))
        return err;

    double velRad = d->angleToInternal(vel);
    double accRad = d->angleToInternal(acc);

    QString reason;
    if (!d->m_safety->validateJointMotion(velRad, accRad, &reason))
        return LancetRobotError::SafetyRejected;

    std::string ipStd = d->m_ip.toStdString();
    joint_direction_e dir = positive ? T_MOVE_UP : T_MOVE_DOWN;

    int ret = ::moveJoint(dir, jointIndex, velRad, accRad, ipStd.c_str());
    if (ret == 0)
        d->doTransition(LancetRobotStateMachine::Event::MoveIssued);
    return ret;
}

int LancetRobotArm::jogTcp(MoveDirection dir, double vel, double acc,
                             CoordinateFrame frame, const QString &tcpName)
{
    Q_D(LancetRobotArm);

    int err;
    if (!d->checkCanIssueMotion(&err))
        return err;

    double velMs = d->lengthToInternal(vel);
    double accMs = d->lengthToInternal(acc);

    QString reason;
    if (!d->m_safety->validateCartMotion(velMs, accMs, &reason))
        return LancetRobotError::SafetyRejected;

    std::string ipStd = d->m_ip.toStdString();

    // MoveDirection → tcp_direction_e 映射
    static const tcp_direction_e dirMap[] = {
        T_MOVE_X_UP, T_MOVE_X_DOWN,
        T_MOVE_Y_UP, T_MOVE_Y_DOWN,
        T_MOVE_Z_UP, T_MOVE_Z_DOWN
    };
    tcp_direction_e apiDir = dirMap[static_cast<int>(dir)];
    coordinate_e apiCoord  = static_cast<coordinate_e>(static_cast<int>(frame));

    int ret = moveTcp_ex(apiCoord, apiDir, velMs, accMs, ipStd.c_str());
    if (ret == 0)
        d->doTransition(LancetRobotStateMachine::Event::MoveIssued);
    return ret;
}

int LancetRobotArm::rotateTcp(MoveDirection axis, double vel, double acc,
                                CoordinateFrame frame, const QString &tcpName)
{
    Q_D(LancetRobotArm);

    int err;
    if (!d->checkCanIssueMotion(&err))
        return err;

    double velRad = d->angleToInternal(vel);
    double accRad = d->angleToInternal(acc);

    QString reason;
    if (!d->m_safety->validateJointMotion(velRad, accRad, &reason))
        return LancetRobotError::SafetyRejected;

    std::string ipStd = d->m_ip.toStdString();

    static const tcp_direction_e dirMap[] = {
        T_MOVE_X_UP, T_MOVE_X_DOWN,
        T_MOVE_Y_UP, T_MOVE_Y_DOWN,
        T_MOVE_Z_UP, T_MOVE_Z_DOWN
    };
    tcp_direction_e apiDir = dirMap[static_cast<int>(axis)];
    coordinate_e apiCoord  = static_cast<coordinate_e>(static_cast<int>(frame));

    int ret = rotationTCP_ex(apiCoord, apiDir, velRad, accRad, ipStd.c_str());
    if (ret == 0)
        d->doTransition(LancetRobotStateMachine::Event::MoveIssued);
    return ret;
}

// =====================================================================
// 速度控制
// =====================================================================

int LancetRobotArm::setJointVelocity(const LancetJoints &vel, double acc,
                                       double durationSec)
{
    Q_D(LancetRobotArm);
    if (!d->checkJointCount(vel))
        return LancetRobotError::JointCountMismatch;

    // SpeedControl 下允许更新
    {
        QMutexLocker lock(&d->m_stateMutex);
        RobotState s = d->m_stateMachine.currentState();
        if (s != RobotState::Idle && s != RobotState::SpeedControl)
            return d->m_stateMachine.motionConflictError();
    }

    double accRad = d->angleToInternal(acc);
    LancetJoints velRad = d->jointsToInternal(vel);

    std::string ipStd = d->m_ip.toStdString();
    int ret = speedJ(velRad.data(), accRad, durationSec, ipStd.c_str());
    if (ret == 0 && state() == RobotState::Idle)
        d->doTransition(LancetRobotStateMachine::Event::SpeedIssued);
    return ret;
}

int LancetRobotArm::setCartVelocity(const LancetPose &vel, double transAcc, double rotAcc,
                                      double durationSec, const QString &tcpName)
{
    Q_D(LancetRobotArm);
    if (!d->checkPoseSize(vel))
        return LancetRobotError::InvalidParam;

    {
        QMutexLocker lock(&d->m_stateMutex);
        RobotState s = d->m_stateMachine.currentState();
        if (s != RobotState::Idle && s != RobotState::SpeedControl)
            return d->m_stateMachine.motionConflictError();
    }

    LancetPose velInternal = d->poseToInternal(vel);
    double transAccMs = d->lengthToInternal(transAcc);
    double rotAccRad  = d->angleToInternal(rotAcc);
    double accArr[2]  = { transAccMs, rotAccRad };

    std::string ipStd = d->m_ip.toStdString();
    QByteArray tcpBa = tcpName.toLatin1();
    const char *tcp = tcpName.isEmpty() ? nullptr : tcpBa.constData();

    int ret = speedL(velInternal.data(), accArr, durationSec, tcp, ipStd.c_str());
    if (ret == 0 && state() == RobotState::Idle)
        d->doTransition(LancetRobotStateMachine::Event::SpeedIssued);
    return ret;
}

// =====================================================================
// 伺服控制
// =====================================================================

int LancetRobotArm::servoJoint(const LancetJoints &target)
{
    Q_D(LancetRobotArm);
    if (!d->checkJointCount(target))
        return LancetRobotError::JointCountMismatch;

    {
        QMutexLocker lock(&d->m_stateMutex);
        RobotState s = d->m_stateMachine.currentState();
        if (s != RobotState::Idle && s != RobotState::ServoControl)
            return d->m_stateMachine.motionConflictError();
    }

    LancetJoints jointsRad = d->jointsToInternal(target);
    std::string ipStd = d->m_ip.toStdString();

    int ret = servoJ(jointsRad.data(), ipStd.c_str());
    if (ret == 0 && state() == RobotState::Idle)
        d->doTransition(LancetRobotStateMachine::Event::ServoIssued);
    return ret;
}

int LancetRobotArm::servoCartesian(const LancetPose &target)
{
    Q_D(LancetRobotArm);
    if (!d->checkPoseSize(target))
        return LancetRobotError::InvalidParam;

    {
        QMutexLocker lock(&d->m_stateMutex);
        RobotState s = d->m_stateMachine.currentState();
        if (s != RobotState::Idle && s != RobotState::ServoControl)
            return d->m_stateMachine.motionConflictError();
    }

    LancetPose poseInternal = d->poseToInternal(target);
    std::string ipStd = d->m_ip.toStdString();

    int ret = servoL(poseInternal.data(), ipStd.c_str());
    if (ret == 0 && state() == RobotState::Idle)
        d->doTransition(LancetRobotStateMachine::Event::ServoIssued);
    return ret;
}

// =====================================================================
// 停止 / 暂停
// =====================================================================

int LancetRobotArm::stop()
{
    Q_D(LancetRobotArm);
    if (!isConnected())
        return LancetRobotError::NotConnected;

    std::string ipStd = d->m_ip.toStdString();
    int ret = ::stop(ipStd.c_str());
    if (ret == 0)
        d->doTransition(LancetRobotStateMachine::Event::Stopped);
    return ret;
}

int LancetRobotArm::pause()
{
    Q_D(LancetRobotArm);
    if (state() != RobotState::Moving)
        return LancetRobotError::CallingConflict;

    std::string ipStd = d->m_ip.toStdString();
    int ret = pauseProgram(ipStd.c_str());
    if (ret == 0)
        d->doTransition(LancetRobotStateMachine::Event::Paused);
    return ret;
}

int LancetRobotArm::resume()
{
    Q_D(LancetRobotArm);
    if (state() != RobotState::Paused)
        return LancetRobotError::CallingConflict;

    std::string ipStd = d->m_ip.toStdString();
    int ret = resumeProgram(ipStd.c_str());
    if (ret == 0)
        d->doTransition(LancetRobotStateMachine::Event::Resumed);
    return ret;
}

int LancetRobotArm::emergencyStop()
{
    Q_D(LancetRobotArm);
    qCCritical(lcRobotArm) << "EMERGENCY STOP triggered";

    std::string ipStd = d->m_ip.toStdString();
    int ret = ::stop(ipStd.c_str());

    // 强制进入 Error 状态，绕过正常转换表
    {
        QMutexLocker lock(&d->m_stateMutex);
        RobotState oldState = d->m_stateMachine.currentState();
        d->m_stateMachine.forceState(RobotState::Error);
        if (oldState != RobotState::Error) {
            emit stateChanged(RobotState::Error, oldState);
            d->m_waitCondition.wakeAll();
        }
    }

    return ret;
}

// =====================================================================
// 特殊模式
// =====================================================================

int LancetRobotArm::enableFreeDriving(bool enable, FreeDriveMode mode)
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();

    if (enable) {
        if (state() != RobotState::Idle)
            return LancetRobotError::CallingConflict;

        int ret = freeDriving(static_cast<int>(mode), ipStd.c_str());
        if (ret == 0)
            d->doTransition(LancetRobotStateMachine::Event::FreeDrivingEnabled);
        return ret;
    } else {
        if (state() != RobotState::FreeDriving)
            return LancetRobotError::CallingConflict;

        int ret = freeDriving(0, ipStd.c_str());
        if (ret == 0)
            d->doTransition(LancetRobotStateMachine::Event::FreeDrivingDisabled);
        return ret;
    }
}

int LancetRobotArm::releaseBrake()
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return ::releaseBrake(ipStd.c_str());
}

int LancetRobotArm::holdBrake()
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return ::holdBrake(ipStd.c_str());
}

int LancetRobotArm::enterForceMode(const LancetForceParams &params)
{
    Q_D(LancetRobotArm);
    if (state() != RobotState::Idle)
        return LancetRobotError::CallingConflict;

    std::string ipStd = d->m_ip.toStdString();
    int ret = ::enterForceMode(params.frameType,
                               const_cast<double *>(params.frameMatrix.constData()),
                               const_cast<double *>(params.forceDirection.constData()),
                               params.forceValue,
                               params.maxVelocity,
                               params.maxOffset,
                               ipStd.c_str());
    if (ret == 0)
        d->doTransition(LancetRobotStateMachine::Event::ForceModeEntered);
    return ret;
}

int LancetRobotArm::leaveForceMode(int exitMode)
{
    Q_D(LancetRobotArm);
    if (state() != RobotState::ForceMode)
        return LancetRobotError::CallingConflict;

    std::string ipStd = d->m_ip.toStdString();
    int ret = ::leaveForceMode(exitMode, ipStd.c_str());
    if (ret == 0)
        d->doTransition(LancetRobotStateMachine::Event::ForceModeLeft);
    return ret;
}

int LancetRobotArm::updateForceTarget(double forceN)
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return updateForce(forceN, ipStd.c_str());
}

int LancetRobotArm::setControlMode(ControlMode mode)
{
    Q_D(LancetRobotArm);

    int apiMode;
    LancetRobotStateMachine::Event event;
    switch (mode) {
    case ControlMode::Position:
        apiMode = T_MODE_POSITION;
        event = LancetRobotStateMachine::Event::ImpedanceModeLeft;
        break;
    case ControlMode::JointImpedance:
        apiMode = T_MODE_JOINT_IMPEDANCE;
        event = LancetRobotStateMachine::Event::ImpedanceModeEntered;
        break;
    case ControlMode::CartImpedance:
        apiMode = T_MODE_CART_IMPEDANCE;
        event = LancetRobotStateMachine::Event::ImpedanceModeEntered;
        break;
    default:
        return LancetRobotError::InvalidParam;
    }

    std::string ipStd = d->m_ip.toStdString();
    int ret = changeControlMode(apiMode, ipStd.c_str());
    if (ret == 0)
        d->doTransition(event);
    return ret;
}

int LancetRobotArm::setJointImpedance(const LancetJoints &params)
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return setJointImpeda(const_cast<double *>(params.constData()), ipStd.c_str());
}

int LancetRobotArm::setCartImpedance(const QVector<double> &params6)
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return setCartImpeda(const_cast<double *>(params6.constData()), ipStd.c_str());
}

int LancetRobotArm::enterRescueMode()
{
    Q_D(LancetRobotArm);
    if (state() != RobotState::Idle)
        return LancetRobotError::CallingConflict;

    std::string ipStd = d->m_ip.toStdString();
    int ret = ::enterRescueMode(ipStd.c_str());
    if (ret == 0)
        d->doTransition(LancetRobotStateMachine::Event::RescueModeEntered);
    return ret;
}

int LancetRobotArm::leaveRescueMode()
{
    Q_D(LancetRobotArm);
    if (state() != RobotState::RescueMode)
        return LancetRobotError::CallingConflict;

    std::string ipStd = d->m_ip.toStdString();
    int ret = ::leaveRescueMode(ipStd.c_str());
    if (ret == 0)
        d->doTransition(LancetRobotStateMachine::Event::RescueModeLeft);
    return ret;
}

// =====================================================================
// 状态查询（外部单位）
// =====================================================================

LancetRobotStatus LancetRobotArm::status() const
{
    Q_D(const LancetRobotArm);
    QReadLocker lock(&d->m_statusLock);
    return d->statusToExternal(d->m_cachedStatus);
}

LancetJoints LancetRobotArm::jointPositions() const
{
    Q_D(const LancetRobotArm);
    QReadLocker lock(&d->m_statusLock);
    return d->jointsToExternal(d->m_cachedStatus.jointPositions);
}

LancetJoints LancetRobotArm::jointVelocities() const
{
    Q_D(const LancetRobotArm);
    QReadLocker lock(&d->m_statusLock);
    return d->jointsToExternal(d->m_cachedStatus.jointVelocities);
}

LancetJoints LancetRobotArm::jointTorques() const
{
    Q_D(const LancetRobotArm);
    QReadLocker lock(&d->m_statusLock);
    return d->m_cachedStatus.jointTorques;  // N·m 不转换
}

LancetPose LancetRobotArm::tcpPose() const
{
    Q_D(const LancetRobotArm);
    QReadLocker lock(&d->m_statusLock);
    return d->poseToExternal(d->m_cachedStatus.tcpPose);
}

double LancetRobotArm::tcpExternalForce() const
{
    Q_D(const LancetRobotArm);
    QReadLocker lock(&d->m_statusLock);
    return d->m_cachedStatus.tcpExternalForce;  // N 不转换
}

bool LancetRobotArm::isCollision() const
{
    Q_D(const LancetRobotArm);
    QReadLocker lock(&d->m_statusLock);
    return d->m_cachedStatus.collision;
}

int LancetRobotArm::lastError() const
{
    Q_D(const LancetRobotArm);
    return d->m_lastError;
}

// =====================================================================
// 运动学
// =====================================================================

int LancetRobotArm::forwardKinematics(const LancetJoints &joints, LancetPose &pose,
                                        const QString &tcpName) const
{
    Q_D(const LancetRobotArm);
    LancetJoints jointsRad = d->jointsToInternal(joints);
    double poseRaw[6] = {};
    std::string ipStd = d->m_ip.toStdString();
    QByteArray tcpBa = tcpName.toLatin1();
    const char *tcp = tcpName.isEmpty() ? nullptr : tcpBa.constData();

    int ret = forward(jointsRad.data(), poseRaw, tcp, ipStd.c_str());
    if (ret == 0) {
        LancetPose poseInternal = {poseRaw[0], poseRaw[1], poseRaw[2],
                                   poseRaw[3], poseRaw[4], poseRaw[5]};
        pose = d->poseToExternal(poseInternal);
    }
    return ret;
}

int LancetRobotArm::inverseKinematics(const LancetPose &pose, LancetJoints &joints,
                                        const QString &tcpName) const
{
    Q_D(const LancetRobotArm);
    LancetPose poseInternal = d->poseToInternal(pose);
    LancetJoints jointsRaw(d->m_jointCount, 0.0);
    std::string ipStd = d->m_ip.toStdString();
    QByteArray tcpBa = tcpName.toLatin1();
    const char *tcp = tcpName.isEmpty() ? nullptr : tcpBa.constData();

    int ret = inverse(poseInternal.data(), jointsRaw.data(), tcp, ipStd.c_str());
    if (ret == 0)
        joints = d->jointsToExternal(jointsRaw);
    return ret;
}

int LancetRobotArm::jacobianMatrix(LancetJacobian &jacobian) const
{
    Q_D(const LancetRobotArm);
    jacobian.resize(6 * d->m_jointCount);
    std::string ipStd = d->m_ip.toStdString();
    return getJacobiMatrix(jacobian.data(), ipStd.c_str());
}

LancetPose LancetRobotArm::homogeneousToPose(const LancetMatrix4x4 &m)
{
    LancetPose pose(6, 0.0);
    double raw[6] = {};
    homogeneous2Pose(const_cast<double *>(m.constData()), raw);
    for (int i = 0; i < 6; ++i)
        pose[i] = raw[i];
    return pose;
}

LancetMatrix4x4 LancetRobotArm::poseToHomogeneous(const LancetPose &p)
{
    LancetMatrix4x4 m(16, 0.0);
    pose2Homogeneous(const_cast<double *>(p.constData()), m.data());
    return m;
}

// =====================================================================
// 配置
// =====================================================================

int LancetRobotArm::setTcpOffset(const LancetPose &pose)
{
    Q_D(LancetRobotArm);
    LancetPose poseInternal = d->poseToInternal(pose);
    std::string ipStd = d->m_ip.toStdString();
    return setDefaultActiveTcpPose(poseInternal.data(), ipStd.c_str());
}

int LancetRobotArm::setPayload(const QVector<double> &payload10)
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return setActiveTcpPayload(const_cast<double *>(payload10.constData()), ipStd.c_str());
}

int LancetRobotArm::setVelocityPercent(int pct)
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return setVelocityPercentValue(pct, ipStd.c_str());
}

int LancetRobotArm::setJointPositionLimits(const LancetJoints &minPos,
                                              const LancetJoints &maxPos)
{
    Q_D(LancetRobotArm);
    LancetJoints minRad = d->jointsToInternal(minPos);
    LancetJoints maxRad = d->jointsToInternal(maxPos);
    std::string ipStd = d->m_ip.toStdString();
    return setJointsPositionRange(minRad.data(), maxRad.data(), ipStd.c_str());
}

int LancetRobotArm::setMaxJointVelocity(const LancetJoints &maxVel)
{
    Q_D(LancetRobotArm);
    LancetJoints maxVelRad = d->jointsToInternal(maxVel);
    std::string ipStd = d->m_ip.toStdString();
    return setMaxJointsVel(maxVelRad.data(), ipStd.c_str());
}

int LancetRobotArm::setCollisionLevel(int level)
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return ::setCollisionLevel(level, ipStd.c_str());
}

int LancetRobotArm::setCollisionTorqueThreshold(const LancetJoints &threshold)
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return setThresholdTorque(const_cast<double *>(threshold.constData()), ipStd.c_str());
}

int LancetRobotArm::enableCollisionDetection(bool enable)
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return ::enableCollisionDetection(enable, ipStd.c_str());
}

int LancetRobotArm::enableSingularityAvoidance(bool enable)
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return setAvoidSingular(enable, ipStd.c_str());
}

int LancetRobotArm::setGravity(const QVector<double> &grav3)
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return setGravInfo(const_cast<double *>(grav3.constData()), ipStd.c_str());
}

int LancetRobotArm::saveConfig()
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return saveEnvironment(ipStd.c_str());
}

// =====================================================================
// I/O
// =====================================================================

int LancetRobotArm::readDI(const QString &group, const QString &name) const
{
    Q_D(const LancetRobotArm);
    int value = 0;
    std::string ipStd = d->m_ip.toStdString();
    int ret = ::readDI(group.toLatin1().constData(), name.toLatin1().constData(),
                       &value, ipStd.c_str());
    return (ret == 0) ? value : ret;
}

int LancetRobotArm::readDO(const QString &group, const QString &name) const
{
    Q_D(const LancetRobotArm);
    int value = 0;
    std::string ipStd = d->m_ip.toStdString();
    int ret = ::readDO(group.toLatin1().constData(), name.toLatin1().constData(),
                       &value, ipStd.c_str());
    return (ret == 0) ? value : ret;
}

int LancetRobotArm::writeDO(const QString &group, const QString &name, int value)
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return ::writeDO(group.toLatin1().constData(), name.toLatin1().constData(),
                     value, ipStd.c_str());
}

double LancetRobotArm::readAI(const QString &group, const QString &name) const
{
    Q_D(const LancetRobotArm);
    int mode = 0;
    double value = 0.0;
    std::string ipStd = d->m_ip.toStdString();
    ::readAI(group.toLatin1().constData(), name.toLatin1().constData(),
             &mode, &value, ipStd.c_str());
    return value;
}

int LancetRobotArm::writeAO(const QString &group, const QString &name,
                              int mode, double value)
{
    Q_D(LancetRobotArm);
    std::string ipStd = d->m_ip.toStdString();
    return ::writeAO(group.toLatin1().constData(), name.toLatin1().constData(),
                     mode, value, ipStd.c_str());
}

// =====================================================================
// 信息
// =====================================================================

QString LancetRobotArm::errorString(int errorCode)
{
    const char *raw = formatError(errorCode);
    return raw ? QString::fromUtf8(raw) : QString("Unknown error %1").arg(errorCode);
}

int LancetRobotArm::jointCount() const
{
    Q_D(const LancetRobotArm);
    return d->m_jointCount;
}

QString LancetRobotArm::ipAddress() const
{
    Q_D(const LancetRobotArm);
    return d->m_ip;
}

// =====================================================================
// 内部方法（供 LancetRobotPath 使用）
// =====================================================================

void LancetRobotArm::notifyPathStarted()
{
    Q_D(LancetRobotArm);
    d->doTransition(LancetRobotStateMachine::Event::MoveIssued);
}
