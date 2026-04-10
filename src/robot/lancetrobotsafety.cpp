/*!
    \file lancetrobotsafety.cpp
    \brief 医疗安全防护层实现。
*/

#include "lancetrobotsafety.h"

#include <QtMath>

Q_LOGGING_CATEGORY(lcRobotSafety, "lancet.robot.safety")

// =====================================================================
// 构造
// =====================================================================

LancetRobotSafety::LancetRobotSafety(QObject *parent)
    : QObject(parent)
{
}

// =====================================================================
// 配置
// =====================================================================

void LancetRobotSafety::setConfig(const LancetSafetyConfig &cfg,
                                   const LancetUnitConfig &unitCfg)
{
    m_externalConfig = cfg;

    const double lf = unitCfg.lengthToInternal();   // mm→m 等
    const double af = unitCfg.angleToInternal();     // deg→rad 等

    m_maxJointVelRad   = cfg.maxJointVelocity   * af;
    m_maxCartVelMs     = cfg.maxCartVelocity     * lf;
    m_maxCartRotVelRad = cfg.maxCartRotVelocity  * af;
    m_maxJointAccRad   = cfg.maxJointAcc         * af;
    m_maxCartAccMs     = cfg.maxCartAcc          * lf;

    // 力 / 力矩单位恒为 N / N·m，不转换
    m_maxTcpForceN       = cfg.maxTcpForceN;
    m_maxJointTorqueNm   = cfg.maxJointTorqueNm;

    // 工作空间边界转换
    m_enableWorkspace = cfg.enableWorkspace;
    if (m_enableWorkspace) {
        m_workspaceMinM.resize(qMin(cfg.workspaceMin.size(), 6));
        m_workspaceMaxM.resize(qMin(cfg.workspaceMax.size(), 6));
        for (int i = 0; i < m_workspaceMinM.size(); ++i) {
            double factor = (i < 3) ? lf : af;
            m_workspaceMinM[i] = cfg.workspaceMin[i] * factor;
            m_workspaceMaxM[i] = cfg.workspaceMax[i] * factor;
        }
    }

    qCInfo(lcRobotSafety) << "Safety config updated:"
                          << "maxJointVel=" << m_maxJointVelRad << "rad/s"
                          << "maxCartVel=" << m_maxCartVelMs << "m/s"
                          << "maxTcpForce=" << m_maxTcpForceN << "N"
                          << "workspace=" << m_enableWorkspace;
}

LancetSafetyConfig LancetRobotSafety::config() const
{
    return m_externalConfig;
}

// =====================================================================
// 指令预校验
// =====================================================================

bool LancetRobotSafety::validateJointMotion(double velocity, double acceleration,
                                             QString *reason) const
{
    if (qAbs(velocity) > m_maxJointVelRad) {
        QString msg = QString("Joint velocity %1 rad/s exceeds limit %2 rad/s")
                          .arg(velocity, 0, 'f', 4)
                          .arg(m_maxJointVelRad, 0, 'f', 4);
        qCWarning(lcRobotSafety) << msg;
        if (reason) *reason = msg;
        return false;
    }

    if (qAbs(acceleration) > m_maxJointAccRad) {
        QString msg = QString("Joint acceleration %1 rad/s^2 exceeds limit %2 rad/s^2")
                          .arg(acceleration, 0, 'f', 4)
                          .arg(m_maxJointAccRad, 0, 'f', 4);
        qCWarning(lcRobotSafety) << msg;
        if (reason) *reason = msg;
        return false;
    }

    return true;
}

bool LancetRobotSafety::validateCartMotion(double velocity, double acceleration,
                                            QString *reason) const
{
    if (qAbs(velocity) > m_maxCartVelMs) {
        QString msg = QString("Cartesian velocity %1 m/s exceeds limit %2 m/s")
                          .arg(velocity, 0, 'f', 6)
                          .arg(m_maxCartVelMs, 0, 'f', 6);
        qCWarning(lcRobotSafety) << msg;
        if (reason) *reason = msg;
        return false;
    }

    if (qAbs(acceleration) > m_maxCartAccMs) {
        QString msg = QString("Cartesian acceleration %1 m/s^2 exceeds limit %2 m/s^2")
                          .arg(acceleration, 0, 'f', 6)
                          .arg(m_maxCartAccMs, 0, 'f', 6);
        qCWarning(lcRobotSafety) << msg;
        if (reason) *reason = msg;
        return false;
    }

    return true;
}

bool LancetRobotSafety::validatePoseInWorkspace(const LancetPose &pose,
                                                  QString *reason) const
{
    if (!m_enableWorkspace)
        return true;

    // 仅检查前 3 个元素（平移 x, y, z）
    const int dims = qMin(3, qMin(pose.size(), qMin(m_workspaceMinM.size(),
                                                     m_workspaceMaxM.size())));
    static const char *axisNames[] = {"X", "Y", "Z"};

    for (int i = 0; i < dims; ++i) {
        if (pose[i] < m_workspaceMinM[i] || pose[i] > m_workspaceMaxM[i]) {
            QString msg = QString("Pose %1=%2 m out of workspace [%3, %4] m")
                              .arg(axisNames[i])
                              .arg(pose[i], 0, 'f', 6)
                              .arg(m_workspaceMinM[i], 0, 'f', 6)
                              .arg(m_workspaceMaxM[i], 0, 'f', 6);
            qCWarning(lcRobotSafety) << msg;
            if (reason) *reason = msg;
            return false;
        }
    }

    return true;
}

// =====================================================================
// 运行时监控
// =====================================================================

bool LancetRobotSafety::checkRuntimeStatus(const LancetRobotStatus &status,
                                             QString *reason)
{
    // ---- 碰撞检测 ----
    if (status.collision) {
        QString msg = QStringLiteral("Collision detected by robot firmware");
        qCCritical(lcRobotSafety) << msg;
        if (reason) *reason = msg;
        emit violationDetected(msg, true);
        return false;
    }

    // ---- TCP 外力监控 ----
    if (m_maxTcpForceN > 0.0 && status.tcpExternalForce > m_maxTcpForceN) {
        QString msg = QString("TCP external force %1 N exceeds limit %2 N")
                          .arg(status.tcpExternalForce, 0, 'f', 2)
                          .arg(m_maxTcpForceN, 0, 'f', 2);
        qCCritical(lcRobotSafety) << msg;
        if (reason) *reason = msg;
        emit violationDetected(msg, true);
        return false;
    }

    // ---- 关节力矩监控 ----
    if (!m_maxJointTorqueNm.isEmpty()) {
        const int count = qMin(status.jointTorques.size(), m_maxJointTorqueNm.size());
        for (int i = 0; i < count; ++i) {
            if (qAbs(status.jointTorques[i]) > m_maxJointTorqueNm[i]) {
                QString msg = QString("Joint %1 torque %2 Nm exceeds limit %3 Nm")
                                  .arg(i)
                                  .arg(status.jointTorques[i], 0, 'f', 2)
                                  .arg(m_maxJointTorqueNm[i], 0, 'f', 2);
                qCCritical(lcRobotSafety) << msg;
                if (reason) *reason = msg;
                emit violationDetected(msg, true);
                return false;
            }
        }
    }

    // ---- 工作空间边界监控 ----
    if (m_enableWorkspace && !status.tcpPose.isEmpty()) {
        const int dims = qMin(3, qMin(status.tcpPose.size(),
                                      qMin(m_workspaceMinM.size(), m_workspaceMaxM.size())));
        static const char *axisNames[] = {"X", "Y", "Z"};

        for (int i = 0; i < dims; ++i) {
            if (status.tcpPose[i] < m_workspaceMinM[i]
                || status.tcpPose[i] > m_workspaceMaxM[i]) {
                QString msg = QString("TCP %1=%2 m out of workspace [%3, %4] m")
                                  .arg(axisNames[i])
                                  .arg(status.tcpPose[i], 0, 'f', 6)
                                  .arg(m_workspaceMinM[i], 0, 'f', 6)
                                  .arg(m_workspaceMaxM[i], 0, 'f', 6);
                qCCritical(lcRobotSafety) << msg;
                if (reason) *reason = msg;
                emit violationDetected(msg, true);
                return false;
            }
        }
    }

    return true;
}
