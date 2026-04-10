/*!
    \file lancetrobotpath.h
    \brief 机械臂路径规划 RAII 辅助类（header-only）。

    LancetRobotPath 封装了思灵 C API 的 createComplexPath / addMoveXByPose /
    runComplexPath / destroyComplexPath 等接口，提供链式调用和自动资源释放。

    \note 路径中添加的位姿和速度参数使用当前 LancetRobotArm 的外部单位。
*/

#ifndef LANCETROBOTPATH_H
#define LANCETROBOTPATH_H

#include "lancetrobotarm.h"
#include "lancetrobottypes.h"

/*!
    \class LancetRobotPath
    \brief 路径规划辅助类，RAII 管理路径生命周期。

    构造时自动创建路径对象（通过 C API），析构时自动销毁。
    支持链式调用添加线性、关节和圆弧段。

    \section1 用法示例
    \code
    LancetRobotPath path(arm);
    path.addMoveLinear({300, 0, 400, 0, 180, 0}, 50.0, 300.0)
        .addMoveLinear({350, 50, 400, 0, 180, 0}, 50.0, 300.0);
    int ret = path.execute();
    arm.waitForIdle(10000);
    \endcode

    \warning execute() 要求机械臂处于 Idle 状态，否则返回冲突错误码。
    \sa LancetRobotArm
*/
class LancetRobotPath
{
public:
    /*!
        \brief 构造路径对象。
        \param arm 关联的机械臂实例

        内部调用 C API createComplexPath() 分配路径资源。
    */
    explicit LancetRobotPath(LancetRobotArm &arm);

    /*!
        \brief 析构，自动释放路径资源。

        即使未调用 execute() 也不会崩溃。
    */
    ~LancetRobotPath();

    /*!
        \brief 添加笛卡尔直线段。
        \param pose 目标位姿 [x,y,z,rx,ry,rz]（外部单位）
        \param vel  平移速度（外部长度单位/s）
        \param acc  平移加速度（外部长度单位/s²）
        \param tcpName 自定义 TCP 名称，空串使用默认
        \return *this（支持链式调用）
    */
    LancetRobotPath &addMoveLinear(const LancetPose &pose, double vel, double acc,
                                   const QString &tcpName = QString());

    /*!
        \brief 添加关节空间段。
        \param joints 目标关节角度（外部角度单位）
        \param vel    关节速度（外部角度单位/s）
        \param acc    关节加速度（外部角度单位/s²）
        \return *this
    */
    LancetRobotPath &addMoveJoint(const LancetJoints &joints, double vel, double acc);

    /*!
        \brief 添加圆弧段。
        \param pose 圆弧途经点位姿（外部单位）
        \param vel  平移速度（外部长度单位/s）
        \param acc  平移加速度（外部长度单位/s²）
        \param tcpName 自定义 TCP 名称
        \return *this
    */
    LancetRobotPath &addMoveCircular(const LancetPose &pose, double vel, double acc,
                                     const QString &tcpName = QString());

    /*!
        \brief 执行路径。

        要求机械臂处于 Idle 状态。执行后机械臂进入 Moving 状态，
        可用 LancetRobotArm::waitForIdle() 等待完成。

        \return 0 成功，负数错误码
    */
    int execute();

    /*!
        \brief 获取底层路径 ID。
        \return 路径 ID，-1 表示未创建
    */
    int pathId() const;

private:
    Q_DISABLE_COPY(LancetRobotPath)
    LancetRobotArm &m_arm;
    int m_pathId;
};

// =====================================================================
// 内联实现
// =====================================================================

// 前置声明 C API（避免在 header 中 #include "DianaApi.h"）
extern "C" {
    int createComplexPath(const char *strIpAddress);
    int addMoveLByPose(int pathId, double *pose, double v, double a,
                       const char *active_tcp, const char *strIpAddress);
    int addMoveJByTarget(int pathId, double *joints, double v, double a,
                         const char *strIpAddress);
    int addMoveCByPose(int pathId, double *pose, double v, double a,
                       const char *active_tcp, const char *strIpAddress);
    int runComplexPath(int pathId, const char *strIpAddress);
    int destroyComplexPath(int pathId, const char *strIpAddress);
}

inline LancetRobotPath::LancetRobotPath(LancetRobotArm &arm)
    : m_arm(arm)
    , m_pathId(-1)
{
    std::string ip = arm.ipAddress().toStdString();
    m_pathId = createComplexPath(ip.c_str());
}

inline LancetRobotPath::~LancetRobotPath()
{
    if (m_pathId >= 0) {
        std::string ip = m_arm.ipAddress().toStdString();
        destroyComplexPath(m_pathId, ip.c_str());
    }
}

inline LancetRobotPath &LancetRobotPath::addMoveLinear(const LancetPose &pose,
                                                         double vel, double acc,
                                                         const QString &tcpName)
{
    if (m_pathId < 0)
        return *this;

    // 单位转换：使用 arm 的 unitConfig
    LancetUnitConfig u = m_arm.unitConfig();
    double lf = u.lengthToInternal();
    double af = u.angleToInternal();

    LancetPose poseInternal = { pose[0]*lf, pose[1]*lf, pose[2]*lf,
                                pose[3]*af, pose[4]*af, pose[5]*af };
    double velMs = vel * lf;
    double accMs = acc * lf;

    std::string ip = m_arm.ipAddress().toStdString();
    QByteArray tcpBa = tcpName.toLatin1();
    const char *tcp = tcpName.isEmpty() ? nullptr : tcpBa.constData();

    addMoveLByPose(m_pathId, poseInternal.data(), velMs, accMs, tcp, ip.c_str());
    return *this;
}

inline LancetRobotPath &LancetRobotPath::addMoveJoint(const LancetJoints &joints,
                                                        double vel, double acc)
{
    if (m_pathId < 0)
        return *this;

    LancetUnitConfig u = m_arm.unitConfig();
    double af = u.angleToInternal();

    LancetJoints jointsRad(joints.size());
    for (int i = 0; i < joints.size(); ++i)
        jointsRad[i] = joints[i] * af;
    double velRad = vel * af;
    double accRad = acc * af;

    std::string ip = m_arm.ipAddress().toStdString();
    addMoveJByTarget(m_pathId, jointsRad.data(), velRad, accRad, ip.c_str());
    return *this;
}

inline LancetRobotPath &LancetRobotPath::addMoveCircular(const LancetPose &pose,
                                                           double vel, double acc,
                                                           const QString &tcpName)
{
    if (m_pathId < 0)
        return *this;

    LancetUnitConfig u = m_arm.unitConfig();
    double lf = u.lengthToInternal();
    double af = u.angleToInternal();

    LancetPose poseInternal = { pose[0]*lf, pose[1]*lf, pose[2]*lf,
                                pose[3]*af, pose[4]*af, pose[5]*af };
    double velMs = vel * lf;
    double accMs = acc * lf;

    std::string ip = m_arm.ipAddress().toStdString();
    QByteArray tcpBa = tcpName.toLatin1();
    const char *tcp = tcpName.isEmpty() ? nullptr : tcpBa.constData();

    addMoveCByPose(m_pathId, poseInternal.data(), velMs, accMs, tcp, ip.c_str());
    return *this;
}

inline int LancetRobotPath::execute()
{
    if (m_pathId < 0)
        return LancetRobotError::InvalidParam;

    if (!m_arm.isIdle())
        return LancetRobotError::CallingConflict;

    std::string ip = m_arm.ipAddress().toStdString();
    int ret = runComplexPath(m_pathId, ip.c_str());
    if (ret == 0)
        m_arm.notifyPathStarted();
    return ret;
}

inline int LancetRobotPath::pathId() const
{
    return m_pathId;
}

#endif // LANCETROBOTPATH_H
