/*!
    \file lancetrobottypes.h
    \brief 机械臂封装库的公共类型、枚举与数据结构定义。

    本文件为整个 lancet robot 封装层的基础头文件，所有其他模块均依赖此文件。
    类型定义遵循 Qt 5.12 风格，使用 QVector<double> 作为动态数组载体。
*/

#ifndef LANCETROBOTTYPES_H
#define LANCETROBOTTYPES_H

#include <QMetaType>
#include <QString>
#include <QVector>

#include <cmath>

// =====================================================================
// 基础类型别名
// =====================================================================

/*!
    \typedef LancetJoints
    \brief 关节角度/速度/力矩数组，长度等于 jointCount()。
*/
typedef QVector<double> LancetJoints;

/*!
    \typedef LancetPose
    \brief 末端位姿，固定 6 个元素：[x, y, z, rx, ry, rz]。

    前 3 个元素为平移量（长度单位），后 3 个为旋转量（角度单位）。
    具体单位取决于 LancetUnitConfig 的设置。
*/
typedef QVector<double> LancetPose;

/*!
    \typedef LancetMatrix4x4
    \brief 4×4 齐次变换矩阵，列优先存储，共 16 个元素。
*/
typedef QVector<double> LancetMatrix4x4;

/*!
    \typedef LancetJacobian
    \brief 雅可比矩阵，大小为 6 × jointCount。
*/
typedef QVector<double> LancetJacobian;

// =====================================================================
// 单位配置
// =====================================================================

/*!
    \enum LengthUnit
    \brief 外部接口使用的长度单位。
*/
enum class LengthUnit {
    Meter,       ///< 米（机械臂原生单位）
    Millimeter   ///< 毫米（默认，医疗场景常用）
};

/*!
    \enum AngleUnit
    \brief 外部接口使用的角度单位。
*/
enum class AngleUnit {
    Radian,      ///< 弧度（机械臂原生单位）
    Degree       ///< 度（默认，直觉友好）
};

/*!
    \struct LancetUnitConfig
    \brief 外部接口的单位系统配置。

    设置后，LancetRobotArm 所有公共方法的位置、速度、加速度参数
    均使用此单位；内部自动转换为机械臂原生单位（m / rad）后调用 C API。
    查询返回值也会自动转换回外部单位。

    \note LancetSafetyConfig 中的限值同样使用此单位。
*/
struct LancetUnitConfig {
    LengthUnit length = LengthUnit::Millimeter;  ///< 长度单位，默认 mm
    AngleUnit  angle  = AngleUnit::Degree;        ///< 角度单位，默认度

    /*! \brief 长度 外部→内部 缩放因子。 */
    double lengthToInternal() const
    {
        return (length == LengthUnit::Millimeter) ? 0.001 : 1.0;
    }

    /*! \brief 长度 内部→外部 缩放因子。 */
    double lengthToExternal() const
    {
        return (length == LengthUnit::Millimeter) ? 1000.0 : 1.0;
    }

    /*! \brief 角度 外部→内部 缩放因子。 */
    double angleToInternal() const
    {
        return (angle == AngleUnit::Degree) ? (M_PI / 180.0) : 1.0;
    }

    /*! \brief 角度 内部→外部 缩放因子。 */
    double angleToExternal() const
    {
        return (angle == AngleUnit::Degree) ? (180.0 / M_PI) : 1.0;
    }
};

// =====================================================================
// 状态枚举
// =====================================================================

/*!
    \enum RobotState
    \brief 机械臂状态机的所有合法状态。

    \value Disconnected  未建立网络连接
    \value Idle          已连接且空闲，可接受运动指令
    \value Moving        正在执行点到点或路径运动
    \value SpeedControl  速度控制模式（speedJ / speedL 持续发送）
    \value ServoControl  伺服控制模式（servoJ / servoL 高频更新）
    \value FreeDriving   拖动示教模式
    \value ForceMode     力控模式
    \value ImpedanceMode 阻抗控制模式（关节或笛卡尔）
    \value Paused        运动暂停（可 resume）
    \value Error         错误状态，需调用 clearError() 恢复
    \value RescueMode    救援模式（手动操作脱困）
*/
enum class RobotState {
    Disconnected,
    Idle,
    Moving,
    SpeedControl,
    ServoControl,
    FreeDriving,
    ForceMode,
    ImpedanceMode,
    Paused,
    Error,
    RescueMode
};

Q_DECLARE_METATYPE(RobotState)

// =====================================================================
// 运动方向 / 坐标系 / 控制模式
// =====================================================================

/*!
    \enum MoveDirection
    \brief TCP 点动或旋转方向。
*/
enum class MoveDirection {
    XPos, XNeg,
    YPos, YNeg,
    ZPos, ZNeg
};

/*!
    \enum CoordinateFrame
    \brief 运动参考坐标系。
*/
enum class CoordinateFrame {
    Base,       ///< 基坐标系
    Tool,       ///< 工具 / TCP 坐标系
    WorkPiece,  ///< 工件坐标系
    View        ///< 视角坐标系
};

/*!
    \enum ControlMode
    \brief 机械臂控制模式。
*/
enum class ControlMode {
    Position,         ///< 位置控制（默认）
    JointImpedance,   ///< 关节阻抗控制
    CartImpedance     ///< 笛卡尔阻抗控制
};

/*!
    \enum FreeDriveMode
    \brief 拖动示教模式选项。
*/
enum class FreeDriveMode {
    Normal         = 1,  ///< 普通拖动
    BrakeReleased  = 2   ///< 释放制动器（慎用）
};

// =====================================================================
// 运动参数
// =====================================================================

/*!
    \struct LancetMotionParams
    \brief 运动指令的参数集合。

    速度和加速度的单位由当前 LancetUnitConfig 决定：
    - 关节运动：角度单位/s、角度单位/s²
    - 笛卡尔运动：长度单位/s、长度单位/s²

    \warning 超出安全层硬上限的参数将被拒绝。
*/
struct LancetMotionParams {
    double velocity      = 0.0;  ///< 运动速度
    double acceleration  = 0.0;  ///< 加速度
    int    zvOrder       = 0;    ///< 输入整形阶次（0 = 禁用，1~2 = 启用）
    double zvFrequency   = 0.0;  ///< 输入整形频率（Hz）
    double zvDamping     = 0.0;  ///< 输入整形阻尼比（0~1）
    bool   avoidSingular = false;///< 是否启用奇异值回避（moveL 有效）
};

// =====================================================================
// 力控参数
// =====================================================================

/*!
    \struct LancetForceParams
    \brief 力控模式的参数集合。

    力值和速度限制的单位恒为 N 和 m/s（不受 LancetUnitConfig 影响），
    因为力控是底层物理量，不适合做单位映射。
*/
struct LancetForceParams {
    int              frameType      = 0;    ///< 参考坐标系类型
    LancetMatrix4x4  frameMatrix;           ///< 坐标系变换矩阵（16 元素）
    QVector<double>  forceDirection;        ///< 力方向向量（3 元素）
    double           forceValue     = 0.0;  ///< 目标力值（N）
    double           maxVelocity    = 0.0;  ///< 最大速度限制（m/s）
    double           maxOffset      = 0.0;  ///< 最大位置偏移（m）
};

// =====================================================================
// 实时状态快照
// =====================================================================

/*!
    \struct LancetRobotStatus
    \brief 机械臂一帧状态的完整快照。

    由 C API 状态回调线程填充，通过 QReadWriteLock 保护读写。
    其中的位置、速度、力矩等值均为机械臂原生单位（m / rad / N / N·m）。
    通过 LancetRobotArm 的公共查询方法获取时会自动转换为外部单位。
*/
struct LancetRobotStatus {
    RobotState   state             = RobotState::Disconnected;
    LancetJoints jointPositions;   ///< 关节位置（rad）
    LancetJoints jointVelocities;  ///< 关节角速度（rad/s）
    LancetJoints jointTorques;     ///< 关节力矩（N·m）
    LancetJoints jointCurrents;    ///< 关节电流（A）
    LancetPose   tcpPose;          ///< TCP 位姿 [x,y,z,rx,ry,rz]（m / rad）
    double       tcpExternalForce  = 0.0;  ///< TCP 外部合力（N）
    bool         collision         = false;///< 碰撞检测标志
    qint64       timestamp         = 0;    ///< 时间戳（ms since epoch）
};

Q_DECLARE_METATYPE(LancetRobotStatus)

// =====================================================================
// 安全配置
// =====================================================================

/*!
    \struct LancetSafetyConfig
    \brief 医疗级安全参数配置。

    指令预校验硬上限和工作空间边界使用与 LancetUnitConfig 相同的外部单位。
    力 / 力矩单位恒为 N / N·m，不受单位配置影响。

    \warning 修改此配置需要机械臂处于 Idle 或 Disconnected 状态。
*/
struct LancetSafetyConfig {
    // ---- 指令预校验硬上限（外部单位） ----
    double maxJointVelocity     = 57.3;    ///< 关节最大速度（度/s ≈ 1 rad/s）
    double maxCartVelocity      = 200.0;   ///< TCP 最大平移速度（mm/s）
    double maxCartRotVelocity   = 28.6;    ///< TCP 最大旋转速度（度/s）
    double maxJointAcc          = 114.6;   ///< 关节最大加速度（度/s²）
    double maxCartAcc           = 1000.0;  ///< TCP 最大平移加速度（mm/s²）

    // ---- 运行时力/力矩监控（超限急停，单位恒为 N / N·m） ----
    double       maxTcpForceN   = 50.0;    ///< TCP 最大外力（N），0 = 不监控
    LancetJoints maxJointTorqueNm;         ///< 各关节最大力矩（N·m），空 = 不监控

    // ---- 工作空间边界（外部单位，可选） ----
    bool       enableWorkspace  = false;   ///< 是否启用工作空间限制
    LancetPose workspaceMin;               ///< 工作空间最小边界 [x,y,z]
    LancetPose workspaceMax;               ///< 工作空间最大边界 [x,y,z]

    // ---- 连接看门狗 ----
    int heartbeatTimeoutMs      = 2000;    ///< 心跳超时（ms），超时触发急停
};

// =====================================================================
// 错误码常量
// =====================================================================

namespace LancetRobotError {

    // ---- 自定义封装层错误码 ----
    static const int Success          =  0;       ///< 成功
    static const int Timeout          = -1;       ///< 操作超时
    static const int InvalidParam     = -2;       ///< 参数无效
    static const int SafetyRejected   = -3;       ///< 安全层拒绝
    static const int JointCountMismatch = -4;     ///< 关节数不匹配

    // ---- 转发自 Diana C API 的典型错误码 ----
    static const int NotConnected     = -1013;    ///< IP 未注册 / 未连接
    static const int CallingConflict  = -2201;    ///< 运动指令冲突
    static const int CollisionDetected= -2202;    ///< 碰撞检测触发
    static const int FreeDrivingBusy  = -2210;    ///< 拖动模式中不可运动
    static const int ConflictTask     = -2215;    ///< 任务冲突
    static const int FatalError       = -2307;    ///< 致命错误

} // namespace LancetRobotError

#endif // LANCETROBOTTYPES_H
