# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Qt 5.12.10 C++ wrapper for the Siling (思灵) Diana industrial robot arm C API (V2.19.0), used in **medical surgical navigation**. Patient safety is the top priority.

## Architecture

```
src/robot/
  lancetrobottypes.h            - All public types, enums, constants (LancetJoints, LancetPose, RobotState, etc.)
  lancetrobotstatemachine.h/cpp - State machine (11 states, event-driven transitions, non-QObject)
  lancetrobotsafety.h/cpp       - Medical safety guard (pre-validation + runtime force/torque monitoring)
  lancetrobotarm.h/cpp          - Main class (QObject, PIMPL, wraps DianaApi.h C functions)
  lancetrobotpath.h             - Path RAII helper (header-only, friend of LancetRobotArm)
```

**Key design decisions:**
- State machine enforces single-actor model: motion commands rejected unless Idle (returns error code, caller must `stop()` first)
- Motion commands are async; `waitForIdle(ms)` blocks via QWaitCondition
- All errors returned as int (0 = success, negative = error code from Diana API or LancetRobotError namespace)
- Unit conversion at API boundary: external uses mm/degrees (configurable via `LancetUnitConfig`), internal always m/rad
- Safety layer (`LancetRobotSafety`) validates every motion command pre-dispatch and monitors force/torque/workspace every frame
- C API callbacks dispatched via static IP→instance registry to correct `LancetRobotArmPrivate`
- PIMPL hides `DianaApi.h` dependency from public headers

## Conventions

- **File naming**: all files prefixed with `lancet` (lowercase), e.g. `lancetrobotarm.h`
- **Code style**: Qt conventions — camelCase methods, `m_` member prefix, `Q_DECLARE_PRIVATE`/`Q_D`/`Q_Q` macros
- **Documentation**: Qt-style doc comments (`/*! \brief ... */`, `\param`, `\return`, `\note`, `\warning`)
- **Types**: `QVector<double>` via typedefs (`LancetJoints`, `LancetPose`), not `std::vector` or raw arrays
- **Thread safety**: `QMutex` for state transitions, `QReadWriteLock` for cached status, `QWaitCondition` for idle wait
- **Signals over callbacks**: public notification via Qt signals (`stateChanged`, `errorOccurred`, `safetyViolation`, `statusUpdated`)

## Unit Conversion

External (public API) defaults to **mm + degrees**. Internal (C API calls) always uses **m + rad**.
- `LancetPose`: elements 0-2 use length factor, elements 3-5 use angle factor
- `LancetJoints`: all elements use angle factor
- Force/torque (N, N·m) and current (A) are never converted
- Safety config limits are specified in external units; converted to internal on `setSafetyConfig()`

## State Machine States

Disconnected → Idle → Moving / SpeedControl / ServoControl / FreeDriving / ForceMode / ImpedanceMode / RescueMode / Paused → back to Idle. Any connected state can transition to Error (on error callback) or Disconnected.

## Safety (Medical Critical)

- Pre-dispatch: velocity/acceleration hard limits, workspace boundary check
- Runtime: TCP force, joint torque, workspace boundary — violation triggers `emergencyStop()` (force Error state)
- Audit logging via `Q_LOGGING_CATEGORY(lcRobotSafety, "lancet.robot.safety")`
- `emergencyStop()` bypasses state machine, directly forces Error state

## Dependencies

- Qt 5.12.10 (Core module: QObject, QVector, QMutex, QReadWriteLock, QWaitCondition, QLoggingCategory)
- Diana robot C API SDK (`DianaApi.h`) — must configure INCLUDEPATH and LIBS in .pro file
- API manual: `Doc/思灵机器人工业版API手册(C&C++版) V2.19.0.pdf`
