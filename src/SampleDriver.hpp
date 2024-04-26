/*
 * Copyright 2024 by READY Robotics Corporation.
 * All rights reserved. No person may copy, distribute, publicly display, create derivative works
 * from or otherwise use or modify this software without first obtaining a license from the
 * READY Robotics Corporation.
 */
#pragma once

#include <device/Configuration.hpp>
#include <ral/common/JointData.pb.h>
#include <ral/common/Pose.pb.h>
#include <ral/common/Result.pb.h>
#include <ral/config/DriverConfiguration.pb.h>
#include <ral/driver/RobotDriver.hpp>
#include <ral/driver/state/RobotState.hpp>
#include <ral/io/DigitalStates.pb.h>
#include <ral/motion/MotionResult.pb.h>
#include <ral/motion/MotionStatus.pb.h>
#include <ral/state/DriverState.pb.h>
#include <ral/tool/LoadTool.pb.h>
#include <ral/tool/Payload.pb.h>
#include <ral/tool/SetTool.pb.h>
#include <ral/tool/SetToolResult.pb.h>
#include <ral/tool/TCP.pb.h>
#include <ral/util/DerivedTwist.hpp>
#include <ral2/Driver.hpp>

#include <QObject>
#include <QString>

// Forward Declarations
namespace ral {
class ContinuousMotion;
class GetAnalog;
class GetDigital;
class GetFaults;
class Jog;
class PauseMotion;
class Payload;
class Quaternion;
class ResumeMotion;
class SetAnalog;
class SetDigital;
class SetDriverMode;
class SetSpeedScale;
class StopMotion;
class ToolDesc;
class Vector3;
} // namespace ral

namespace ready {
class SampleDriver final : public ral2::Driver
{
    Q_OBJECT

public:
    SampleDriver(const device::Configuration &configuration, QObject *parent);

    void start(const ral::RequiredSignals &required_signals) final;
    void move(const ral::ContinuousMotion &motion) final;
    void setSpeed(const ral2::SpeedFactor &speed) final;
    void stopMotion(const ral::Acceleration &deceleration) final;
    void pauseMotion(const ral::Acceleration &deceleration) final;
    void resumeMotion() final;
    void loadTools(const ral::LoadTool &tools) final;
    void setTool(const ral::TCP &tcp, const ral::Payload &payload) final;
    void jogVelocity(const ral::Jog &jog) final;
    void jogIterative(const ral::SingleMotion &target) final;
    void setDigitalOutputs(const ral::SetDigital &set) final;
    void setAnalogOutputs(const ral::SetAnalog &set) final;
    void clearFaults() final;

private:
};
} // namespace ready
