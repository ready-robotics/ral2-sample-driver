/*
 * Copyright 2024 by READY Robotics Corporation.
 * All rights reserved. No person may copy, distribute, publicly display, create derivative works from or otherwise
 * use or modify this software without first obtaining a license from the READY Robotics Corporation.
 */
#include "SampleDriver.hpp"

#include <device/Configuration.hpp>
#include <forge/core/Logger.hpp>
#include <forge/core/Time.hpp>
#include <ral/common/JointLimit.pb.h>
#include <ral/config/DriverConfiguration.pb.h>
#include <ral/hand-guide/HandGuide.pb.h>
#include <ral/io/DigitalEntry.pb.h>
#include <ral/io/DigitalStates.pb.h>
#include <ral/io/IOType.pb.h>
#include <ral/io/SetAnalog.pb.h>
#include <ral/io/SetDigital.pb.h>
#include <ral/kinematics/JointPositionToPose.pb.h>
#include <ral/kinematics/PoseToJointPosition.pb.h>
#include <ral/motion/ContinuousMotion.pb.h>
#include <ral/motion/Jog.pb.h>
#include <ral/motion/PauseMotion.pb.h>
#include <ral/motion/ResumeMotion.pb.h>
#include <ral/motion/SetDriverMode.pb.h>
#include <ral/motion/SetSpeedScale.pb.h>
#include <ral/motion/SingleMotion.pb.h>
#include <ral/state/CartesianState.pb.h>
#include <ral/state/DriverState.pb.h>
#include <ral/state/JointState.pb.h>
#include <ral/state/State.pb.h>
#include <ral/tool/LoadTool.pb.h>
#include <ral2/Attributes.hpp>
#include <ral2/Driver.hpp>
#include <ral2/Fault.hpp>

#include <QTimer>

static ral::Pose identityPose()
{
    ral::Pose result;

    // No translation
    result.mutable_translation_mm()->add_data(0.0);
    result.mutable_translation_mm()->add_data(0.0);
    result.mutable_translation_mm()->add_data(0.0);

    // Identity quaternion
    result.mutable_rotation()->add_data(0.0);
    result.mutable_rotation()->add_data(0.0);
    result.mutable_rotation()->add_data(0.0);
    result.mutable_rotation()->add_data(1.0);

    result.set_config("");

    return result;
}

static ral::Twist zeroTwist()
{
    ral::Twist result;

    // No translation velocity
    result.mutable_linear_mm_per_s()->add_data(0.0);
    result.mutable_linear_mm_per_s()->add_data(0.0);
    result.mutable_linear_mm_per_s()->add_data(0.0);

    // No rotation velocity
    result.mutable_angular_rad_per_s()->add_data(0.0);
    result.mutable_angular_rad_per_s()->add_data(0.0);
    result.mutable_angular_rad_per_s()->add_data(0.0);

    return result;
}

static ral::JointData zeroJointData()
{
    ral::JointData result;

    // No translation
    result.add_data(0.0);
    result.add_data(0.0);
    result.add_data(0.0);
    result.add_data(0.0);
    result.add_data(0.0);
    result.add_data(0.0);

    return result;
}

namespace ready {
SampleDriver::SampleDriver(const device::Configuration &configuration, QObject *parent) : Driver(configuration, parent)
{}

void SampleDriver::start(const ral::RequiredSignals & /*unused */)
{
    static constexpr int NUM_JOINTS = 6;
    static constexpr auto DIGITAL_INPUT_KEY = "DIN";
    static constexpr auto DIGITAL_OUTPUT_KEY = "DOUT";
    static constexpr int NUM_DIGITAL_INPUT = 6;
    static constexpr int NUM_DIGITAL_OUTPUT = 6;

    const auto now = forge::common::utcTime();

    ral::CartesianState cartesian_state;
    *cartesian_state.mutable_timestamp() = now;
    *cartesian_state.mutable_pose() = identityPose();
    *cartesian_state.mutable_velocity() = zeroTwist();

    ral::JointState joint_state;
    *joint_state.mutable_timestamp() = now;
    *joint_state.mutable_position() = zeroJointData();
    *joint_state.mutable_velocity() = zeroJointData();
    *joint_state.mutable_effort() = zeroJointData();
    emit armStateChanged(cartesian_state, joint_state);

    ral::DriverState driver_state;
    *driver_state.mutable_timestamp() = now;
    driver_state.set_operational_mode(ral::State::MANUAL_REDUCED);
    driver_state.set_control_mode(ral::State::REMOTE);
    *driver_state.mutable_guide_mode() = {};
    driver_state.set_motion_possible(true);
    driver_state.set_in_motion(false);
    driver_state.set_enabling_switch_held(true);
    driver_state.set_can_run(true);
    driver_state.set_fence_closed(true);
    driver_state.mutable_speed()->set_speed_factor(1.0);
    driver_state.set_safety_override(ral::State::NONE);
    emit controllerStateChanged(driver_state);

    ral::DriverConfiguration driver_config;
    driver_config.set_controller_model("RCU");
    driver_config.set_robot_model("READY-5.0");

    // io_configurationuration
    auto &io_configuration = *driver_config.mutable_io_configuration();
    static constexpr auto set_digital_io =
        +[](const char *key, ral::IOType io_type, int count, ral::DigitalEntry &signal) {
            signal.mutable_signal()->set_key(key);
            signal.mutable_signal()->set_type(io_type);
            signal.mutable_signal()->set_count(count);
            signal.mutable_signal()->set_start_index(1);
        };
    set_digital_io(DIGITAL_INPUT_KEY,
                   ral::IOType::INPUT,
                   NUM_DIGITAL_INPUT,
                   *io_configuration.mutable_digital()->Add());
    set_digital_io(DIGITAL_OUTPUT_KEY,
                   ral::IOType::OUTPUT,
                   NUM_DIGITAL_OUTPUT,
                   *io_configuration.mutable_digital()->Add());
    // analog null
    io_configuration.set_mismatch(false);

    // joint_limits
    auto &joint_limits = *driver_config.mutable_joint_limits();
    joint_limits.Reserve(NUM_JOINTS);
    static constexpr auto set_joint_limit = +[](const char *name, ral::JointLimit &limit) {
        // TODO: Update limits, ranges
        limit.set_name(name);
        limit.set_min(-M_PI);
        limit.set_max(M_PI);
        limit.set_type(ral::JointLimit::ROTATIONAL);
        // alias null
    };
    set_joint_limit("J1", *joint_limits.Add());
    set_joint_limit("J2", *joint_limits.Add());
    set_joint_limit("J3", *joint_limits.Add());
    set_joint_limit("J4", *joint_limits.Add());
    set_joint_limit("J5", *joint_limits.Add());
    set_joint_limit("J6", *joint_limits.Add());

    // Unlimited payloads (mass, com) and TCPs
    driver_config.set_max_num_payloads(0);
    driver_config.set_max_num_tcps(0);
    driver_config.set_max_num_tools(0);

    // Single robot
    driver_config.set_num_ctrl_robots(1);
    driver_config.set_num_ext_axes(0);

    // Standard world orientation
    auto &robot_orientation = *driver_config.mutable_robot_orientation();
    robot_orientation.add_data(0);
    robot_orientation.add_data(0);
    robot_orientation.add_data(1);

    driver_config.set_keyless_override(ral::DriverConfiguration::USE_CONFIG);

    driver_config.set_use_internal_id_io(false);
    emit ready(driver_config);
}

void SampleDriver::move(const ral::ContinuousMotion &motion)
{
    static constexpr int DELAYED_RESULT_MS = 2000;
    (void)motion;
    QTimer::singleShot(DELAYED_RESULT_MS,
                       [this] { emit moveResult(RequestResult::SUCCEEDED, identityPose(), zeroJointData()); });
}

void SampleDriver::stopMotion(const ral::Acceleration &deceleration)
{
    (void)deceleration;
    emit stopMotionResult(RequestResult::SUCCEEDED);
}

void SampleDriver::pauseMotion(const ral::Acceleration &deceleration)
{
    (void)deceleration;
    emit pauseMotionResult(RequestResult::SUCCEEDED, identityPose(), zeroJointData());
}

void SampleDriver::resumeMotion()
{
    emit resumeMotionResult(RequestResult::SUCCEEDED);
}

void SampleDriver::loadTools(const ral::LoadTool &tools)
{
    emit loadToolsResult(RequestResult::SUCCEEDED,
                         tools.active_tool().tcp(),
                         tools.active_tool().payload(),
                         identityPose(),
                         zeroJointData());
}

void SampleDriver::setTool(const ral::TCP &tcp, const ral::Payload &payload)
{
    (void)tcp;
    (void)payload;
    emit setToolResult(RequestResult::SUCCEEDED, identityPose(), zeroJointData());
}

void SampleDriver::setSpeed(const ral2::SpeedFactor &speed)
{
    (void)speed;
    emit setSpeedResult(RequestResult::SUCCEEDED);
}

void SampleDriver::jogVelocity(const ral::Jog &jog)
{
    (void)jog;
}

void SampleDriver::jogIterative(const ral::SingleMotion &target)
{
    (void)target;
}

void SampleDriver::clearFaults()
{
    emit clearFaultsResult(RequestResult::SUCCEEDED);
}

void SampleDriver::setAnalogOutputs(const ral::SetAnalog &set)
{
    (void)set;
    emit setAnalogOutputsResult({});
}

void SampleDriver::setDigitalOutputs(const ral::SetDigital &set)
{
    (void)set;
    emit setDigitalOutputsResult({});
}
} // namespace ready
