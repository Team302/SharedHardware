
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#include <string>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/drive/DifferentialDrive.h>

#include <chassis/differential/DifferentialChassis.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

using namespace std;
using namespace ctre::phoenix::motorcontrol::can;

DifferentialChassis::DifferentialChassis
(
    shared_ptr<IDragonMotorController>             leftMotor, 
    shared_ptr<IDragonMotorController>             rightMotor,
    units::meter_t                                 trackWidth,
    units::velocity::meters_per_second_t           maxSpeed,
    units::angular_velocity::degrees_per_second_t  maxAngSpeed,
    units::length::inch_t                          wheelDiameter,
    string                                         networktablename,
    string                                         controlfilename 
) : m_leftMotor(leftMotor),
    m_rightMotor(rightMotor),
    m_maxSpeed(maxSpeed),
    m_maxAngSpeed(maxAngSpeed),
    m_wheelDiameter(wheelDiameter),
    m_track(trackWidth),
    m_kinematics(new frc::DifferentialDriveKinematics(trackWidth)),
    m_differentialOdometry(new frc::DifferentialDriveOdometry(frc::Rotation2d(), frc::Pose2d())),
    m_controlFile(controlfilename),
    m_ntName(networktablename)
{

}

IChassis::CHASSIS_TYPE DifferentialChassis::GetType() const
{
    return IChassis::CHASSIS_TYPE::DIFFERENTIAL;
}

void DifferentialChassis::Drive
(
    frc::ChassisSpeeds            chassisSpeeds,
    IChassis::CHASSIS_DRIVE_MODE  mode,
    IChassis::HEADING_OPTION      headingOption
) 
{
    Drive(chassisSpeeds);
}

//Moves the robot
void DifferentialChassis::Drive(frc::ChassisSpeeds chassisSpeeds)
{
    auto wheels = m_kinematics->ToWheelSpeeds(chassisSpeeds);
    wheels.Desaturate(m_maxSpeed);
    if (m_leftMotor.get() != nullptr)
    {
        m_leftMotor.get()->Set(wheels.left/m_maxSpeed);
    }
    if (m_rightMotor.get() != nullptr)
    {
        m_rightMotor.get()->Set(wheels.right/m_maxSpeed);
    }
}

frc::Pose2d DifferentialChassis::GetPose() const
{
    return frc::Pose2d();
}

void DifferentialChassis::ResetPose(const frc::Pose2d& pose)
{

}

void DifferentialChassis::UpdateOdometry()
{
    
}

units::velocity::meters_per_second_t DifferentialChassis::GetMaxSpeed() const
{
    return m_maxSpeed;
}

units::angular_velocity::radians_per_second_t DifferentialChassis::GetMaxAngularSpeed() const
{
    return m_maxAngSpeed;
}

units::length::inch_t DifferentialChassis::GetWheelDiameter() const
{
    return units::length::inch_t(4);
}    

units::length::inch_t DifferentialChassis::GetTrack() const
{
    return m_track;
}

units::angle::degree_t DifferentialChassis::GetYaw() const
{
    units::degree_t yaw{0.0}; // get from pigeon
    return yaw;
}

void DifferentialChassis::SetEncodersToZero()
{
    if (m_leftMotor.get() != nullptr)
    {
        auto motor = m_leftMotor.get()->GetSpeedController();
        auto fx = dynamic_cast<WPI_TalonFX*>(motor.get());
        auto driveMotorSensors = fx->GetSensorCollection();
        driveMotorSensors.SetIntegratedSensorPosition(0, 0);
    }
    if (m_rightMotor.get() != nullptr)
    {
        auto motor = m_rightMotor.get()->GetSpeedController();
        auto fx = dynamic_cast<WPI_TalonFX*>(motor.get());
        auto driveMotorSensors = fx->GetSensorCollection();
        driveMotorSensors.SetIntegratedSensorPosition(0, 0);
    }
}
void DifferentialChassis::SetTargetHeading(units::angle::degree_t targetYaw) 
{
}

