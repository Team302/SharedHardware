
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


#pragma once
#include <string>

#include <units/velocity.h>
#include <units/angular_velocity.h>

#include <chassis/IChassis.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/drive/DifferentialDrive.h>

class DifferentialChassis : public IChassis {

    public:
        DifferentialChassis() = delete;
        virtual ~DifferentialChassis() = default;

        DifferentialChassis(std::shared_ptr<IDragonMotorController>         leftMotor, 
                            std::shared_ptr<IDragonMotorController>         rightMotor,
                            units::meter_t                                  trackWidth,
                            units::velocity::meters_per_second_t            maxSpeed,
                            units::angular_velocity::degrees_per_second_t   maxAngSpeed,
                            units::length::inch_t                           wheelDiameter,
                            std::string                                     networktablename,
                            std::string                                     controlfilename);

        IChassis::CHASSIS_TYPE GetType() const override;
        void Drive(frc::ChassisSpeeds chassisSpeeds);
        void Drive
        (
            frc::ChassisSpeeds            chassisSpeeds,
            IChassis::CHASSIS_DRIVE_MODE  mode,
            IChassis::HEADING_OPTION      headingOption
        ) override;
        
        inline void Initialize() override {};
        frc::Pose2d GetPose() const override;
        void ResetPose
        (
            const frc::Pose2d&      pose
        ) override;
        void UpdateOdometry() override;
        units::velocity::meters_per_second_t GetMaxSpeed() const override;
        units::angular_velocity::radians_per_second_t GetMaxAngularSpeed() const override;
        units::length::inch_t GetWheelDiameter() const override ;
        units::length::inch_t GetTrack() const override;
        units::angle::degree_t GetYaw() const override;
        void SetTargetHeading(units::angle::degree_t targetYaw) override;
        void SetEncodersToZero() override;





    private:
        std::shared_ptr<IDragonMotorController>         m_leftMotor;
        std::shared_ptr<IDragonMotorController>         m_rightMotor;
        
        units::velocity::meters_per_second_t            m_maxSpeed;
        units::angular_velocity::degrees_per_second_t   m_maxAngSpeed;
        units::length::inch_t                           m_wheelDiameter;
        units::length::inch_t                           m_track;

        frc::DifferentialDriveKinematics*               m_kinematics;
        frc::DifferentialDriveOdometry*                 m_differentialOdometry;
        std::string                                     m_controlFile;
        std::string                                     m_ntName;

};