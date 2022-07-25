
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

// C++ Includes

// FRC includes
#include <wpi/array.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

// Team 302 includes
#include <chassis/ISwerveChassisModuleStates.h>

// Third Party Includes

namespace frc
{
    struct ChassisSpeeds;
    struct SwerveModuleState;
}

class DragonPigeon;

class EtherDirtySwerve : public ISwerveChassisModuleStates
{
	public:

        wpi::array<frc::SwerveModuleState, 4> CalcModuleStates
        (
            frc::ChassisSpeeds                      speeds
        ) override;


        EtherDirtySwerve() = delete;
        EtherDirtySwerve
        (
            units::length::meter_t                  wheelBase,
            units::length::meter_t                  wheelTrack,
            units::velocity::meters_per_second_t    maxSpeed
        );
        virtual ~EtherDirtySwerve() = default;

    protected:
        wpi::array<frc::SwerveModuleState, 4> CalcSwerveModuleStates
        (
            frc::ChassisSpeeds speeds
        );
        units::length::meter_t                  m_wheelBase;
        units::length::meter_t                  m_wheelTrack;
        units::velocity::meters_per_second_t    m_maxSpeed;
};



