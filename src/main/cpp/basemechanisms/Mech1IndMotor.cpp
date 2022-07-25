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

// C++ Includes
#include <memory>
#include <string>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <mechanisms/controllers/ControlData.h>
#include <basemechanisms/Mech1IndMotor.h>
#include <basemechanisms/interfaces/IMech1IndMotor.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <utils/Logger.h>

// Third Party Includes
#include <units/time.h>

using namespace std;

/// @brief Create a generic mechanism wiht 1 independent motor 
/// @param [in] MechanismTypes::MECHANISM_TYPE the type of mechansim
/// @param [in] std::string the name of the file that will set control parameters for this mechanism
/// @param [in] std::string the name of the network table for logging information
/// @param [in] std::shared_ptr<IDragonMotorController> motor controller used by this mechanism
Mech1IndMotor::Mech1IndMotor

(
    MechanismTypes::MECHANISM_TYPE              type,
    std::string                                 controlFileName,
    std::string                                 networkTableName,
    std::shared_ptr<IDragonMotorController>     motorController
) : m_type(type),
    m_controlFile(controlFileName),
    m_ntName(networkTableName),
    m_logging(false),
    m_motor( motorController ),
    m_target( 0.0 )
{
    if (m_motor.get() == nullptr )
    {
        Logger::GetLogger()->LogData( Logger::LOGGER_LEVEL::ERROR_ONCE, networkTableName, string( "Mech1IndMotor constructor" ), string( "motorController is nullptr" ) );
    }
}

/// @brief          Indicates the type of mechanism this is
/// @return         MechanismTypes::MECHANISM_TYPE
MechanismTypes::MECHANISM_TYPE Mech1IndMotor::GetType() const 
{
    return m_type;
}

/// @brief indicate the file used to get the control parameters from
/// @return std::string the name of the file 
std::string Mech1IndMotor::GetControlFileName() const 
{
    return m_controlFile;
}


/// @brief indicate the network table name used to for logging parameters
/// @return std::string the name of the network table 
std::string Mech1IndMotor::GetNetworkTableName() const 
{
    return m_ntName;
}

/// @brief log data to the network table if it is activated and time period has past
void Mech1IndMotor::LogData()
{
    auto ntName = GetNetworkTableName();
    Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, ntName, "Speed", GetSpeed() );
    Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, ntName, "Position", GetPosition() );
    Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, ntName, "Target", GetTarget() );
}

void Mech1IndMotor::Update()
{
    if ( m_motor.get() != nullptr )
    {
        auto ntName = GetNetworkTableName();
        m_motor.get()->Set(ntName, m_target );
    }
    LogData();
}

void Mech1IndMotor::UpdateTarget
(
    double  target
)
{
    m_target = target;
    Update();
}


double Mech1IndMotor::GetPosition() const

{
    return m_motor.get()->GetRotations() * 360.0;
}



double Mech1IndMotor::GetSpeed() const

{
    return m_motor.get()->GetRPS();
}


/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] ControlData* pid:  the control constants
/// @return void
void Mech1IndMotor::SetControlConstants
(
    int                                         slot,
    ControlData*                                pid                 
)
{
    if ( m_motor.get() != nullptr )
    {
        m_motor.get()->SetControlConstants( slot, pid );
    }
}





