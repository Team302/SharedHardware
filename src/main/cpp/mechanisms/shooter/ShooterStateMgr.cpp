//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302 
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================
#include <map>
#include <memory>
#include <vector>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <mechanisms/controllers/MechanismTargetData.h>
#include <TeleopControl.h>
#include <hw/DragonLimelight.h>
#include <hw/factories/LimelightFactory.h>
#include <mechanisms/MechanismFactory.h>
#include <mechanisms/MechanismTypes.h>
#include <mechanisms/StateMgr.h>
#include <mechanisms/StateStruc.h>
#include <basemechanisms/interfaces/IState.h>
#include <mechanisms/shooter/Shooter.h>
#include <mechanisms/shooter/ShooterState.h>
#include <mechanisms/shooter/ShooterStateMgr.h>
#include <utils/Logger.h>
#include <mechanisms/controllers/StateDataXmlParser.h>


// Third Party Includes

using namespace std;


ShooterStateMgr* ShooterStateMgr::m_instance = nullptr;
ShooterStateMgr* ShooterStateMgr::GetInstance()
{
	if ( ShooterStateMgr::m_instance == nullptr )
	{
	    auto mechFactory = MechanismFactory::GetMechanismFactory();
	    auto shooter = mechFactory->GetShooter();
	    if (shooter != nullptr)
        {
		    ShooterStateMgr::m_instance = new ShooterStateMgr();
        }
	}
	return ShooterStateMgr::m_instance;
    
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
ShooterStateMgr::ShooterStateMgr() : StateMgr(),
                                     m_shooter(MechanismFactory::GetMechanismFactory()->GetShooter()),
                                     m_nt(),
                                     m_buttonTriggerStateChange(false)
{
    map<string, StateStruc> stateMap;
    stateMap[m_shooterOffXmlString] = m_offState;
    stateMap[m_shooterHighGoalCloseXmlString] = m_shootFarState;
    stateMap[m_shooterHighGoalFarXmlString] = m_shootCloseState;
    stateMap[m_shooterLowGoalXmlString] = m_shootLowState;
    stateMap[m_shooterManualXmlString] = m_manualShootState;
    stateMap[m_shooterPrepareXmlString] = m_prepareToShoot;

    m_dragonLimeLight = LimelightFactory::GetLimelightFactory()->GetLimelight();
    

    Init(m_shooter, stateMap);
    if (m_shooter != nullptr)
    {
        auto m_nt = m_shooter->GetNetworkTableName();
    }
}   


bool ShooterStateMgr::AtTarget() const
{
    Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, m_nt, string("At Target"), GetCurrentStatePtr()->AtTarget() ? "true" : "false");
    return GetCurrentStatePtr()->AtTarget();
}

void ShooterStateMgr::CheckForStateTransition()
{

    if ( m_shooter != nullptr )
    {    
        auto currentState = static_cast<SHOOTER_STATE>(GetCurrentState());
        auto targetState = currentState;
        Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, m_nt, string("current state "), currentState);

        auto isShootHighSelected    = false;
        auto isShootLowSelected     = false;
        auto isManualShootSelected  = false;
        auto isShooterOffSelected   = false;
        auto isPrepareToShootSelected = false;

        auto controller = TeleopControl::GetInstance();
        if ( controller != nullptr )
        {
            isShootHighSelected      = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::AUTO_SHOOT_HIGH);
            isShootLowSelected       = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::AUTO_SHOOT_LOW);
            isManualShootSelected    = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::MANUAL_SHOOT);
            isShooterOffSelected     = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_OFF) ||
                                       controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::ENABLE_CLIMBER);
            isPrepareToShootSelected = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_MTR_ON);
        }

        if (isShootHighSelected)
        {
            targetState = SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR;
        }
        else if (isShootLowSelected)
        {
            targetState = SHOOTER_STATE::SHOOT_LOW_GOAL;
        }
        else if (isPrepareToShootSelected)
        {
            targetState = SHOOTER_STATE::PREPARE_TO_SHOOT;
        }
        else if (isManualShootSelected)
        {
            targetState = SHOOTER_STATE::SHOOT_MANUAL;
        }
        else if (isShooterOffSelected)
        {
            targetState = SHOOTER_STATE::OFF;
        }
        else if (currentState != SHOOTER_STATE::OFF && m_buttonTriggerStateChange)
        {
            targetState = SHOOTER_STATE::PREPARE_TO_SHOOT;
        }

        if (m_dragonLimeLight != nullptr && (targetState == SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE ||
                                             targetState == SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR ||
                                             targetState == SHOOTER_STATE::SHOOT_LOW_GOAL))
        {
            if(m_dragonLimeLight->GetTargetHorizontalOffset() > 5.0_deg)
            {
                targetState = currentState;
            }
        }

        m_buttonTriggerStateChange = isShootHighSelected    || 
                                     isShootLowSelected     || 
                                     isManualShootSelected  || 
                                     isShooterOffSelected   || 
                                     isPrepareToShootSelected;

        if (targetState != currentState)
        {
            Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, m_nt, string("Changing Shooter State"), targetState);
            SetCurrentState(targetState, true);
        }
        
    }
}

bool ShooterStateMgr::IsShooting() const
{
    auto shooterState = static_cast<SHOOTER_STATE>(GetCurrentState());
    return (shooterState == ShooterStateMgr::SHOOTER_STATE::SHOOT_MANUAL ||
            shooterState == ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE ||
            shooterState == ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR ||
            shooterState == ShooterStateMgr::SHOOTER_STATE::SHOOT_LOW_GOAL);
}

