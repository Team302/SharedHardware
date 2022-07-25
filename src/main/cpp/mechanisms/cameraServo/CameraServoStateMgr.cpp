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

// C++ Includes
#include <map>
#include <memory>
#include <vector>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <basemechanisms/interfaces/IState.h>
#include <mechanisms/controllers/StateDataXmlParser.h>
#include <mechanisms/controllers/MechanismTargetData.h>
#include <utils/Logger.h>
#include <TeleopControl.h>
#include <mechanisms/StateMgr.h>
#include <mechanisms/StateStruc.h>
#include <mechanisms/MechanismFactory.h>
#include <mechanisms/MechanismTypes.h>
#include <mechanisms/cameraServo/CameraServoStateMgr.h>
#include <hw/DragonServo.h>


// Third Party Includes

using namespace std;


CameraServoStateMgr* CameraServoStateMgr::m_instance = nullptr;
CameraServoStateMgr* CameraServoStateMgr::GetInstance()
{
	if ( CameraServoStateMgr::m_instance == nullptr )
	{
	    auto mechFactory = MechanismFactory::GetMechanismFactory();
	    auto cameraServo = mechFactory->GetCameraServo();
	    if (cameraServo != nullptr)
        {
	        CameraServoStateMgr::m_instance = new CameraServoStateMgr();
        }
	}
	return CameraServoStateMgr::m_instance;
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
CameraServoStateMgr::CameraServoStateMgr() : m_camera((MechanismFactory::GetMechanismFactory()->GetCameraServo()))
{
    map<string, StateStruc> stateMap;
    stateMap["CAMERASERVOLOOKRIGHT"] = m_rightState;
    stateMap["CAMERASERVOLOOKLEFT"] = m_leftState;
    stateMap["CAMERASERVOSCAN"] = m_scanState;

    Init(m_camera, stateMap);
}   

/// @brief  run the current state
/// @return void
void CameraServoStateMgr::CheckForStateTransition()
{
    if ( MechanismFactory::GetMechanismFactory()->GetCameraServo() != nullptr )
    {
        // process teleop/manual interrupts
        auto currentState = static_cast<CAMERA_SERVO_STATE>(GetCurrentState());
        auto targetState = currentState;
        //auto targetState

        auto controller = TeleopControl::GetInstance();
        if ( controller != nullptr )
        {
            auto rightPressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::LOOK_RIGHT);
            auto leftPressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::LOOK_LEFT);
            auto scanPressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::SCAN);



            if (rightPressed  &&  currentState != CAMERA_SERVO_STATE::LOOK_RIGHT )
            {
                targetState = CAMERA_SERVO_STATE::LOOK_RIGHT;
            }
            else if (leftPressed && currentState != CAMERA_SERVO_STATE::LOOK_LEFT )
            {
                targetState = CAMERA_SERVO_STATE::LOOK_LEFT;
            }
            else if (scanPressed && currentState != CAMERA_SERVO_STATE::SCAN)
            {
                targetState = CAMERA_SERVO_STATE::SCAN;

                if (m_camera != nullptr)
                {
                    auto currentAngle = m_camera->GetAngle();
                    currentAngle += m_increment;

                    if (currentAngle > 180 || currentAngle < 0)
                    {
                        m_increment *= -1;
                        currentAngle += m_increment;
                    }
                    m_camera->SetAngle(currentAngle);
                } 


            }
        }
        if (targetState != currentState && targetState != CAMERA_SERVO_STATE::SCAN)
        {
            SetCurrentState( targetState, true );
        }
        if (targetState == CAMERA_SERVO_STATE::SCAN)
        {
            if (m_camera != nullptr)
            {
                auto currentAngle = m_camera->GetAngle();
                currentAngle += m_increment;

                if (currentAngle > 180 || currentAngle < 0)
                {
                    m_increment *= -1;
                    currentAngle += m_increment;
                }
                m_camera->SetAngle(currentAngle);
            } 
        }
    }
}