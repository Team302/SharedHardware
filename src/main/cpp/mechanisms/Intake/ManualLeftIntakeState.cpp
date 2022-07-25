
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

#include <mechanisms/controllers/ControlData.h>
#include <TeleopControl.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <mechanisms/intake/Intake.h>
#include <mechanisms/intake/IntakeState.h>
#include <mechanisms/intake/ManualLeftIntakeState.h>

ManualLeftIntakeState::ManualLeftIntakeState
(
    Intake*      intake,
    ControlData* controlSpin,
    ControlData* controlExtend, 
    double       spinTarget,
    double       extendTarget
) : IntakeState(intake, controlSpin, controlExtend, spinTarget, extendTarget),
    m_controller(TeleopControl::GetInstance())
{
}


void ManualLeftIntakeState::Init()
{
}


void ManualLeftIntakeState::Run()           
{
    auto intake = GetIntake();
    if ( m_controller != nullptr && intake != nullptr )
    {
        auto percent  = m_controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::INTAKE_RETRACT_LEFT);
        auto motor = intake->GetSecondaryMotor();
        motor.get()->Set(percent);
    }    
}

bool ManualLeftIntakeState::AtTarget() const
{
    return true;
}


