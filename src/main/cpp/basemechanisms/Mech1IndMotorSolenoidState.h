
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

#pragma once

#include <memory>

#include <basemechanisms/interfaces/IState.h>
#include <mechanisms/controllers/ControlData.h>
#include <mechanisms/controllers/MechanismTargetData.h>
#include <basemechanisms/Mech1MotorState.h>
#include <basemechanisms/MechSolenoidState.h>
#include <basemechanisms/interfaces/IMech1IndMotorSolenoid.h>

class Mech1IndMotorSolenoidState : public IState
{
    public:

        Mech1IndMotorSolenoidState
        (
            IMech1IndMotorSolenoid*         mechanism,
            ControlData*                    control,
            double                          target,
            MechanismTargetData::SOLENOID   solState
        );
        Mech1IndMotorSolenoidState() = delete;
        ~Mech1IndMotorSolenoidState() = default;

        void Init() override;
        void Run() override;
        void Exit() override;
        bool AtTarget() const override;

        double GetTarget() const;
        double GetRPS() const;

    private:
        IMech1IndMotorSolenoid*                 m_mechanism;
        std::shared_ptr<Mech1MotorState>        m_motorState;
        std::shared_ptr<MechSolenoidState>      m_solenoidState;
};
