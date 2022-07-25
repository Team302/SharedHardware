
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

//========================================================================================================
/// @interface IMech1IndMotor
/// @brief     This is the interface for mechanisms that have one independently controlled motor.
//========================================================================================================

// C++ Includes
#include <memory>

// FRC includes

// Team 302 includes
#include <mechanisms/controllers/ControlModes.h>
#include <basemechanisms/interfaces/IMech.h>
#include <basemechanisms/interfaces/IMech1IndMotor.h>
#include <basemechanisms/interfaces/IMech1Solenoid.h>
#include <mechanisms/MechanismTypes.h>
#include <mechanisms/controllers/ControlData.h>
// Third Party Includes


///	 @interface IMech1IndMotor
///  @brief	    Interface for subsystems
class IMech1IndMotorSolenoid : public IMech
{
	public:
        /// @brief update the output to the mechanism using the current controller and target value(s)
        /// @return void 
        virtual void Update() = 0;

        virtual void UpdateTarget
        (
            double target
        ) = 0;

        /// @brief  Return the current position of the mechanism.  The value is in inches or degrees.
        /// @return double	position in inches (translating mechanisms) or degrees (rotating mechanisms)
        virtual double GetPosition() const = 0;

        /// @brief  Get the current speed of the mechanism.  The value is in inches per second or degrees per second.
        /// @return double	speed in inches/second (translating mechanisms) or degrees/second (rotating mechanisms)
        virtual double GetSpeed() const =0;

        /// @brief  Set the control constants (e.g. PIDF values).
        /// @param [in] ControlData*                                   pid:  the control constants
        /// @return void
        virtual void SetControlConstants
        (
            int                                         slot,
            ControlData*                                pid                 
        ) = 0;

        /// @brief      Activate/deactivate pneumatic solenoid
        /// @param [in] bool - true == extend, false == retract
        /// @return     void 
        virtual void ActivateSolenoid
        (
            bool     activate
        ) = 0;

        /// @brief      Check if the pneumatic solenoid is activated
        /// @return     bool - true == extended, false == retract
        virtual bool IsSolenoidActivated() const = 0;

	    IMech1IndMotorSolenoid() = default;
	    virtual ~IMech1IndMotorSolenoid() = default;

        virtual IMech1IndMotor* Get1IndMotorMech() const = 0;
        virtual IMech1Solenoid* GetSolenoidMech() const = 0;
};



