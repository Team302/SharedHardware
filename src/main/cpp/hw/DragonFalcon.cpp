
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
#include <frc/PowerDistribution.h>
#include <frc/motorcontrol/MotorController.h>

// Team 302 includes
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/DragonFalcon.h>
#include <hw/factories/PDPFactory.h>
//#include <hw/DragonPDP.h>
#include <hw/usages/MotorControllerUsage.h>
#include <utils/Logger.h>
#include <utils/ConversionUtils.h>

// Third Party Includes
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>


using namespace frc;
using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

DragonFalcon::DragonFalcon
(
	MotorControllerUsage::MOTOR_CONTROLLER_USAGE deviceType, 
	int deviceID, 
    int pdpID, 
	int countsPerRev, 
	double gearRatio,
	double countsPerInch,
	double countsPerDegree,
	MOTOR_TYPE motorType 
) : m_talon( make_shared<WPI_TalonFX>(deviceID)),
	m_controlMode(ControlModes::CONTROL_TYPE::PERCENT_OUTPUT),
	m_type(deviceType),
	m_id(deviceID),
	m_pdp( pdpID ),
	m_countsPerRev(countsPerRev),
	m_tickOffset(0),
	m_gearRatio(gearRatio),
	m_diameter( 1.0 ),
	m_countsPerInch(countsPerInch),
	m_countsPerDegree(countsPerDegree),
	m_motorType(motorType)
{
	// for all calls if we get an error log it; for key items try again
	auto prompt = string("Dragon Falcon");
	prompt += to_string(deviceID);
	auto error = m_talon.get()->ConfigFactoryDefault();
	if ( error != ErrorCode::OKAY )
	{
		m_talon.get()->ConfigFactoryDefault();
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigFactoryDefault"), string("error"));
		error = ErrorCode::OKAY;
	}

	m_talon.get()->SetNeutralMode(NeutralMode::Brake);

	error = m_talon.get()->ConfigNeutralDeadband(0.01, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigNeutralDeadband"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigNominalOutputForward(0.0, 0);
	if ( error != ErrorCode::OKAY )
	{
		m_talon.get()->ConfigNominalOutputForward(0.0, 0);
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigNominalOutputForward"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigNominalOutputReverse(0.0, 0);
	if ( error != ErrorCode::OKAY )
	{
		m_talon.get()->ConfigNominalOutputReverse(0.0, 0);
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigNominalOutputReverse"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigOpenloopRamp(0.0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigOpenloopRamp"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigPeakOutputForward(1.0, 0);
	if ( error != ErrorCode::OKAY )
	{
		m_talon.get()->ConfigPeakOutputForward(1.0, 0);
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigPeakOutputForward"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigPeakOutputReverse(-1.0, 0);
	if ( error != ErrorCode::OKAY )
	{
		m_talon.get()->ConfigPeakOutputReverse(-1.0, 0);
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigPeakOutputReverse"), string("error"));
		error = ErrorCode::OKAY;
	}

	SupplyCurrentLimitConfiguration climit;
	climit.enable = false;
	climit.currentLimit = 1.0;
	climit.triggerThresholdCurrent = 1.0;
	climit.triggerThresholdTime = 0.001;
	error = m_talon.get()->ConfigSupplyCurrentLimit(climit, 50);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigSupplyCurrentLimit"), string("error"));
		error = ErrorCode::OKAY;
	}
	StatorCurrentLimitConfiguration climit2;
	climit2.enable = false;
	climit2.currentLimit = 1.0;
	climit2.triggerThresholdCurrent = 1.0;
	climit2.triggerThresholdTime = 0.001;
	error = m_talon.get()->ConfigStatorCurrentLimit( climit2, 50);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigStatorCurrentLimit"), string("error"));
		error = ErrorCode::OKAY;
	}

	error = m_talon.get()->ConfigVoltageCompSaturation(12.0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigVoltageCompSaturation"), string("error"));
		error = ErrorCode::OKAY;
	}

	error = m_talon.get()->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_Deactivated, LimitSwitchNormal::LimitSwitchNormal_Disabled, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigForwardLimitSwitchSource"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_Deactivated, LimitSwitchNormal::LimitSwitchNormal_Disabled, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigReverseLimitSwitchSource"), string("error"));
		error = ErrorCode::OKAY;
	}

	error = m_talon.get()->ConfigForwardSoftLimitEnable(false, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigForwardSoftLimitEnable"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigForwardSoftLimitThreshold(0.0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigForwardSoftLimitThreshold"), string("error"));
		error = ErrorCode::OKAY;
	}

	error = m_talon.get()->ConfigReverseSoftLimitEnable(false, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigReverseSoftLimitEnable"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigReverseSoftLimitThreshold(0.0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigReverseSoftLimitThreshold"), string("error"));
		error = ErrorCode::OKAY;
	}
	
	
	error = m_talon.get()->ConfigMotionAcceleration(1500.0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigMotionAcceleration"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigMotionCruiseVelocity(1500.0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigMotionCruiseVelocity"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigMotionSCurveStrength(0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigMotionSCurveStrength"), string("error"));
		error = ErrorCode::OKAY;
	}

	error = m_talon.get()->ConfigMotionProfileTrajectoryPeriod(0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigMotionProfileTrajectoryPeriod"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigMotionProfileTrajectoryInterpolationEnable(true, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigMotionProfileTrajectoryInterpolationEnable"), string("error"));
		error = ErrorCode::OKAY;
	}

	m_talon.get()->ConfigAllowableClosedloopError(0.0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigAllowableClosedloopError"), string("error"));
		error = ErrorCode::OKAY;
	}

	for ( auto inx=0; inx<4; ++inx )
	{
		error = m_talon.get()->ConfigClosedLoopPeakOutput(inx, 1.0, 0);
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigClosedLoopPeakOutput"), string("error"));
			error = ErrorCode::OKAY;
		}
		error = m_talon.get()->ConfigClosedLoopPeriod(inx, 10, 0);
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigClosedLoopPeriod"), string("error"));
			error = ErrorCode::OKAY;
		}
		error = m_talon.get()->Config_kP(inx, 0.01, 0);
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("Config_kP"), string("error"));
			error = ErrorCode::OKAY;
		}
		error = m_talon.get()->Config_kI(inx, 0.0, 0);
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("Config_kI"), string("error"));
			error = ErrorCode::OKAY;
		}
		error = m_talon.get()->Config_kD(inx, 0.0, 0);
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("Config_kD"), string("error"));
			error = ErrorCode::OKAY;
		}
		error = m_talon.get()->Config_kF(inx, 1.0, 0);
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("Config_kF"), string("error"));
			error = ErrorCode::OKAY;
		}
		error = m_talon.get()->Config_IntegralZone(inx, 0.0, 0);
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("Config_IntegralZone"), string("error"));
			error = ErrorCode::OKAY;
		}
	}

	error = m_talon.get()->ConfigRemoteFeedbackFilter(60, RemoteSensorSource::RemoteSensorSource_Off, 0, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigRemoteFeedbackFilter"), string("error"));
		error = ErrorCode::OKAY;
	}
	error = m_talon.get()->ConfigRemoteFeedbackFilter(60, RemoteSensorSource::RemoteSensorSource_Off, 1, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigRemoteFeedbackFilter"), string("error"));
	}
}

double DragonFalcon::GetRotations() const
{
	if (m_countsPerDegree > 0.01)
	{
		return m_talon.get()->GetSelectedSensorPosition() / (m_countsPerDegree * 360.0);
	}
	return (ConversionUtils::CountsToRevolutions( (m_talon.get()->GetSelectedSensorPosition()), m_countsPerRev) / m_gearRatio);
}

double DragonFalcon::GetRPS() const
{
	if (m_countsPerDegree > 0.01)
	{
		return m_talon.get()->GetSelectedSensorVelocity() * 10.0 / (m_countsPerDegree * 360.0);
	}
	return (ConversionUtils::CountsPer100msToRPS( m_talon.get()->GetSelectedSensorVelocity(), m_countsPerRev) / m_gearRatio);
}

void DragonFalcon::SetControlMode(ControlModes::CONTROL_TYPE mode)
{ 
	m_controlMode = mode;
}

shared_ptr<MotorController> DragonFalcon::GetSpeedController() const
{
	return m_talon;
}

double DragonFalcon::GetCurrent() const
{
	auto pdp = PDPFactory::GetFactory()->GetPDP();
	if (pdp != nullptr)
	{
		return pdp->GetCurrent(m_pdp);
	}
	return 0.0;
}

void DragonFalcon::UpdateFramePeriods
(
	ctre::phoenix::motorcontrol::StatusFrameEnhanced	frame,
	uint8_t												milliseconds
)
{
	m_talon.get()->SetStatusFramePeriod( frame, milliseconds, 0 );
}

void DragonFalcon::SetFramePeriodPriority
(
	MOTOR_PRIORITY              priority
)
{
	switch ( priority )
	{
		case HIGH:
			UpdateFramePeriods( StatusFrameEnhanced::Status_1_General, 10 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_2_Feedback0, 20 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_3_Quadrature, 100 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_4_AinTempVbat, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_8_PulseWidth, 120 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_10_Targets, 120 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_11_UartGadgeteer, 120 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_12_Feedback1, 120 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_13_Base_PIDF0, 120 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_14_Turn_PIDF1, 120 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_15_FirmareApiStatus, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_Brushless_Current, 200 );
			break;

		case MEDIUM:
			UpdateFramePeriods( StatusFrameEnhanced::Status_1_General, 60 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_2_Feedback0, 120 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_3_Quadrature, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_4_AinTempVbat, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_8_PulseWidth, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_10_Targets, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_11_UartGadgeteer, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_12_Feedback1, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_13_Base_PIDF0, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_14_Turn_PIDF1, 150 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_15_FirmareApiStatus, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_Brushless_Current, 200 );
			break;

		case LOW:
			UpdateFramePeriods( StatusFrameEnhanced::Status_1_General, 120 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_2_Feedback0, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_3_Quadrature, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_4_AinTempVbat, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_8_PulseWidth, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_10_Targets, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_11_UartGadgeteer, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_12_Feedback1, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_13_Base_PIDF0, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_14_Turn_PIDF1, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_15_FirmareApiStatus, 200 );
			UpdateFramePeriods( StatusFrameEnhanced::Status_Brushless_Current, 200 );
			break;

		default:
		break;

	}
}

void DragonFalcon::Set(std::string nt, double value)
{
	Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, nt, string("motor id"), m_talon.get()->GetDeviceID());
	Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, nt, string("control mode"), m_controlMode);

	if ( m_controlMode == ControlModes::CONTROL_TYPE::VOLTAGE)
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, nt, string("motor target output voltage"), value);
		m_talon.get()->SetVoltage(units::voltage::volt_t(value));
	}
	else
	{
		auto output = value;
		ctre::phoenix::motorcontrol::TalonFXControlMode ctreMode = ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput;
		switch (m_controlMode)
		{
			case ControlModes::CONTROL_TYPE::PERCENT_OUTPUT:
				ctreMode = ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput;
				break;
				
			case ControlModes::CONTROL_TYPE::POSITION_ABSOLUTE:
			case ControlModes::CONTROL_TYPE::POSITION_DEGREES:
			case ControlModes::CONTROL_TYPE::POSITION_INCH:
				ctreMode =:: ctre::phoenix::motorcontrol::TalonFXControlMode::Position;
				break;

			case ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE:
			case ControlModes::CONTROL_TYPE::TRAPEZOID:
				ctreMode =:: ctre::phoenix::motorcontrol::TalonFXControlMode::MotionMagic;
				break;
			
			case ControlModes::CONTROL_TYPE::VELOCITY_DEGREES:
			case ControlModes::CONTROL_TYPE::VELOCITY_INCH:
			case ControlModes::CONTROL_TYPE::VELOCITY_RPS:
				ctreMode = ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity;
				break;

			case ControlModes::CONTROL_TYPE::CURRENT:
				ctreMode = ctre::phoenix::motorcontrol::TalonFXControlMode::Current;
				break;

			case ControlModes::CONTROL_TYPE::MOTION_PROFILE:
				ctreMode = ctre::phoenix::motorcontrol::TalonFXControlMode::MotionProfile;
				break;

			case ControlModes::CONTROL_TYPE::MOTION_PROFILE_ARC:
				ctreMode = ctre::phoenix::motorcontrol::TalonFXControlMode::MotionProfileArc;
				break;

			default:
				string msg;
				msg = string("Invalid control mode ");
				msg += to_string(m_controlMode);
				msg += " ";
				msg += to_string(m_talon.get()->GetDeviceID());
				Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, string("DragonFalcon"), string("SetControlMode"), msg);
				ctreMode = ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput;
				break;
		}	

		switch (m_controlMode)
		{
			case ControlModes::CONTROL_TYPE::POSITION_DEGREES:
				if (m_countsPerDegree > 0.01)
				{
					output = m_countsPerDegree*value;
				}
				else
				{
					output = (ConversionUtils::DegreesToCounts(value,m_countsPerRev) * m_gearRatio);
				}
				break;

			case ControlModes::CONTROL_TYPE::POSITION_INCH:
			case ControlModes::CONTROL_TYPE::TRAPEZOID:
				if (m_countsPerInch > 0.01)
				{
					output = m_countsPerInch*value;
				}
				else
				{
					output = (ConversionUtils::InchesToCounts(value, m_countsPerRev, m_diameter) * m_gearRatio);
				}
				break;
			
			case ControlModes::CONTROL_TYPE::VELOCITY_DEGREES:
				if (m_countsPerDegree > 0.01)
				{
					output = m_countsPerDegree*value * 0.1;
				}
				else
				{
					output = (ConversionUtils::DegreesPerSecondToCounts100ms( value, m_countsPerRev ) * m_gearRatio);
				}
				break;

			case ControlModes::CONTROL_TYPE::VELOCITY_INCH:
				if (m_countsPerInch > 0.01)
				{
					output = m_countsPerInch*value *0.1;
				}
				else
				{
					output = (ConversionUtils::InchesPerSecondToCounts100ms( value, m_countsPerRev, m_diameter ) * m_gearRatio);
				}
				break;

			case ControlModes::CONTROL_TYPE::VELOCITY_RPS:
				if (m_countsPerDegree > 0.01)
				{
					output = value * 360.0 * m_countsPerDegree * 0.1;
				}
				else
				{
					output = (ConversionUtils::RPSToCounts100ms( value, m_countsPerRev ) * m_gearRatio);
				}
				break;

			default:
				break;
		}	

		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, nt, string("motor target output"), output);

		m_talon.get()->Set( ctreMode, output );

	}
	Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, nt, string("motor current percent output"), m_talon.get()->Get() );
	Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, nt, string("motor current RPS"), GetRPS() );
	Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, nt, string("voltage"), m_talon.get()->GetMotorOutputVoltage());
}

void DragonFalcon::Set(double value)
{
	auto id = m_talon.get()->GetDeviceID();
	auto ntName = std::string("MotorOutput");
	ntName += to_string(id);
	Set(ntName, value);
}

void DragonFalcon::SetRotationOffset(double rotations)
{
//	double newRotations = -rotations + DragonFalcon::GetRotations();
//	m_tickOffset += (int) (newRotations * m_countsPerRev / m_gearRatio);
}

void DragonFalcon::SetVoltageRamping(double ramping, double rampingClosedLoop)
{
	auto prompt = string("Dragon Falcon");
	prompt += to_string(m_talon.get()->GetDeviceID());
    auto error = m_talon.get()->ConfigOpenloopRamp(ramping);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigOpenloopRamp"), string("error"));
	}
	if (rampingClosedLoop >= 0)
	{
		error = m_talon.get()->ConfigClosedloopRamp(rampingClosedLoop);
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigClosedloopRamp"), string("error"));
		}
	}
}


void DragonFalcon::EnableCurrentLimiting(bool enabled)
{
	SupplyCurrentLimitConfiguration limit;
	auto prompt = string("Dragon Falcon");
	prompt += to_string(m_talon.get()->GetDeviceID());
	int timeout = 50.0;
	auto error = m_talon.get()->ConfigGetSupplyCurrentLimit( limit, timeout );
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigGetSupplyCurrentLimit"), string("error"));
	}
	limit.enable = enabled;
	error = m_talon.get()->ConfigSupplyCurrentLimit( limit, timeout );
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigSupplyCurrentLimit"), string("error"));
	}
}

void DragonFalcon::EnableBrakeMode(bool enabled)
{
    m_talon.get()->SetNeutralMode(enabled ? ctre::phoenix::motorcontrol::NeutralMode::Brake : ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

void DragonFalcon::Invert(bool inverted)
{
    m_talon.get()->SetInverted(inverted);
}

void DragonFalcon::SetSensorInverted(bool inverted)
{
    m_talon.get()->SetSensorPhase(inverted);
}

MotorControllerUsage::MOTOR_CONTROLLER_USAGE DragonFalcon::GetType() const
{
	return m_type;
}

int DragonFalcon::GetID() const
{
	return m_id;
}

//------------------------------------------------------------------------------
// Method:		SelectClosedLoopProfile
// Description:	Selects which profile slot to use for closed-loop control
// Returns:		void
//------------------------------------------------------------------------------
void DragonFalcon::SelectClosedLoopProfile
(
	int	   slot,			// <I> - profile slot to select
	int    pidIndex			// <I> - 0 for primary closed loop, 1 for cascaded closed-loop
)
{
	auto error = m_talon.get()->SelectProfileSlot( slot, pidIndex );
	if ( error != ErrorCode::OKAY )
	{
		auto prompt = string("Dragon Falcon");
		prompt += to_string(m_talon.get()->GetDeviceID());
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("SelectProfileSlot"), string("error"));
	}
}

int DragonFalcon::ConfigSelectedFeedbackSensor
(
	FeedbackDevice feedbackDevice,
	int pidIdx,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		error = m_talon.get()->ConfigSelectedFeedbackSensor( feedbackDevice, pidIdx, timeoutMs );
	}
	else
	{
		auto prompt = string("Dragon Falcon");
		prompt += to_string(m_talon.get()->GetDeviceID());
        Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("DragonFalcon::ConfigSelectedFeedbackSensor"), string("m_talon is a nullptr"));
	}
	return error;
}

int DragonFalcon::ConfigSelectedFeedbackSensor
(
	RemoteFeedbackDevice feedbackDevice,
	int pidIdx,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		error = m_talon.get()->ConfigSelectedFeedbackSensor( feedbackDevice, pidIdx, timeoutMs );
	}
	else
	{
		auto prompt = string("Dragon Falcon");
		prompt += to_string(m_talon.get()->GetDeviceID());
        Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("DragonFalcon::ConfigSelectedFeedbackSensor"), string("m_talon is a nullptr"));
	}
	return error;
}

int DragonFalcon::ConfigPeakCurrentLimit
(
	int amps,
	int timeoutMs
)
{
	int ierror = 0;
	if ( m_talon.get() != nullptr )
	{
		auto prompt = string("Dragon Falcon");
		prompt += to_string(m_talon.get()->GetDeviceID());
		SupplyCurrentLimitConfiguration limit;
		auto error = m_talon.get()->ConfigGetSupplyCurrentLimit( limit, timeoutMs );
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigGetSupplyCurrentLimit"), string("error"));
		}
		limit.triggerThresholdCurrent = amps;
		error = m_talon.get()->ConfigSupplyCurrentLimit( limit, timeoutMs );
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigSupplyCurrentLimit"), string("error"));
		}
		ierror = error;
	}
	else
	{
        Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, string("DragonFalcon"), string("DragonFalcon::ConfigPeakCurrentLimit"), string("m_talon is a nullptr"));
	}
	return ierror;
}

int DragonFalcon::ConfigPeakCurrentDuration
(
	int milliseconds,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		auto prompt = string("Dragon Falcon");
		prompt += to_string(m_talon.get()->GetDeviceID());
		SupplyCurrentLimitConfiguration limit;
		auto error = m_talon.get()->ConfigGetSupplyCurrentLimit( limit, timeoutMs );
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigGetSupplyCurrentLimit"), string("error"));
		}
		limit.triggerThresholdTime = milliseconds;
		error = m_talon.get()->ConfigSupplyCurrentLimit( limit, timeoutMs );
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigSupplyCurrentLimit"), string("error"));
		}
	}
	else
	{
        Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, string("DragonFalcon"), string("ConfigPeakCurrentDuration"), string("m_talon is a nullptr"));
	}
	return error;
}

int DragonFalcon::ConfigContinuousCurrentLimit
(
	int amps,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		auto prompt = string("Dragon Falcon");
		prompt += to_string(m_talon.get()->GetDeviceID());
		SupplyCurrentLimitConfiguration limit;
		auto error = m_talon.get()->ConfigGetSupplyCurrentLimit( limit, timeoutMs );
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigGetSupplyCurrentLimit"), string("error"));
		}
		limit.currentLimit = amps;
		error = m_talon.get()->ConfigSupplyCurrentLimit( limit, timeoutMs );
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigSupplyCurrentLimit"), string("error"));
		}
	}
	else
	{
        Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, string("DragonFalcon"), string("ConfigContinuousCurrentLimit"), string("m_talon is a nullptr"));
	}
	return error;
}

void DragonFalcon::SetAsFollowerMotor
(
    int         masterCANID         // <I> - master motor
)
{
    m_talon.get()->Set( ControlMode::Follower, masterCANID );
}


/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] int             slot - hardware slot to use
/// @param [in] ControlData*    pid - the control constants
/// @return void
void DragonFalcon::SetControlConstants(int slot, ControlData* controlInfo)
{
	SetControlMode(controlInfo->GetMode());

	auto prompt = string("Dragon Falcon");
	prompt += to_string(m_talon.get()->GetDeviceID());

	auto id = m_talon.get()->GetDeviceID();
	auto ntName = std::string("MotorOutput");
	ntName += to_string(id);
	Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, ntName, string("P"), controlInfo->GetP());
	Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, ntName, string("I"), controlInfo->GetI());
	Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, ntName, string("D"), controlInfo->GetD());
	Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::PRINT, ntName, string("F"), controlInfo->GetF());

	auto peak = controlInfo->GetPeakValue();
	auto error = m_talon.get()->ConfigPeakOutputForward(peak);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, ntName, prompt, string("ConfigPeakOutputForward error"));
	}
	error = m_talon.get()->ConfigPeakOutputReverse(-1.0*peak);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, ntName, prompt, string("ConfigPeakOutputReverse error"));
	}

	auto nom = controlInfo->GetNominalValue();
	error = m_talon.get()->ConfigNominalOutputForward(nom);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, ntName, prompt, string("ConfigNominalOutputForward error"));
	}
	error = m_talon.get()->ConfigNominalOutputReverse(-1.0*nom);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, ntName, prompt, string("ConfigNominalOutputReverse error"));
	}

	if ( controlInfo->GetMode() == ControlModes::CONTROL_TYPE::POSITION_ABSOLUTE ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::POSITION_DEGREES ||
	     controlInfo->GetMode() == ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::POSITION_INCH ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::VELOCITY_DEGREES ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::VELOCITY_INCH ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::VELOCITY_RPS  ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::VOLTAGE ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::CURRENT ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::TRAPEZOID )
	{
		error = m_talon.get()->Config_kP(slot, controlInfo->GetP());
		if ( error != ErrorCode::OKAY )
		{
			m_talon.get()->Config_kP(slot, controlInfo->GetP());
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, ntName, prompt, string("Config_kP error"));
		}
		error = m_talon.get()->Config_kI(slot, controlInfo->GetI());
		if ( error != ErrorCode::OKAY )
		{
			m_talon.get()->Config_kI(slot, controlInfo->GetI());
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, ntName, prompt, string("Config_kI error"));
		}
		error = m_talon.get()->Config_kD(slot, controlInfo->GetD());
		if ( error != ErrorCode::OKAY )
		{
			m_talon.get()->Config_kD(slot, controlInfo->GetD());
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, ntName, prompt, string("Config_kD error"));
		}
		error = m_talon.get()->Config_kF(slot, controlInfo->GetF());
		if ( error != ErrorCode::OKAY )
		{
			m_talon.get()->Config_kF(slot, controlInfo->GetF());
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, ntName, prompt, string("Config_kF error"));
		}
		error = m_talon.get()->SelectProfileSlot(slot, 0);
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, ntName, prompt, string("SelectProfileSlot error"));
		}
	}

	
	if ( //controlInfo->GetMode() == ControlModes::CONTROL_TYPE::POSITION_ABSOLUTE ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE ||
	     controlInfo->GetMode() == ControlModes::CONTROL_TYPE::TRAPEZOID  )
	{
		error = m_talon.get()->ConfigMotionAcceleration( controlInfo->GetMaxAcceleration() );
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, ntName, prompt, string("ConfigMotionAcceleration error"));
		}
		error = m_talon.get()->ConfigMotionCruiseVelocity( controlInfo->GetCruiseVelocity(), 0);
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, ntName, prompt, string("ConfigMotionCruiseVelocity error"));
		}

	}
}


void DragonFalcon::SetForwardLimitSwitch
( 
	bool normallyOpen
)
{
	LimitSwitchNormal type = normallyOpen ? LimitSwitchNormal::LimitSwitchNormal_NormallyOpen : LimitSwitchNormal::LimitSwitchNormal_NormallyClosed;
	auto error = m_talon.get()->ConfigForwardLimitSwitchSource( LimitSwitchSource::LimitSwitchSource_FeedbackConnector, type, 0  );
	if ( error != ErrorCode::OKAY )
	{
		auto ntName = std::string("MotorOutput");
		ntName += to_string(m_talon.get()->GetDeviceID());		
		auto prompt = string("Dragon Falcon");
		prompt += to_string(m_talon.get()->GetDeviceID());
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, ntName, prompt, string("ConfigForwardLimitSwitchSource error"));
	}
}

void DragonFalcon::SetReverseLimitSwitch
(
	bool normallyOpen
)
{
	LimitSwitchNormal type = normallyOpen ? LimitSwitchNormal::LimitSwitchNormal_NormallyOpen : LimitSwitchNormal::LimitSwitchNormal_NormallyClosed;
	auto error = m_talon.get()->ConfigReverseLimitSwitchSource( LimitSwitchSource::LimitSwitchSource_FeedbackConnector, type, 0  );
	if ( error != ErrorCode::OKAY )
	{
		auto ntName = std::string("MotorOutput");
		ntName += to_string(m_talon.get()->GetBaseID());		
		auto prompt = string("Dragon Falcon");
		prompt += to_string(m_talon.get()->GetDeviceID());
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigReverseLimitSwitchSource"), string("error"));
	}
}

void DragonFalcon::SetRemoteSensor
(
    int                                             canID,
    ctre::phoenix::motorcontrol::RemoteSensorSource deviceType
)
{
	auto prompt = string("Dragon Falcon");
	prompt += to_string(m_talon.get()->GetDeviceID());
	auto error = m_talon.get()->ConfigRemoteFeedbackFilter( canID, deviceType, 0, 0.0 );
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigRemoteFeedbackFilter"), string("error"));
	}
	error = m_talon.get()->ConfigSelectedFeedbackSensor( RemoteFeedbackDevice::RemoteFeedbackDevice_RemoteSensor0, 0, 0 );
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, prompt, string("ConfigSelectedFeedbackSensor"), string("error"));
	}
}

void DragonFalcon::SetDiameter
(
	double 	diameter
)
{
	m_diameter = diameter;
}

void DragonFalcon::SetVoltage
(
	units::volt_t output
)
{
	m_talon.get()->SetVoltage(output);
}

bool DragonFalcon::IsForwardLimitSwitchClosed() const
{
	auto sensors = m_talon.get()->GetSensorCollection();
	auto closed = sensors.IsFwdLimitSwitchClosed();
	return closed == 1;
}

bool DragonFalcon::IsReverseLimitSwitchClosed() const
{
	auto sensors = m_talon.get()->GetSensorCollection();
	auto closed = sensors.IsRevLimitSwitchClosed();
	return closed == 1;
}

void DragonFalcon::EnableDisableLimitSwitches
(
	bool enable
)
{
	m_talon.get()->OverrideLimitSwitchesEnable(enable);
}

void DragonFalcon::EnableVoltageCompensation( double fullvoltage) 
{
	m_talon.get()->ConfigVoltageCompSaturation(fullvoltage);
	m_talon.get()->EnableVoltageCompensation(true);
}



void DragonFalcon::SetSelectedSensorPosition
(
	double  initialPosition
) 
{
	m_talon.get()->SetSelectedSensorPosition(initialPosition, 0, 50);
}

        
double DragonFalcon::GetCountsPerInch() const 
{
	return m_countsPerInch;
}
double DragonFalcon::GetCountsPerDegree() const 
{
	return m_countsPerDegree;
}


ControlModes::CONTROL_TYPE DragonFalcon::GetControlMode() const
{
	return m_controlMode;
}

double DragonFalcon::GetCounts() const 
{
	return m_talon.get()->GetSelectedSensorPosition();
}

IDragonMotorController::MOTOR_TYPE DragonFalcon::GetMotorType() const
{
	return m_motorType;
}
