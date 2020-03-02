#include "Periscope.h"
#include <ctre/phoenix.h>
#include <frc/WPILib.h>
using namespace frc;

Periscope::Periscope(int leftM, int rightM): _left(leftM), _right(rightM)
{
//Set these when we have a motor to run them
    posVals[Down][0] = 100;//Position Value
	posVals[Down][1] = 0;//Ramp
	posVals[Down][2] = 0;//Max Speed

	posVals[Low][0] = 12.0*4096/.8;
	posVals[Low][1] = 0;
	posVals[Low][2] = 0;

	posVals[Level][0] = 18.0*4096/.8;
	posVals[Level][1] = 0;
	posVals[Level][2] = 0;

    posVals[High][0] = 24.0*4096/.8;
	posVals[High][1] = 0;
	posVals[High][2] = 0;

	current = PPos::Down;
	_homed = false;
	_UsePID = false;
//Also Set these when we have a motor to run them

	TalonSRXConfiguration left, right;
	_left.ConfigFactoryDefault();
	_right.ConfigFactoryDefault();
/*
	left.primaryPID.selectedFeedbackSensor = FeedbackDevice::CTRE_MagEncoder_Relative;
	left.slot0.kP = .3;
	left.slot0.kI = 0;
	left.slot0.kD = 0;
	left.slot0.kF = 0;
	left.motionAcceleration = 10000;
    left.motionCruiseVelocity = 4096;
    left.neutralDeadband = .001;

	left.reverseLimitSwitchSource = LimitSwitchSource::LimitSwitchSource_FeedbackConnector;
	left.forwardLimitSwitchSource = LimitSwitchSource::LimitSwitchSource_Deactivated;
	left.reverseSoftLimitEnable = true;
	left.reverseSoftLimitThreshold = 0;
	left.forwardSoftLimitEnable = true;
	left.forwardSoftLimitThreshold = 28.0*4096.0/.8;
	left.clearPositionOnLimitR = true;
	left.reverseLimitSwitchNormal = LimitSwitchNormal::LimitSwitchNormal_NormallyOpen;
	left.forwardLimitSwitchNormal = LimitSwitchNormal::LimitSwitchNormal_Disabled;


	left.remoteFilter0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor;
	left.remoteFilter0.remoteSensorDeviceID = _right.GetDeviceID();
	left.sum0Term = FeedbackDevice::CTRE_MagEncoder_Relative;
	left.sum1Term = FeedbackDevice::RemoteSensor0;
	left.auxPIDPolarity = false;
		
	right.primaryPID.selectedFeedbackSensor = FeedbackDevice::CTRE_MagEncoder_Relative;
	right.slot0.kP = .3;
	right.slot0.kI = 0;
	right.slot0.kD = 0;
	right.slot0.kF = 0;
	right.motionAcceleration = 10000;
    right.motionCruiseVelocity = 4096;
    right.neutralDeadband = .001;

	right.reverseLimitSwitchSource = LimitSwitchSource::LimitSwitchSource_FeedbackConnector;
	right.forwardLimitSwitchSource = LimitSwitchSource::LimitSwitchSource_Deactivated;
	right.reverseSoftLimitEnable = true;
	right.reverseSoftLimitThreshold = 0;
	right.forwardSoftLimitEnable = true;
	right.forwardSoftLimitThreshold = 28.0*4096.0/.8;
	right.clearPositionOnLimitR = true;
	right.reverseLimitSwitchNormal = LimitSwitchNormal::LimitSwitchNormal_NormallyOpen;
	right.forwardLimitSwitchNormal = LimitSwitchNormal::LimitSwitchNormal_Disabled;


	ConfigureLeft(left);
	ConfigureRight(right);
*/
	_left.SetSensorPhase(false);
	_left.SetInverted(false);
	_right.SetSensorPhase(false);
	_right.SetInverted(false);
	_left.ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyClosed);
	_right.ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyClosed);
	_left.SetSelectedSensorPosition(0);
	_right.SetSelectedSensorPosition(0);
	
	_left.Config_kF(0, 0);
	_left.Config_kP(0, .8);
	_left.Config_kI(0, 0);
	_left.Config_kD(0, 0);

	_right.Config_kF(0, 0);
	_right.Config_kP(0, .8);
	_right.Config_kI(0, 0);
	_right.Config_kD(0, 0);
	//_left.ConfigRemoteFeedbackFilter(_right.GetDeviceID(), RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor, 0);
	_left.ConfigMotionAcceleration(10000);
	_left.ConfigMotionCruiseVelocity(2000);
	_right.ConfigMotionAcceleration(10000);
	_right.ConfigMotionCruiseVelocity(2000);



	

	_left.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_1_General_, 50);
    _left.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_2_Feedback0_, 50);
	_right.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_1_General_, 50);
    _right.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_2_Feedback0_, 50);

    
}

void Periscope::ConfigureLeft(TalonSRXConfiguration left)
{
	_left.ConfigAllSettings(left);
}

void Periscope::ConfigureRight(TalonSRXConfiguration right)
{
	_right.ConfigAllSettings(right);
}

void Periscope::SetMotorSpeed(double speed)
{
	
	if(!_left.IsRevLimitSwitchClosed())
	{
		_left.SetSelectedSensorPosition(0);
	}
	if(!_right.IsRevLimitSwitchClosed())
	{
		_right.SetSelectedSensorPosition(0);
	}
	if(_left.GetSelectedSensorPosition() == 0 && _right.GetSelectedSensorPosition()==0)
	{
		_homed= true;
	}
	
         _left.Set(ControlMode::PercentOutput,speed);
		 _right.Set(ControlMode::PercentOutput,speed);
}

bool Periscope::Home()
{
	if(!_homed)
	{
		SetMotorSpeed(-1);
		if(_homed)
		{
			SetMotorSpeed(0);
		}
	}	


	return _homed;

}

void Periscope::SetPPos(PPos pos)
{
	if(_homed && !_UsePID)
	{
		_UsePID = true;
		//_left.ConfigSelectedFeedbackSensor(FeedbackDevice::SensorSum);
	}

	if(pos != current)
	{
		current = pos;
		if(posVals[pos][0]<=_left.GetSelectedSensorPosition(0))
		{
			_left.SelectProfileSlot(0,0);
			_right.SelectProfileSlot(0,0);

		}else
		{
			_left.SelectProfileSlot(0,0);
			_right.SelectProfileSlot(0,0);
		}
		_left.Set(ControlMode::MotionMagic, posVals[pos][0]);
		_right.Set(ControlMode::MotionMagic, posVals[pos][0]);
	}
	
}


void Periscope::sendData(std::string name )
{
	frc::SmartDashboard::PutNumber(name + " right position", _right.GetSelectedSensorPosition());
	frc::SmartDashboard::PutNumber(name + " left position", _left.GetSelectedSensorPosition());

	frc::SmartDashboard::PutNumber(name + " left clear", _left.ConfigGetCustomParam(ParamEnum::eClearPositionOnLimitR));
	frc::SmartDashboard::PutNumber(name + " right clear", _right.ConfigGetCustomParam(ParamEnum::eClearPositionOnLimitR));

	frc::SmartDashboard::PutBoolean(name + " limit F LS", _left.GetSensorCollection().IsFwdLimitSwitchClosed());
	frc::SmartDashboard::PutBoolean(name + " limit R LS", _left.GetSensorCollection().IsRevLimitSwitchClosed());
	frc::SmartDashboard::PutBoolean(name + " limit F RS", _right.GetSensorCollection().IsFwdLimitSwitchClosed());
	frc::SmartDashboard::PutBoolean(name + " limit R RS", _right.GetSensorCollection().IsRevLimitSwitchClosed());
}
