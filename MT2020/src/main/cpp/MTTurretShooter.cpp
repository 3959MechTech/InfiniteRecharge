/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "MTTurretShooter.h"
#include "MTMath.h"


MTTurretShooter::MTTurretShooter(int leftMotor, int rightMotor, int turretMotor, int imu): _lm(leftMotor),
    _rm(rightMotor), 
    _turretMotor(turretMotor),
    _imu(imu)
{
//    _autoTrack = false;
//    _threadRunning = false;
    
    _lm.Follow(_rm);

    _rm.SetInverted(false);
    _lm.SetInverted(InvertType::OpposeMaster);//could be FollowMaster
    
    _rm.Config_kF(0,.05,0);//old .04778
    _rm.Config_kP(0,.06,0);
    _rm.Config_kI(0,.0,0);
    _rm.Config_kD(0,.0,0);

    TalonSRXConfiguration turretMotorConfig;
    //pigeon is 8192 ticks per rev
    turretMotorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw;
    turretMotorConfig.remoteFilter0.remoteSensorDeviceID = _imu.GetDeviceNumber();

    //turretMotorConfig.primaryPID.selectedFeedbackCoefficient = 3600.0/8192.0;
    turretMotorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice::RemoteSensor0;
    turretMotorConfig.slot0.kP = 1.0;
    turretMotorConfig.slot0.kI = .01;
    turretMotorConfig.slot0.maxIntegralAccumulator = .2;
    turretMotorConfig.slot0.allowableClosedloopError = 4;
    
    turretMotorConfig.motionAcceleration = 8000;
    turretMotorConfig.motionCruiseVelocity = 900;

    turretMotorConfig.neutralDeadband = .001;
    

    _turretMotor.ConfigAllSettings(turretMotorConfig);

    _imu.SetYaw(0);


    //Turn down un-needed data on Can Bus
    _lm.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_1_General_, 255);
    _lm.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_2_Feedback0_, 255);
    _imu.SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_RawStatus_4_Mag, 255);
    _imu.SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_6_SensorFusion, 255);
    _imu.SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_11_GyroAccum, 255);

    _imu.SetFusedHeading(0);
    

}

MTTurretShooter::~MTTurretShooter()
{
    
}


bool MTTurretShooter::isShooterReady()
{
    if(abs(_rm.GetClosedLoopError())<250 )
        return true;
    else 
        return false;
}

void MTTurretShooter::zeroTurret()
{
    _imu.SetYaw(0);
}

double MTTurretShooter::GetHeading()
{
    return _imu.GetFusedHeading();
}

void MTTurretShooter::spin(double speed)
{
    _turretMotor.Set(ControlMode::PercentOutput, speed);
}
void MTTurretShooter::setWheelSpeed(double speed)
{
    _targetWheelSpeed = speed;
    if(speed < 1.0 && speed > -1.0)
    {
        _rm.Set(ControlMode::PercentOutput, 0.0);
    }else
    {
        _rm.Set(ControlMode::Velocity, _targetWheelSpeed);    
    }
}

void MTTurretShooter::sendData(std::string name)
{
    frc::SmartDashboard::PutNumber(name + " heading", _imu.GetFusedHeading());
    switch(_imu.GetState())
    {
        case PigeonIMU::PigeonState::Initializing: 
            frc::SmartDashboard::PutString(name + " State", "Init");
            break;
        case PigeonIMU::PigeonState::NoComm:  
            frc::SmartDashboard::PutString(name + " State", "NoComm");
            break;
        case PigeonIMU::PigeonState::Ready:  
            frc::SmartDashboard::PutString(name + " State", "Ready");
            break;
        case PigeonIMU::PigeonState::UserCalibration:  
            frc::SmartDashboard::PutString(name + " State", "UserCalibration");
            break;
        default:
            frc::SmartDashboard::PutString(name + " State", "unk");

    }
    //frc::SmartDashboard::PutNumber(name + " heading", _imu.GetState().toString());
    frc::SmartDashboard::PutNumber(name + " Turret heading", _turretMotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber(name + " Turret Target", _turretMotor.GetClosedLoopTarget());
    frc::SmartDashboard::PutNumber(name + " Turret Error", _turretMotor.GetClosedLoopError());
    frc::SmartDashboard::PutNumber(name + " Speed", _rm.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber(name + " Speed Error", _rm.GetClosedLoopError());
    frc::SmartDashboard::PutNumber(name + " Temp (C)", _rm.GetTemperature());
    frc::SmartDashboard::PutBoolean(name + " Ready", isShooterReady());


} // namespace name

void MTTurretShooter::updateTargetHeading(double offset)
{
    //if(offset>=0.0)
    //{
        _targetHeading = _turretMotor.GetSelectedSensorPosition() - offset*(8192.0/360.0);
    /*}else
    {
        _targetHeading = 1800.0 - offset*10.0 ;
    }*/
    if(_targetHeading > -900 && _targetHeading <900)
    {
        _turretMotor.Set(ControlMode::Position, _targetHeading);
    }
}

void MTTurretShooter::track(MTPoseData drivePose, double x_angle, double y_angle)
{
    double turretHeading = _turretMotor.GetSelectedSensorPosition();
    double angle = turretHeading - x_angle*10;
    angle -= MTMath::RadiansToDegrees(MTMath::CleanAngle(drivePose.phi-M_PI));

    //if(angle<_turretRightLimit && angle > _turretLeftLimit)
    {
        _targetHeading = _turretMotor.GetSelectedSensorPosition() - x_angle*10.0;
        _turretMotor.Set(ControlMode::MotionMagic, _targetHeading);
    }

}

/*
void MTTurretShooter::threadTrack()
{
    _mutex.lock();
    _threadRunning = true;
    _mutex.unlock();

    while (isAutoTracking())
    {
        track();
    }
    
    //let the world know we died
    _mutex.lock();
    _threadRunning = false;
    _mutex.unlock();

}
*/


