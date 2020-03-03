#include "MechTechDifferential.hpp"

#include "MTMath.h"

using namespace frc;
//Constructor for the Drivetrain class
MTDifferential::MTDifferential(int rightmaster, int rightslave, int leftmaster, int leftslave, int imu) : _rm(rightmaster), 
    _rs(rightslave), _lm(leftmaster), _ls(leftslave), _imu(imu)
{
    _wheelDiameter = 6;//6 inch wheel diameter
    _encoderTick = 2048;//Encoder tick per revolution of motor
    _wheelBase = 27;// Size of drive base from left to right
    _baseLength = 33;//Size of drive base from front to back

    _gearRatio = 10.3896;

    _rEncoderValue = 0;
    _lEncoderValue = 0;

    UpdateTickScalers();

    SupplyCurrentLimitConfiguration currentLimit;
    currentLimit.enable = true;
    currentLimit.triggerThresholdCurrent = 15.0;
    currentLimit.currentLimit = 10.0;

    _lm.ConfigSupplyCurrentLimit(currentLimit);
    _rm.ConfigSupplyCurrentLimit(currentLimit);

    _ls.ConfigSupplyCurrentLimit(currentLimit);
    _rs.ConfigSupplyCurrentLimit(currentLimit);

    _rs.Follow(_rm);
    _ls.Follow(_lm);
    _rm.SetInverted(true);
    _lm.SetInverted(false);
    _rs.SetInverted(InvertType::FollowMaster);
    _ls.SetInverted(InvertType::FollowMaster);


    _lm.Config_kF(0,.045);
    _rm.Config_kF(0,.045);

    _lm.Config_kP(0,.1);//.15
    _rm.Config_kP(0,.1);

    _lm.Config_kI(0,.0);
    _rm.Config_kI(0,.0);

    _lm.Config_kD(0,.0);
    _rm.Config_kD(0,.0);

    _lm.SelectProfileSlot(0,0);
    _rm.SelectProfileSlot(0,0);

    _lm.GetAllConfigs(_lm_drive);
    _rm.GetAllConfigs(_rm_drive);

    _lm_auto = _lm_drive;
    _rm_auto = _rm_drive;

    _lm_auto.primaryPID.selectedFeedbackSensor = FeedbackDevice::SensorDifference;
    /* Auxiliary PID will be RemoteSensor0 which is the Pigeon */
    _lm_auto.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice::RemoteSensor1;
    _lm_auto.neutralDeadband = 0.001; /* 0.1% super small for best low-speed control */

    /* Find these gains in Phoenix Tuner first and later put them here */
    _lm_auto.slot0.kF = _lm_drive.slot0.kF;
    _lm_auto.slot0.kP = _lm_drive.slot0.kP;
    _lm_auto.slot0.kI = _lm_drive.slot0.kI;
    _lm_auto.slot0.kD = _lm_drive.slot0.kD;
    _lm_auto.slot0.integralZone = 400;
    _lm_auto.slot0.closedLoopPeakOutput = 1.0;

    _lm_auto.slot1.kF = 0;
    _lm_auto.slot1.kP = 0.1;
    _lm_auto.slot1.kI = 0.0;
    _lm_auto.slot1.kD = 0.0;
    _lm_auto.slot1.integralZone = 400;
    _lm_auto.slot1.closedLoopPeakOutput = 0.1;

    /* Remote Sensor 0 is the other talon's quadrature encoder */
    _lm_auto.remoteFilter0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_TalonFX_SelectedSensor;
    _lm_auto.remoteFilter0.remoteSensorDeviceID = _rm.GetDeviceID();

    /* Remote Sensor 1 is the Pigeon over CAN */
    _lm_auto.remoteFilter1.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw;
    _lm_auto.remoteFilter1.remoteSensorDeviceID = _imu.GetDeviceNumber();

    /* Configure sensor sum to be this quad encoder and the other talon's encoder */
    _lm_auto.diff0Term = FeedbackDevice::IntegratedSensor;
    _lm_auto.diff1Term = FeedbackDevice::RemoteSensor0;

    /* Configure auxPIDPolarity to match the drive train */
    _lm_auto.auxPIDPolarity = false;

    //Turn down un-needed data on Can Bus
    _ls.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_1_General_, 255);
    _ls.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_2_Feedback0_, 255);
    _rs.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_1_General_, 255);
    _rs.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_2_Feedback0_, 255);
    _imu.SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_RawStatus_4_Mag, 255);
    _imu.SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_6_SensorFusion, 255);
    _imu.SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_11_GyroAccum, 255);

    _poseTimer.Start();

}

//Deconstructor for the Drivetrain class
MTDifferential::~MTDifferential()
{

}

void MTDifferential::ResetEncoders()
{
    _lm.GetSensorCollection().SetIntegratedSensorPosition(0);
    _rm.GetSensorCollection().SetIntegratedSensorPosition(0);
    //_lm.SetSelectedSensorPosition(0,0,0);
    //_rm.SetSelectedSensorPosition(0,0,0);
}

void MTDifferential::SetSpeed(double leftSpeed, double rightSpeed)
{
    if(_isConfigAuto)
    {
        UseDriveConfig();
    }
    _lm.Set(ControlMode::PercentOutput, leftSpeed);
    _rm.Set(ControlMode::PercentOutput, rightSpeed);
}

void MTDifferential::SetSpeed(ControlMode mode ,double leftSpeed, double rightSpeed)
{
    if(_isConfigAuto)
    {
        UseDriveConfig();
    }
    _lm.Set(mode, leftSpeed);
    _rm.Set(mode, rightSpeed);
}
void MTDifferential::ArcadeDrive(ControlMode mode ,double transVel, double rotVel)
{
    if(_isConfigAuto)
    {
        UseDriveConfig();
    }
    rotVel = -rotVel/2.0;
    _lm.Set(mode, (transVel+rotVel));
    _rm.Set(mode, (transVel-rotVel));
}


void MTDifferential::StartMotionProfile(BufferedTrajectoryPointStream& leftStream, BufferedTrajectoryPointStream& rightStream, ControlMode mode )
{
    if(_isConfigAuto)
    {
        UseDriveConfig();
    }

    _lm.StartMotionProfile(leftStream, 10, mode);
    _rm.StartMotionProfile(rightStream, 10, mode);
}
void MTDifferential::StartArcMotionProfile(int trajLen, const double position[], const double velocity[], const double heading[], double duration)
{
    if(!_isConfigAuto)
    {
        UseAutoConfig();
    }
    _autoBuffer1.Clear();
    TrajectoryPoint point;
    for (int i = 0; i < trajLen; ++i) {

        double positionRot = position[i];
        double velocityRPM = velocity[i];
        int durationMilliseconds = (int) duration;

        /* for each point, fill our structure and pass it to API */
        point.timeDur = durationMilliseconds;
        point.position = positionRot * _ticksPerInch; // Convert Revolutions to
                                                         // Units
        point.velocity = velocityRPM * _ticksPerInch / 600.0; // Convert RPM to
                                                                 // Units/100ms
        
        /** 
         * Here is where you specify the heading of the robot at each point. 
         * In this example we're linearly interpolating creating a segment of a circle to follow
         */
        point.auxiliaryPos = heading[i]*10.0; //Linearly interpolate the turn amount to do a circle
        point.auxiliaryVel = 0;


        point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
        point.profileSlotSelect1 = 1; /* which set of gains would you like to use [0,3]? */
        point.zeroPos = (i == 0); /* set this to true on the first point */
        point.isLastPoint = ((i + 1) == trajLen); /* set this to true on the last point */
        point.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */

        point.useAuxPID = true; /* Using auxiliary PID */
        _autoBuffer1.Write(point);
    }

    _lm.StartMotionProfile(_autoBuffer1, 10, ControlMode::MotionProfileArc);

}

bool MTDifferential::IsMotionProfileFinished()
{
    if(_lm.IsMotionProfileFinished() )
    {
        return true;
    }
    return false;
}

void MTDifferential::configMotors(TalonFXConfiguration leftMotor, TalonFXConfiguration rightMotor)
{
    _lm.ConfigAllSettings(leftMotor);
    _rm.ConfigAllSettings(rightMotor);
}

void MTDifferential::UseDriveConfig()
{
    if(_isConfigAuto)
    {
        _isConfigAuto = false;
        configMotors(_lm_drive, _rm_drive);
        
    }
}

void MTDifferential::UseAutoConfig()
{
    if(!_isConfigAuto)
    {
        _isConfigAuto = true;
        _lm.ConfigAllSettings(_lm_auto);
        _rm.Follow(_lm, FollowerType::FollowerType_AuxOutput1);

    }
}


void MTDifferential::SetGearRatio(double ratio)
{
    _gearRatio = ratio;
    UpdateTickScalers();
}

void MTDifferential::SetMaxVel(double maxVel)
{
    _maxVel = maxVel;
    UpdateTickScalers();
}
void MTDifferential::SetDriveTrainData(double wheelDiameter, double wheelBase, double length)
{
    _wheelDiameter = wheelDiameter;
    _wheelBase = wheelBase;
    _baseLength = length;
    UpdateTickScalers();
}

void MTDifferential::UpdateTickScalers()
{
    _inchPerTick = MTMath::InchPerTicks(_wheelDiameter,_gearRatio, _encoderTick);
    _ticksPerInch = MTMath::TicksPerInch(_wheelDiameter, _gearRatio, _encoderTick);
    
    //_ticksPerInch = (ticksPerMotorRev*gearRatio)/(wheelDiameter*3.14159);
}
void MTDifferential::SetHeading(double degrees)
{
    _imu.SetFusedHeading(degrees);
}

double MTDifferential::GetHeading()
{
    return _imu.GetFusedHeading();
}


double MTDifferential::GetLeftEncoderPosition()
{
    return _lm.GetSelectedSensorPosition(0);
}

double MTDifferential::GetRightEncoderPosition()
{
    return _rm.GetSelectedSensorPosition(0);
}

void MTDifferential::UpdatePose()
{
    MTPoseData lastPose = _pose.Get();
    MTPoseData newPose;

    newPose.timestamp = _poseTimer.Get();

    //get current heading
    newPose.phi = MTMath::CleanAngle( MTMath::DegreesToRadians(_imu.GetFusedHeading()));

    //update x and y position as long as we aren't doing a 0 point turn
    double l,r,d, tickScaler;
    
    tickScaler = _inchPerTick;
    

    l = _lm.GetSelectedSensorPosition()*tickScaler;
    r = _rm.GetSelectedSensorPosition()*tickScaler;
    d = ((l-_lEncoderValue)+(r-_rEncoderValue))/2.0;
    //update position based on heading and distance moved.
    newPose.x = lastPose.x + d*cos(newPose.phi);
    newPose.y = lastPose.y + d*sin(newPose.phi);
    
    //store current l and r into old values to be removed next time
    _lEncoderValue = l;
    _rEncoderValue = r;

    double vl = _lm.GetSelectedSensorVelocity()*10.0;//convert from ticks per 100ms to ticks per s
    double vr = _rm.GetSelectedSensorVelocity()*10.0;//convert from ticks per 100ms to ticks per s
    
    newPose.vx = (tickScaler*(vl+vr)/2.0)*cos(newPose.phi);
    newPose.vy = (tickScaler*(vl+vr)/2.0)*sin(newPose.phi);
    newPose.w = (newPose.phi-lastPose.phi)/(newPose.timestamp-lastPose.timestamp);

    newPose.ax = (newPose.vx-lastPose.vx)/(newPose.timestamp-lastPose.timestamp);
    newPose.ay = (newPose.vy-lastPose.vy)/(newPose.timestamp-lastPose.timestamp);
    newPose.aa = (newPose.w-lastPose.w)/(newPose.timestamp-lastPose.timestamp);

    SetPose(newPose);

/*//old 2018 code
    //grab euler angles from imu and store in member vector.
		eulers = imu.GetVector(BNO055::vector_type_t::VECTOR_EULER);

		//convert yaw angle to radians
		double phi = -CleanAngle(3.14159*eulers.x/180.0);
		phi = CleanAngle(phi-headingOffset); //apply offset

		double l,r,d;
		//update x and y position as long as we aren't doing a 0 point turn
		if(!turning)
		{
			//get the distance traveled by each wheel
			l = 3.14159*4.0*(drive.GetLeftEncoderPosition()/1440.0);
			r = 3.14159*4.0*(drive.GetRightEncoderPosition()/1440.0);

			//remove old position, so we are only adding the change in distance
			//find the distance the CENTER of the robot moved
			d = ((l-oldLDrivePos)+(r-oldRDrivePos))/2.0;

			if(useSimData) //used to simulate robot moving
			{
				drivePos.SetX(simx);
				drivePos.SetY(simy);
			}else
			{
				//update position based on heading and distance moved.
				drivePos.SetX( drivePos.GetX() + d*cos(phi));
				drivePos.SetY(drivePos.GetY() + d*sin(phi));
			}
			//store current l and r into old values to be removed next time
			oldLDrivePos = l;
			oldRDrivePos = r;
		}else
		{
			drive.ResetEncoders();
			oldLDrivePos = drive.GetLeftEncoderPosition();
			oldRDrivePos = drive.GetRightEncoderPosition();
		}


		drivePos.SetPhi(phi);
*/
}

MTPoseData MTDifferential::GetPose()
{
    return _pose.Get();
}

void MTDifferential::SetPose(MTPoseData pose)
{
    _pose.Update(pose);
}

void MTDifferential::SendData(std::string name)
{

    MTPoseData pos = GetPose();

    frc::SmartDashboard::PutNumber("Drive Pose X", pos.x);
    frc::SmartDashboard::PutNumber("Drive Pose Y", pos.y);
    frc::SmartDashboard::PutNumber("Drive Pose phi (D)", MTMath::RadiansToDegrees( pos.phi));
    frc::SmartDashboard::PutNumber("Drive Pose vX", pos.vx);
    frc::SmartDashboard::PutNumber("Drive Pose vY", pos.vy);
    frc::SmartDashboard::PutNumber("Drive Pose w", pos.w);
    frc::SmartDashboard::PutNumber("Drive Pose AX", pos.ax);
    frc::SmartDashboard::PutNumber("Drive Pose AY", pos.ay);
    frc::SmartDashboard::PutNumber("Drive Pose AA", pos.aa);
    frc::SmartDashboard::PutNumber("Drive Pose timestamp", pos.timestamp);

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

    frc::SmartDashboard::PutNumber("Drive IMU Heading", _imu.GetFusedHeading());

    frc::SmartDashboard::PutNumber("Left Drive Motor Speed", _lm.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("Right Drive Motor Speed", _rm.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("Left Drive Motor Position", _lm.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Right Drive Motor Position", _rm.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Left Drive Motor Temp (C)", _lm.GetTemperature());
    frc::SmartDashboard::PutNumber("Right Drive Motor Temp (C)", _rm.GetTemperature());

    frc::SmartDashboard::PutNumber("ControlMode", (int)_lm.GetControlMode());
    frc::SmartDashboard::PutBoolean("Left Path Finished", _lm.IsMotionProfileFinished());
    frc::SmartDashboard::PutBoolean("Right Path Finished", _rm.IsMotionProfileFinished());

    if( _lm.GetControlMode() == ControlMode::Velocity ||
        _lm.GetControlMode() == ControlMode::Position ||
        _lm.GetControlMode() == ControlMode::MotionProfile ||
        _lm.GetControlMode() == ControlMode::MotionProfileArc ||
        _lm.GetControlMode() == ControlMode::MotionMagic
        )
    {
        frc::SmartDashboard::PutNumber("Left PID Error", _lm.GetClosedLoopError());
        frc::SmartDashboard::PutNumber("Right PID Error", _rm.GetClosedLoopError());
        frc::SmartDashboard::PutNumber("Left Derivative Error", _lm.GetErrorDerivative());
        frc::SmartDashboard::PutNumber("Right Derivative Error", _rm.GetErrorDerivative());
        frc::SmartDashboard::PutNumber("Left Intergral Error", _lm.GetIntegralAccumulator());
        frc::SmartDashboard::PutNumber("Right Intergral Error", _rm.GetIntegralAccumulator());
    }else
    {
        frc::SmartDashboard::PutNumber("Left PID Error", 0);
        frc::SmartDashboard::PutNumber("Right PID Error", 0);
        frc::SmartDashboard::PutNumber("Left Derivative Error", 0);
        frc::SmartDashboard::PutNumber("Right Derivative Error", 0);
        frc::SmartDashboard::PutNumber("Left Intergral Error", 0);
        frc::SmartDashboard::PutNumber("Right Intergral Error", 0);
    }
    frc::SmartDashboard::PutBoolean("Path Finished", IsMotionProfileFinished());
}

