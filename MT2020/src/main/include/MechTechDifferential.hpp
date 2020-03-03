

#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

#include "MTPose.hpp"


class MTDifferential
{
    private:
        //Declares the 4 Falcon motors
        TalonFX _rm,
                _rs,
                _lm,
                _ls;
        PigeonIMU _imu;
        
        MTPose _pose;
        frc::Timer _poseTimer{};
        
        double _wheelDiameter;//Size of wheel
        double _wheelBase;//distance from left to right
        double _baseLength;//distance from front to back

        double _encoderTick;//encoder ticks per rev of motor
        double _rEncoderValue;//latest values of encoders
        double _lEncoderValue;//latest values of encoders

        double _gearRatio;//gear ratio of gear box in low
        double _inchPerTick;
        double _ticksPerInch;
        
        double _maxVel;//max velocity in low gear:

        bool _isConfigAuto;
        TalonFXConfiguration _lm_drive;
        TalonFXConfiguration _rm_drive;
        TalonFXConfiguration _lm_auto;
        TalonFXConfiguration _rm_auto;
        BufferedTrajectoryPointStream _autoBuffer1;
        BufferedTrajectoryPointStream _autoBuffer2;



    public:
        //Constructor and Destructor classes for the Mech tech Differential drive
        MTDifferential(int rightmaster, int rightslave, int leftmaster, int leftslave, int imu);
        ~MTDifferential();

        void UpdatePose();
        MTPoseData GetPose();
        void SetPose(MTPoseData pose);

        void SetHeading(double degrees);
        double GetHeading();
        
        void SetSpeed(double leftSpeed, double rightSpeed);
        void SetSpeed(ControlMode mode ,double leftSpeed, double rightSpeed);
        void ArcadeDrive(ControlMode mode ,double transVel, double rotVel);

        void StartArcMotionProfile(int trajLen, const double position[], const double velocity[], const double heading[], double duration);
        void StartMotionProfile(BufferedTrajectoryPointStream& leftStream, BufferedTrajectoryPointStream& rightStream, ControlMode mode);
        bool IsMotionProfileFinished();

        void ResetEncoders();
        void SendData(std::string name = "Drive");

	    void configMotors(TalonFXConfiguration leftMotor, TalonFXConfiguration rightMotor);
        void UseDriveConfig();
        void UseAutoConfig();

        void SetGearRatio(double ratio);
        void SetMaxVel(double maxVel);
        void SetDriveTrainData(double wheelDiameter, double wheelBase, double length);
        void UpdateTickScalers();

        double GetLeftEncoderPosition();
        double GetRightEncoderPosition();

        void ConfigRobot(double diameter, double base, double length, double ticks)
        {
            _wheelDiameter = diameter;
            _wheelBase = base;
            _baseLength = length;
            _encoderTick = ticks;
        }


};