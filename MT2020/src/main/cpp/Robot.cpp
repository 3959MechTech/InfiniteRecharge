/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
#include <unistd.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "TrenchRun_TrenchRun1.h"
#include "TrenchRun_TrenchRun2.h"


#include "AutoPaths/Straight_f1.h"    //move forward 53"

#include "AutoPaths/SideTrench_f1.h"  //for right wall starting line
#include "AutoPaths/SideTrench_f2.h"  //move to TrenchRun_r2 after this 
#include "AutoPaths/TrenchRun_f1.h"   //setup infront of target
#include "AutoPaths/TrenchRun_r2.h"
#include "AutoPaths/TrenchRun_f3.h"
#include "AutoPaths/TrenchRun_r4.h"

#include "AutoPaths/TrenchNinja_f1.h"   //setup on enemy trench balls
#include "AutoPaths/TrenchNinja_r2t.h"  //move to TrenchRun_F1 after this path
#include "AutoPaths/TrenchNinja_r2g.h"  //Generator run
#include "AutoPaths/TrenchNinja_f3g.h"
#include "AutoPaths/TrenchNinja_r4g.h"
#include "AutoPaths/TrenchNinja_f5g.h"
#include "AutoPaths/TrenchNinja_r6g.h"




/*
INDEXER
Front Index (Device ID 30)
Mid Index (Device ID 31)
Back Index (Device ID 32)

FEEDER 
Intake (Device ID 60)

PERISCOPE
Left Climber (Device ID 50)
Right Climber (Device ID 51)

PIGEON IMU
Base Pigeon (Device ID 1)
Shooter Pigeon (Device ID 2)

SHOOTER
Left Shooter (Device ID 20)
Right Shooter (Device ID 21)
Shooter Turret (Device ID 22)

DRIVETRAIN
LeftMaster (Device ID 11)
LeftSlave (Device ID 13)
RightMaster (Device ID 10)
RightSlave (Device ID 12)

OTHER DEVICES
PCM (Device ID 5)
PDP (Device ID 0)

currently no probe ID
*/

void Robot::RobotInit() 
{
    m_fireModeChooser.SetDefaultOption(kAutoWait, kAutoWait);
    m_fireModeChooser.AddOption(kAutoFireOnRun, kAutoFireOnRun);
    frc::SmartDashboard::PutData("Fire Mode", &m_fireModeChooser);

    m_routineChooser.SetDefaultOption(kAutoNameTrenchRun3, kAutoNameTrenchRun3);
    m_routineChooser.AddOption(kAutoNameStraight, kAutoNameStraight);
    m_routineChooser.AddOption(kAutoNameTrenchRun1, kAutoNameTrenchRun1);
    m_routineChooser.AddOption(kAutoNameTrenchRun2, kAutoNameTrenchRun2);
    m_routineChooser.AddOption(kAutoNameTrenchRun4, kAutoNameTrenchRun4);
    m_routineChooser.AddOption(kAutoNameTrenchNinja1, kAutoNameTrenchNinja1);
    m_routineChooser.AddOption(kAutoNameTrenchNinja2, kAutoNameTrenchNinja2);
    m_routineChooser.AddOption(kAutoNameTrenchNinja3, kAutoNameTrenchNinja3);
    m_routineChooser.AddOption(kAutoNameDev, kAutoNameDev);
    frc::SmartDashboard::PutData("Select Routine", &m_routineChooser);

    _autoWaitForShooter = true;
    _routine = AutoRoutine::TrenchRun6;

    _lowSpeed = false;

    //Initializes a smart timer to begin
    smTimer.Start();

    drive.ConfigRobot(6.0,27.0,33.0,2048.0);

    _telemetryThread = new std::thread(&Robot::sendData, this);
    
    _PoseThread = new frc::Notifier(&Robot::updatePose, this);
    _PoseThread->StartPeriodic(.010);
    _PoseTimer.Start();

    _shooterThread = new frc::Notifier(&Robot::autoTrack, this);
    _shooterThread->StartPeriodic(.010);
    _shooterTimer.Start();
    
    shooterSpeedSelect = 0.0;
    _autotrack = true;
    _lastAutoTrack = _autotrack;

    _feederDown = false;
    _feederPiston.Set(_feederDown);

    //indexerS1Timer.Start();
    _useAutoIndex = true;
    if(_useAutoIndex)
    {
        indexer.SetIndexState(Indexer::IndexState::AutoLoad);
    }else
    {
        indexer.SetIndexState(Indexer::IndexState::FullStop);
    }
    
    _climbing = true;

    //cam.EnableTermination();
    //cam.SetReadBufferSize(16);
    //cam.SetTimeout(1.0);
    //cam1.DisableTermination();
    //cam1.SetReadBufferSize(5);
    //cam1.SetTimeout(1.0);
    //cam2.DisableTermination();
    //cam2.SetReadBufferSize(5);
    //cam2.SetTimeout(1.0);

    NetTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

}

void Robot::toggleFeeder()
{
    setFeeder(!_feederDown);
}

void Robot::setFeeder(bool down )
{
    _feederDown = down;
    _feederPiston.Set(_feederDown);
    if(_feederDown)
    {
        _feederDownTime.Start();
        _feederDownTime.Reset();
    }else
    {
        _feederDownTime.Stop();
        _feederDownTime.Reset();
    }
    
    if(!_feederDown)
    {
        periscope.SetMotorSpeed(0.0);
    }
}

void Robot::updatePose()
{
    drive.UpdatePose();
    _poseThreadMutex.lock();
    _poseThreadPeriod = _PoseTimer.Get();
    _poseThreadMutex.unlock();
    _PoseTimer.Reset();
}

void Robot::sendData()
{
    usleep(100000);//give system time to finish starting up.
    _telemetryTimer.Start();
    while (!_stopTelemetryThread)
    {
        frc::SmartDashboard::PutNumber("state", state);
        //char data[128];
        //int recved = cam.Read(data, 127);
        //data[recved] = '\0';
        //frc::SmartDashboard::PutNumber("cam Data len", recved);
        //frc::SmartDashboard::PutString("cam Data", std::string(data));
        //int rxLen = cam.GetBytesReceived();
        //frc::SmartDashboard::PutNumber("cam Data len", rxLen);
/*
        if(rxLen>12)
        {
            char* buf = new char[14];
            cam.Read(buf, 14);
            frc::SmartDashboard::PutString("UART", buf);
        }
*/
/*
        recved = cam1.Read(data, 127);
        data[recved] = '\0';
        frc::SmartDashboard::PutNumber("cam1 Data len", recved);
        frc::SmartDashboard::PutString("cam1 Data", std::string(data));

        recved = cam2.Read(data, 127);
        data[recved] = '\0';
        frc::SmartDashboard::PutNumber("cam2 Data len", recved);
        frc::SmartDashboard::PutString("cam2 Data", std::string(data));
*/
        
        frc::SmartDashboard::PutNumber("limelight X", NetTable->GetNumber("tx", 0.0));
        frc::SmartDashboard::PutNumber("limelight Y", NetTable->GetNumber("ty", 0.0));
        frc::SmartDashboard::PutNumber("limelight area", NetTable->GetNumber("ta", 0.0));
        frc::SmartDashboard::PutNumber("limelight skew", NetTable->GetNumber("ts", 0.0));

        double targetangle;
        _shooterThreadMutex.lock_shared();
        targetangle = _targetAngle;
        _shooterThreadMutex.unlock_shared();

        frc::SmartDashboard::PutNumber("Angle to Target", targetangle);

        double posePeriod;
        _poseThreadMutex.lock_shared();
        posePeriod = _poseThreadPeriod;
        _poseThreadMutex.unlock_shared();
        frc::SmartDashboard::PutNumber("Pose Thread Period", posePeriod);

        shooter.sendData();
        indexer.SendData();
        periscope.sendData();
        feeder.SendData();
        drive.SendData();

        frc::SmartDashboard::PutBoolean("Climb State", _climbing);
        frc::SmartDashboard::PutNumber("Match Time", DriverStation::GetInstance().GetMatchTime());
        frc::SmartDashboard::PutBoolean("FeederDown", _feederDown);
        frc::SmartDashboard::PutNumber("FeederDownTime", _feederDownTime.Get());

        frc::SmartDashboard::PutNumber("auto state", state);
        frc::SmartDashboard::PutNumber("auto Timer", timer.Get());
        frc::SmartDashboard::PutNumber("Telemetery Thread Period", _telemetryTimer.Get());
        _telemetryTimer.Reset();       
        usleep(10000);//sleep for 20ms aka 20,000 us
    }
    
}

void Robot::autoTrack()
{
    if(frc::DriverStation::GetInstance().IsDisabled())
    {
        return;
    }
    /*
    if(_autotrack != _lastAutoTrack)
    {
        shooter.setTargetHeading(shooter.GetHeading());
        _lastAutoTrack = _autotrack;
    }
    */
    MTPoseData pose = drive.GetPose();
    bool track = false; 
    _shooterThreadMutex.lock();
    track = _autotrack;
    _targetAngle = std::atan2(226.81-pose.y,0-pose.x)-pose.phi;
    _shooterThreadMutex.unlock();
    
    if(DriverStation::GetInstance().GetMatchTime()<35.0)
    {
        _climbing = true;
    }
    
    double area = NetTable->GetNumber("ta", 0.0);
    int valid = NetTable->GetNumber("tv", 0.0);

    
    if(track )
    {
        shooter.updateTargetHeading(NetTable->GetNumber("tx", 0.0));
    }    

    
    
    
    //shooter.track(pose, NetTable->GetNumber("tx", 0.0), NetTable->GetNumber("ty", 0.0));

}

void Robot::indexerS1()
{
    if(indexerS1Timer.Get()<=1.0)
    {
        indexer.SetM1(.5);
    }else
    {
        if(indexerS1Timer.Get()>1.0 && indexerS1Timer.Get()<1.25)
        {
            indexer.SetM1(0.0);
        }else
        {
            indexerS1Timer.Reset();
        }
    }
}

void Robot::devStick()
{
    shooter.setWheelSpeed(1000);
    setFeeder();
    if(stick3.GetXButton())
    {
        indexer.SetIndexState(Indexer::IndexState::AutoLoad);
    }

    if(stick3.GetAButton())
    {
        indexer.SetIndexState(Indexer::IndexState::FullStop);
        //shooter.setWheelSpeed(18000.0);
        //shooterSpeedSelect = 18000.0;
        //shooter.zeroTurret();
    }

    if(stick3.GetBButton())
    {
        indexer.SetIndexState(Indexer::IndexState::Eject);
        //shooter.setWheelSpeed(15000.0);
        //shooterSpeedSelect = 15000.0;
    }

    if(stick3.GetYButton())
    {
        indexer.SetIndexState(Indexer::IndexState::Fire);
        //shooter.setWheelSpeed(13500.0);
        //shooterSpeedSelect = 13500.0;
    }
    indexer.AutoIndex();
/*
    if(stick3.GetStartButtonPressed())
    {
        shooter.setWheelSpeed(0.0);
    }
    if(stick3.GetBackButtonPressed())
    {
        shooter.setWheelSpeed(0.0);
        shooterSpeedSelect = 0.0;
        indexer.DirectDrive(0,0,0);
        feeder.Feed(0);
    }

    if(stick3.GetBumperPressed(frc::GenericHID::kRightHand))
    {
        shooterSpeedSelect += 100.0;
        shooter.setWheelSpeed(shooterSpeedSelect);
    }
    
    if(stick3.GetBumperPressed(frc::GenericHID::kLeftHand))
    {
        shooterSpeedSelect -= 100.0;
        shooter.setWheelSpeed(shooterSpeedSelect);
    }

    

    if(stick3.GetTriggerAxis(frc::GenericHID::kRightHand)>.1)
    {
        shooter.setWheelSpeed(stick3.GetTriggerAxis(frc::GenericHID::kRightHand)*19000.0);
    }else
    {  
        //shooter.setWheelSpeed(0);
    }
    
    
    if(stick3.GetTriggerAxis(frc::GenericHID::kLeftHand)>.1)
    {
        //feeder.Feed(stick3.GetTriggerAxis(frc::GenericHID::kLeftHand)*.75);
    }else
    {
        //feeder.Feed(0);
    }

    if(!_autotrack)
    {
        shooter.spin(stick3.GetX(frc::GenericHID::kRightHand));
    }
*/
    /*
    if(periscope.Home())
    {
        periscope.SetMotorSpeed(-stick3.GetY(frc::GenericHID::kRightHand));
    }
    */
    //periscope.SetMotorSpeed(-stick3.GetY(frc::GenericHID::kRightHand));
    //shooter.spin(stick3.GetX(frc::GenericHID::kLeftHand)/2.0);
    //autoTrack();
}

void Robot::executeTasks()
{
    stick.GetY(frc::GenericHID::kLeftHand);
    stick.GetY(frc::GenericHID::kRightHand);

    double v = -stick.GetY(frc::GenericHID::kLeftHand);
    double w = stick.GetX(frc::GenericHID::kRightHand);

    //rm.Set( (2.0*v-w)/2.0 );
    //lm.Set( (2.0*v+w)/2.0 );
    //rs.Set( (2.0*v-w)/2.0 );
    //ls.Set( (2.0*v+w)/2.0 );

    double lY = -stick.GetY(frc::GenericHID::kLeftHand);
    double rY = -stick.GetY(frc::GenericHID::kRightHand);

    if(lY>-.2 && lY<.2)
    {
        lY = 0.0;
    }
    if(rY>-.2 && rY<.2)
    {
        rY = 0.0;
    }
    double lX = -stick.GetX(frc::GenericHID::kLeftHand);
    double rX = -stick.GetX(frc::GenericHID::kRightHand);

    if(lX>-.2 && lX<.2)
    {
        lX = 0.0;
    }
    if(rX>-.2 && rX<.2)
    {
        rX = 0.0;
    }

    if(stick.GetBumper(frc::GenericHID::kLeftHand))
    {
        _lowSpeed = !_lowSpeed;
    }

    if(_lowSpeed)
    {
        drive.ArcadeDrive(ControlMode::PercentOutput, lY*.15, rX*.25);
    }else
    {
        drive.ArcadeDrive(ControlMode::PercentOutput, lY*.75, rX*.75);
    }
    
    
    if(_climbing)
    {
        if(stick.GetAButtonPressed())
        {
            periscope.SetPPos(Periscope::Down);
        }
        if(stick.GetBButtonPressed())
        {
            periscope.SetPPos(Periscope::Low);
        }
        if(stick.GetXButtonPressed())
        {
            periscope.SetPPos(Periscope::Level);
        }
        if(stick.GetYButtonPressed())
        {
            periscope.SetPPos(Periscope::High);
        }
        if(stick.GetPOV()==0)
        {
            periscope.SetPosition(periscope.GetEncoderPos()+3000.0);
        }
        if(stick.GetPOV()==180)
        {
            periscope.SetPosition(periscope.GetEncoderPos()-3000.0);
        }
    }

    bool at;
    _shooterThreadMutex.lock_shared();
    at = _autotrack;
    _shooterThreadMutex.unlock_shared();

    if(stick2.GetPOV()==0)
    {
        if(!at)
        {
            _shooterThreadMutex.lock();
            _autotrack = true;
            _shooterThreadMutex.unlock();
            NetTable->PutNumber("camMode", 0);
            NetTable->PutNumber("ledMode", 0);
        }
    }
    if(stick2.GetPOV()==180)
    {
        if(at)
        {
            _shooterThreadMutex.lock();
            _autotrack = false;
            _shooterThreadMutex.unlock();
            NetTable->PutNumber("camMode", 1);
            NetTable->PutNumber("ledMode", 1);
        }
    }
    

    if(!at)
    {
        if(fabs(stick2.GetX(frc::GenericHID::kRightHand))>.1)
        {
            shooter.spin(stick2.GetX(frc::GenericHID::kRightHand));
        }else
        {
            shooter.spin(0);
        }
        

    }

    if(stick2.GetAButtonPressed())
    {
        shooter.setWheelSpeed(13500);
    }
    if(stick2.GetBButtonPressed())
    {
        shooter.setWheelSpeed(17000);
    }
    if(stick2.GetYButtonPressed())
    {
        shooter.setWheelSpeed(20000);
    }
    if(stick2.GetXButtonPressed())
    {
        shooter.setWheelSpeed(14500);
    }

    if(stick2.GetBumperPressed(frc::GenericHID::kRightHand)||stick.GetBumperPressed(frc::GenericHID::kRightHand))
    {
        toggleFeeder();
    }

    if(stick2.GetTriggerAxis(frc::GenericHID::kRightHand)>.1)
    {
        if(_feederDown)
        {
            feeder.Feed(stick2.GetTriggerAxis(frc::GenericHID::kRightHand)*.75);
        }
    }else
    {
        if(stick.GetTriggerAxis(frc::GenericHID::kRightHand)>.1)
        {
            if(_feederDown)
            {
                feeder.Feed(stick.GetTriggerAxis(frc::GenericHID::kRightHand)*.45);
            }
        }else
        {
            if(stick2.GetStartButtonPressed())
            {
                feeder.Eject(1.0);
            }else
            {
                feeder.Feed(-0.1);    
            }
        }
    }
    if(!_feederDown)
    {
        feeder.Feed(0.0);
    }
    
    
    
    if(stick2.GetBumper(frc::GenericHID::kLeftHand))
    {
        indexer.SetIndexState(Indexer::IndexState::Fire);
    }else
    {
        if(stick2.GetTriggerAxis(frc::GenericHID::kLeftHand)>.1)
        {
            indexer.SetIndexState(Indexer::IndexState::Fire);
        }else
        {
            if(_useAutoIndex)
            {
                indexer.SetIndexState(Indexer::IndexState::AutoLoad);
            }else
            {
                indexer.SetIndexState(Indexer::IndexState::FullStop);
            }
        }
    }
    if(stick2.GetPOV()>255 && stick2.GetPOV()<285)
    {
        _useAutoIndex = !_useAutoIndex;
    }

    

    if(stick2.GetBackButton())
    {
        indexer.SetIndexState(Indexer::IndexState::Eject);
    }
}

void Robot::RobotPeriodic() {
  if(smTimer.Get()>.02)
  {
    smTimer.Reset();
    //drive.SendData();
  }
  /*
  if(stick.GetPOV()==0)
  {
      _shooterThreadMutex.lock();
      _autotrack = false;
      _shooterThreadMutex.unlock();
  }
  */
    indexer.AutoIndex();

}

void Robot::AutonomousInit() {
    m_autoSelected = m_routineChooser.GetSelected();
    m_autoFireModeSelected = m_fireModeChooser.GetSelected();
    std::cout << "Auto selected: " << m_autoSelected << std::endl;
    _autoWaitForShooter = true;
    _routine = AutoRoutine::TrenchRun8;

    if (m_autoSelected == kAutoNameStraight)
        _routine = AutoRoutine::Straight;
    if (m_autoSelected == kAutoNameTrenchRun1)//side trench6
        _routine = AutoRoutine::SideTrench;
    if (m_autoSelected == kAutoNameTrenchRun2)//target trench6
        _routine = AutoRoutine::TrenchRun6;
    if (m_autoSelected == kAutoNameTrenchRun3)//side trench8
        _routine = AutoRoutine::SideTrench8;
    if (m_autoSelected == kAutoNameTrenchRun4)//target trench8
        _routine = AutoRoutine::TrenchRun8;
    if (m_autoSelected == kAutoNameTrenchNinja1)//ninja 5
        _routine = AutoRoutine::TrenchNinja5;
    if (m_autoSelected == kAutoNameTrenchNinja2)//ninja + trench
        _routine = AutoRoutine::TrenchNinjaTrench;
    if (m_autoSelected == kAutoNameTrenchNinja3)//ninja + gen
        _routine = AutoRoutine::TrenchNinjaGenerator8;
    if (m_autoSelected == kAutoNameDev)//ninja + gen
        _routine = AutoRoutine::Dev;
    
    state = 0;

    setFeeder();
    NetTable->PutNumber("camMode", 0);
    NetTable->PutNumber("ledMode", 0);
    //_autotrack = true;

    drive.SetHeading(0);
    shooter.zeroTurret();
    _shooterThreadMutex.lock();
    _autotrack = false;
    _shooterThreadMutex.unlock();
}

void Robot::AutonomousPeriodic() {
    switch (_routine)
    {
    case AutoRoutine::Straight:
        AutoStraight();
        break;
    case AutoRoutine::SideTrench:
        AutoTrench2();
        break;
    case AutoRoutine::SideTrench8:
        AutoTrench2();
        break;    
    case AutoRoutine::TrenchRun6:
        AutoOldTrench();
        break;
    case AutoRoutine::TrenchRun8:
        AutoTrench1();
        break;
    case AutoRoutine::TrenchNinja5:
        AutoTrenchNinja1();
        break;
    case AutoRoutine::TrenchNinjaTrench:
        AutoTrenchNinja1();
        break;
    case AutoRoutine::TrenchNinjaGenerator8:
        AutoTrenchNinja1();
        break;
    case Dev:
        AutoTrenchNinja3();
    default:
        break;
    }

}

void Robot::TeleopInit() {
  //drive.ResetEncoders();

    if(_autotrack)
    {
        NetTable->PutNumber("ledMode", 0);
    }
    indexer.SetIndexState(Indexer::IndexState::AutoLoad);
    state  = 0;
    _climbing = false;
}

void Robot::TeleopPeriodic() {
  
    //devStick();
    executeTasks();
    indexer.AutoIndex();
    //shooter.setWheelSpeed(3000);
    
    //frc::SmartDashboard::PutNumber("rv", rv);

    //A button = serpintine
    //B button = right turn
    //X button = left turn
    //Y button = straight

    
    
}

void Robot::TestPeriodic() {}
void Robot::DisabledInit() 
{
    NetTable->PutNumber("ledMode", 1);
}

void Robot::AutoStraight()
{
    switch (state)
    {
    case 0:
        timer.Start();
        timer.Reset();//reset timer
        shooter.setWheelSpeed(14000);
        drive.ResetEncoders();
        setFeeder(false);
        state++;
        break;
    case 1:
        if(timer.Get()>.75){indexer.SetIndexState(Indexer::IndexState::Fire);}
        if(timer.Get()>1.5){state++;}
        if(!_autoWaitForShooter){state++;}
        break;
    case 3:
        drive.ResetEncoders();
        tragTool.GetStreamFromArray(left_bufferedStream, Straight_f1Points, Straight_f1Len, true);
        tragTool.GetStreamFromArray(right_bufferedStream, Straight_f1Points, Straight_f1Len, false);
        drive.StartMotionProfile(right_bufferedStream, left_bufferedStream, ControlMode::MotionProfile); 
    /*
        drive.StartArcMotionProfile(Straight_f1Len, 
                                    Straight_f1Points[MTPath::column2::position_center_v2],
                                    Straight_f1Points[MTPath::column2::velocity_center_v2],
                                    Straight_f1Points[MTPath::column2::heading_v2],
                                    Straight_f1Points[0][MTPath::column2::duration_v2]
                                    );
    */
        state++;
        break;
    case 4: 
        if(drive.IsMotionProfileFinished()){state++;}
        break;
    default:
        break;
    }
}
void Robot::AutoOldTrench()
{
  //Old Code 
    switch (state)
    {
    case 0: //setup
        timer.Start();
        timer.Reset();//reset timer
        shooter.setWheelSpeed(15000);
        _shooterThreadMutex.lock();
        _autotrack = false;
        _shooterThreadMutex.unlock();
        shooter.setTargetHeading(0);
        //indexer.DirectDrive(.5,.3,1.0);
        //zero the encoders and state machine variable
        drive.ResetEncoders();
        //load trajectory into buffer
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, true, true);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, false, true);
        state++;
    case 1: 
        if(timer.Get()>1.0){indexer.SetIndexState(Indexer::IndexState::Fire);}
        if(timer.Get()>2.5){state++;}
        if(!_autoWaitForShooter){state++;}
        //state++;
        break;
    case 2://start traj
        _shooterThreadMutex.lock();
        _autotrack = true;
        _shooterThreadMutex.unlock();
        drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
        state++;
        break;
    case 3: //wait for traj to finish
        feeder.Feed(AutoFeederSpeed);
        indexer.SetIndexState(Indexer::IndexState::AutoLoad);
        if(drive.IsMotionProfileFinished()){state++;}
        shooter.setWheelSpeed(16000);
        break;
    case 4: //prepare to reverse traj
        drive.ResetEncoders();
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, true, false);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, false, false);
        drive.StartMotionProfile(right_bufferedStream, left_bufferedStream, ControlMode::MotionProfile); 
        timer.Reset();
        timer.Start();
        state++;
        break;
    case 5: 
        if(drive.IsMotionProfileFinished()){state++;timer.Reset();}
        break;
    case 6: //start rev traj
        if(timer.Get()>.50){state++;}//wait 2 seconds
        indexer.SetIndexState(Indexer::IndexState::Fire);
        
        break;
    case 7: //start rev traj
        
        state++;
        break;
    default:
        break;
    }


}

void Robot::AutoTrench1()
{
    switch (state)
    {
    case 0: //setup
        timer.Start();
        timer.Reset();//reset timer
        shooter.setWheelSpeed(15000);
        feeder.Feed(AutoFeederSpeed);
        //indexer.DirectDrive(.5,.3,1.0);
        //zero the encoders and state machine variable
        drive.ResetEncoders();
        //load trajectory into buffer
        state++;
    case 1: 
        if(timer.Get()>1.0){indexer.SetIndexState(Indexer::IndexState::Fire);}
        if(timer.Get()>2.5){state++;}
        if(!_autoWaitForShooter){state++;}
        //state++;
        break;
    case 2://start traj
        _shooterThreadMutex.lock();
        _autotrack = true;
        _shooterThreadMutex.unlock();
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchRun_f1Points, TrenchRun_f1Len, true);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchRun_f1Points, TrenchRun_f1Len, false);
        drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
        state++;
        break;
    case 3: //wait for traj to finish
        feeder.Feed(AutoFeederSpeed);
        indexer.SetIndexState(Indexer::IndexState::AutoLoad);
        if(drive.IsMotionProfileFinished()){state++;}
        shooter.setWheelSpeed(16000);
        break;
    case 4: //prepare to reverse traj
        drive.ResetEncoders();
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchRun_r2Points, TrenchRun_r2Len, true);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchRun_r2Points, TrenchRun_r2Len, false);
        drive.StartMotionProfile(right_bufferedStream, left_bufferedStream, ControlMode::MotionProfile); 
        timer.Reset();
        timer.Start();
        state++;
        break;
    case 5: 
        if(drive.IsMotionProfileFinished()){state++;timer.Reset();}
        break;
    case 6: //FIRE
        indexer.SetIndexState(Indexer::IndexState::Fire); 
        if(timer.Get()>.5){state++;}
        break;
    case 7: //start generator Run
        if(_routine != AutoRoutine::TrenchRun8)
        {
            state = 99;
            break;
        }
        setFeeder(false);
        drive.ResetEncoders();
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchRun_f3Points, TrenchRun_f3Len, true);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchRun_f3Points, TrenchRun_f3Len, false);
        drive.StartMotionProfile(right_bufferedStream, left_bufferedStream, ControlMode::MotionProfile); 
        state++;
        break;
    case 8:
        if(drive.IsMotionProfileFinished()){state++;timer.Reset();}
        break;
    case 9: //start generator Run
        drive.ResetEncoders();
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchRun_r4Points, TrenchRun_r4Len, true);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchRun_r4Points, TrenchRun_r4Len, false);
        drive.StartMotionProfile(right_bufferedStream, left_bufferedStream, ControlMode::MotionProfile); 
        state++;
        break;
    case 10://FIRE When done 
        if(drive.IsMotionProfileFinished()){state++;timer.Reset();indexer.DirectDrive(.5,.3,1.0);}
        break;
    case 11:
        break;
    
    default:
        break;
    }
}

void Robot::AutoTrench2()
{
    switch (state)
    {
    case 0:
        timer.Start();
        timer.Reset();//reset timer
        shooter.setWheelSpeed(18000);
        drive.ResetEncoders();
        feeder.Feed(AutoFeederSpeed);
        state++;
        _shooterThreadMutex.lock();
        _autotrack = true;
        _shooterThreadMutex.unlock();
        break;
    case 1:
        tragTool.GetStreamFromArray(left_bufferedStream, SideTrench_f1Points, SideTrench_f1Len, true);
        tragTool.GetStreamFromArray(right_bufferedStream, SideTrench_f1Points, SideTrench_f1Len, false);
        drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile);
    /*
        drive.StartArcMotionProfile(SideTrench_f1Len, 
                                    SideTrench_f1Points[MTPath::column2::position_center_v2],
                                    SideTrench_f1Points[MTPath::column2::velocity_center_v2],
                                    SideTrench_f1Points[MTPath::column2::heading_v2],
                                    SideTrench_f1Points[0][MTPath::column2::duration_v2]
                                    );
    */
        state++;
        break;
    case 2: 
        if(drive.IsMotionProfileFinished())
        {
            state++; 
            timer.Reset();
            indexer.DirectDrive(.5, .4, 1.0);
        }
        break;
    case 3: //shoot
        if(timer.Get()>.5){state++;}
        break;
    case 4: 
        tragTool.GetStreamFromArray(left_bufferedStream, SideTrench_f2Points, SideTrench_f2Len, true);
        tragTool.GetStreamFromArray(right_bufferedStream, SideTrench_f2Points, SideTrench_f2Len, false);
        drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile);
    /*
        drive.StartArcMotionProfile(SideTrench_f2Len, 
                                    SideTrench_f2Points[MTPath::column2::position_center_v2],
                                    SideTrench_f2Points[MTPath::column2::velocity_center_v2],
                                    SideTrench_f2Points[MTPath::column2::heading_v2],
                                    SideTrench_f2Points[0][MTPath::column2::duration_v2]
                                    );
    */
        state++;
        break;
    case 5: 
        if(drive.IsMotionProfileFinished())
        {
            state++; 
        }
        break;
    case 6: 
        if(_routine == AutoRoutine::SideTrench8)
        {
            _routine = AutoRoutine::TrenchRun8;
            state = 4;
        }
        if(_routine == AutoRoutine::SideTrench)
        {
            _routine = AutoRoutine::TrenchRun6;
            state = 4;
        }
        AutoTrench1();
        break;
    case 7: 
        break;
    default:
        break;
    }
    
}

void Robot::AutoTrenchNinja1()
{
    switch (state)
    {
    case 0: //setup
        timer.Start();
        timer.Reset();//reset timer
        //setFeeder();
        shooter.setWheelSpeed(16000);
        feeder.Feed(AutoFeederSpeed);
        indexer.SetIndexState(Indexer::IndexState::AutoLoad);
        //indexer.DirectDrive(.5,.3,1.0);
        //zero the encoders and state machine variable
        drive.ResetEncoders();
        //load trajectory into buffer
        state++;
    case 1:
        if(timer.Get()>.25)
        {
            tragTool.GetStreamFromArray(left_bufferedStream, TrenchNinja_f1Points, TrenchNinja_f1Len, true);
            tragTool.GetStreamFromArray(right_bufferedStream, TrenchNinja_f1Points, TrenchNinja_f1Len, false);
            drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
            state++;
        }
        break;
    case 2: //Move to Trench or Generator 3 ball 
        if(drive.IsMotionProfileFinished())
        {
            _shooterThreadMutex.lock();
            _autotrack = true;
            _shooterThreadMutex.unlock();
            drive.ResetEncoders();
            
            if(_routine == AutoRoutine::TrenchNinja5 || _routine == AutoRoutine::TrenchNinjaTrench )
            {
                state = 10;
            }
            if(_routine == AutoRoutine::TrenchNinjaGenerator8)
            {
                state = 30;
            }
        }
        //state++;
        break;
    case 10://start traj To Target from Enemy Trench
        drive.ResetEncoders();
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchNinja_r2tPoints, TrenchNinja_r2tLen, true);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchNinja_r2tPoints, TrenchNinja_r2tLen, false);
        drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
        state++;
        break;
    
    case 11: //wait for traj to finish
        if(drive.IsMotionProfileFinished())
        {
            indexer.SetIndexState(Indexer::IndexState::Fire);//FIRE
            timer.Reset();
            state++;
        }
        break;
    case 12: //wait for traj to finish
        if(timer.Get() > 1.5)
        {
            indexer.SetIndexState(Indexer::IndexState::AutoLoad);//Load
            if(_routine == AutoRoutine::TrenchNinja5)
            {
                state  = 99;//done
                break;
            }else
            {
                state = 2;
                _routine = AutoRoutine::TrenchRun6;
                break;
            }
        }
        break;
    case 30: //Run Generator
        drive.ResetEncoders();
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchNinja_r2gPoints, TrenchNinja_r2gLen, true);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchNinja_r2gPoints, TrenchNinja_r2gLen, false);
        drive.StartMotionProfile(right_bufferedStream, left_bufferedStream, ControlMode::MotionProfile); 
        timer.Reset();
        timer.Start();
        shooter.setWheelSpeed(17000);
        state++;
        break;
    case 31: 
        if(drive.IsMotionProfileFinished())
        {
            state++;
            timer.Reset();
            indexer.SetIndexState(Indexer::IndexState::Fire);
        }
        break;
    case 32: //FIRE
        if(timer.Get()>1.0)
        {
            state++;
            indexer.SetIndexState(Indexer::IndexState::AutoLoad);
            setFeeder(false);
        }//wait 2 seconds
        break;
    case 33: //start generator Run
        drive.ResetEncoders();
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchNinja_f3gPoints, TrenchNinja_f3gLen, true);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchNinja_f3gPoints, TrenchNinja_f3gLen, false);
        drive.StartMotionProfile(right_bufferedStream, left_bufferedStream, ControlMode::MotionProfile); 
        state++;
        break;
    case 34:
        if(drive.IsMotionProfileFinished()){state++;timer.Reset();}
        break;
    case 35: //back up for 3rd ball
        drive.ResetEncoders();
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchNinja_r4gPoints, TrenchNinja_r4gLen, true);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchNinja_r4gPoints, TrenchNinja_r4gLen, false);
        drive.StartMotionProfile(right_bufferedStream, left_bufferedStream, ControlMode::MotionProfile); 
        state++;
        break;
    case 36:
        if(drive.IsMotionProfileFinished()){state++;timer.Reset();}
        break;
    case 37:
        drive.ResetEncoders();
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchNinja_f5gPoints, TrenchNinja_f5gLen, true);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchNinja_f5gPoints, TrenchNinja_f5gLen, false);
        drive.StartMotionProfile(right_bufferedStream, left_bufferedStream, ControlMode::MotionProfile); 
        state++;
        break;
    case 38:
        if(drive.IsMotionProfileFinished()){state++;timer.Reset();}
        break;
    case 39:

    case 45:
        drive.ResetEncoders();
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchNinja_r6gPoints, TrenchNinja_r6gLen, true);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchNinja_r6gPoints, TrenchNinja_r6gLen, false);
        drive.StartMotionProfile(right_bufferedStream, left_bufferedStream, ControlMode::MotionProfile); 
        state++;
        break;
    case 46:
        if(drive.IsMotionProfileFinished())
        {
            state++;
            timer.Reset();
            indexer.SetIndexState(Indexer::IndexState::Fire);//FIRE
        }
        break;
    
    default:
        break;
    }

}

void Robot::AutoTrenchNinja2()
{
    
}

void Robot::AutoTrenchNinja3()
{
      //Old Code 
    switch (state)
    {
    case 0: //setup
        timer.Start();
        timer.Reset();//reset timer
        shooter.setWheelSpeed(16000);
        shooter.setTargetHeading(0);\
        //indexer.DirectDrive(.5,.3,1.0);
        //zero the encoders and state machine variable
        drive.ResetEncoders();
        //load trajectory into buffer
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, true, true);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, false, true);
        state++;
    case 1: 
        if(timer.Get()>1.0){indexer.SetIndexState(Indexer::IndexState::Fire);}
        if(timer.Get()>2.5){state++;}
        if(!_autoWaitForShooter){state++;}
        //state++;
        break;
    case 2://start traj
        _shooterThreadMutex.lock();
        _autotrack = true;
        _shooterThreadMutex.unlock();
        drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
        state++;
        break;
    }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
