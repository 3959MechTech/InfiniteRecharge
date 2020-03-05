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
    frc::SmartDashboard::PutData("Select Routine", &m_routineChooser);

    _autoWaitForShooter = true;
    _routine = AutoRoutine::TrenchRun6;

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

    _feederDown = false;
    _feederPiston.Set(_feederDown);

    indexerS1Timer.Start();

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
    MTPoseData pose = drive.GetPose();
    bool track = false; 
    _shooterThreadMutex.lock();
    track = _autotrack;
    _targetAngle = std::atan2(226.81-pose.y,0-pose.x)-pose.phi;
    _shooterThreadMutex.unlock();

    if(track)
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
    if(stick3.GetXButton())
    {
        indexer.DirectDrive(.5,.3,1.0);
    }

    if(stick3.GetAButton())
    {
        shooter.setWheelSpeed(18000.0);
        shooterSpeedSelect = 18000.0;
        //shooter.zeroTurret();
    }

    if(stick3.GetBButton())
    {
        shooter.setWheelSpeed(15000.0);
        shooterSpeedSelect = 15000.0;
    }

    if(stick3.GetYButton())
    {
        shooter.setWheelSpeed(13500.0);
        shooterSpeedSelect = 13500.0;
    }

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

    drive.ArcadeDrive(ControlMode::PercentOutput, lY, rX);

    if(stick.GetAButtonPressed())
    {
        if(_feederDown)// && _feederDownTime.Get()>MinFeederDownTime)
        {
            periscope.SetPPos(Periscope::Down);
        }
    }
    if(stick.GetBButtonPressed())
    {
        if(_feederDown)//&& _feederDownTime.Get()>MinFeederDownTime)
        {
            periscope.SetPPos(Periscope::Low);
        }
    }
    if(stick.GetXButtonPressed())
    {
        if(_feederDown)// && _feederDownTime.Get()>MinFeederDownTime)
        {
            periscope.SetPPos(Periscope::Level);
        }
    }
    if(stick.GetYButtonPressed())
    {
        if(_feederDown)// && _feederDownTime.Get()>MinFeederDownTime)
        {
            periscope.SetPPos(Periscope::High);
        }
    }
    if(stick.GetPOV()==0)
    {
        if(_feederDown)// && _feederDownTime.Get()>MinFeederDownTime)
        {
            periscope.SetPosition(periscope.GetEncoderPos()+100.0);
        }
    }
    if(stick.GetPOV()==180)
    {
        if(_feederDown )//&& _feederDownTime.Get()>MinFeederDownTime)
        {
            periscope.SetPosition(periscope.GetEncoderPos()-100.0);
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
        }
    }
    if(stick2.GetPOV()==180)
    {
        if(at)
        {
            _shooterThreadMutex.lock();
            _autotrack = true;
            _shooterThreadMutex.unlock();
        }
    }
    if(stick2.GetPOV()==90)
    {
        shooter.setWheelSpeed(0.0);    
    }

    if(!at)
    {
        shooter.spin(stick2.GetX(frc::GenericHID::kRightHand));
    }

    if(stick2.GetAButtonPressed())
    {
        shooter.setWheelSpeed(13500);
    }
    if(stick2.GetBButtonPressed())
    {
        shooter.setWheelSpeed(16000);
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
                feeder.Feed(stick.GetTriggerAxis(frc::GenericHID::kRightHand)*.75);
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
    
    if(stick2.GetTriggerAxis(frc::GenericHID::kLeftHand)>.1)
    {
        indexer.SetM2(stick2.GetTriggerAxis(frc::GenericHID::kLeftHand)*.8);
    }else
    {
        indexer.SetM2(0.0);
    }
    
    if(stick2.GetBumper(frc::GenericHID::kLeftHand))
    {
        indexer.SetM3(1.0);
    }else
    {
        indexer.SetM3(0.0);
    }
    

    

    if(stick2.GetBackButtonPressed())
    {
        indexer.DirectDrive(-.5,-1.0,-1.0);
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
    indexerS1();

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
    
    state = 0;

    setFeeder();
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
        AutoTrench1();
        break;
    case AutoRoutine::TrenchRun8:
        AutoTrench1();
        break;
    case AutoRoutine::TrenchNinja5:
        AutoTrenchNinja1();
        break;
    case AutoRoutine::TrenchNinjaTrench:
        AutoTrenchNinja2();
        break;
    case AutoRoutine::TrenchNinjaGenerator8:
        AutoTrenchNinja3();
        break;
    
    default:
        break;
    }

}

void Robot::TeleopInit() {
  drive.ResetEncoders();
  state  = 0;
}

void Robot::TeleopPeriodic() {
  
    //devStick();
    executeTasks();
    
    //frc::SmartDashboard::PutNumber("rv", rv);

    //A button = serpintine
    //B button = right turn
    //X button = left turn
    //Y button = straight

    
    
}

void Robot::TestPeriodic() {}

void Robot::AutoStraight()
{
    switch (state)
    {
    case 0:
        timer.Start();
        timer.Reset();//reset timer
        shooter.setWheelSpeed(14000);
        drive.ResetEncoders();
        state++;
        break;
    case 1:
        drive.StartArcMotionProfile(Straight_f1Len, 
                                    Straight_f1Points[MTPath::column2::position_center_v2],
                                    Straight_f1Points[MTPath::column2::velocity_center_v2],
                                    Straight_f1Points[MTPath::column2::heading_v2],
                                    Straight_f1Points[0][MTPath::column2::duration_v2]
                                    );
        state++;
        break;
    case 2: 
        if(drive.IsMotionProfileFinished()){state++;}
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
        shooter.setWheelSpeed(14000);
        //indexer.DirectDrive(.5,.3,1.0);
        //zero the encoders and state machine variable
        drive.ResetEncoders();
        //load trajectory into buffer
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, true, true);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, false, true);
        state++;
    case 1: 
        if(shooter.isShooterReady()){indexer.DirectDrive(.5,.3,1.0);}
        if(timer.Get()>4.5){state++;}
        if(!_autoWaitForShooter){state++;}
        //state++;
        break;
    case 2://start traj
        drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
        state++;
        break;
    case 3: //wait for traj to finish
        feeder.Feed(.6);
        if(drive.IsMotionProfileFinished()){state++;}
        indexer.DirectDrive(0,0,0);
        shooter.setWheelSpeed(17000);
        break;
    case 4: //prepare to reverse traj
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, true, false);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, false, false);
        timer.Reset();
        timer.Start();
        state++;
        break;
    case 5: 
        if(timer.Get()>.20){state++;}//wait 2 seconds
        {indexer.DirectDrive(.5,.3,1.0);} 
        break;
    case 6: //start rev traj
        drive.StartMotionProfile(right_bufferedStream, left_bufferedStream, ControlMode::MotionProfile); 
        timer.Reset();
        state++;
        break;
    case 7: //start rev traj
        
        state++;
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
        shooter.setWheelSpeed(16000);
        drive.ResetEncoders();
        state++;
        break;
    case 1:
        drive.StartArcMotionProfile(SideTrench_f1Len, 
                                    SideTrench_f1Points[MTPath::column2::position_center_v2],
                                    SideTrench_f1Points[MTPath::column2::velocity_center_v2],
                                    SideTrench_f1Points[MTPath::column2::heading_v2],
                                    SideTrench_f1Points[0][MTPath::column2::duration_v2]
                                    );
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
        if(timer.Get()>2.5){state++;}
        break;
    case 4: 
        drive.StartArcMotionProfile(SideTrench_f2Len, 
                                    SideTrench_f2Points[MTPath::column2::position_center_v2],
                                    SideTrench_f2Points[MTPath::column2::velocity_center_v2],
                                    SideTrench_f2Points[MTPath::column2::heading_v2],
                                    SideTrench_f2Points[0][MTPath::column2::duration_v2]
                                    );
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
    AutoStraight();
}

void Robot::AutoTrenchNinja2()
{
    //currently this is the old trench run from target code
    switch (state)
    {
    case 0: //setup
        timer.Start();
        timer.Reset();//reset timer
        shooter.setWheelSpeed(14000);
        //indexer.DirectDrive(.5,.3,1.0);
        //zero the encoders and state machine variable
        drive.ResetEncoders();
        //load trajectory into buffer
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, true, true);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, false, true);
        state++;
    case 1: 
        if(shooter.isShooterReady()){indexer.DirectDrive(.5,.3,1.0);}
        if(timer.Get()>4.5){state++;}
        if(!_autoWaitForShooter){state++;}
        //state++;
        break;
    case 2://start traj
        drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
        state++;
        break;
    case 3: //wait for traj to finish
        feeder.Feed(.6);
        if(drive.IsMotionProfileFinished()){state++;}
        indexer.DirectDrive(0,0,0);
        shooter.setWheelSpeed(17000);
        break;
    case 4: //prepare to reverse traj
        tragTool.GetStreamFromArray(left_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, true, false);
        tragTool.GetStreamFromArray(right_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, false, false);
        timer.Reset();
        timer.Start();
        state++;
        break;
    case 5: 
        if(timer.Get()>.20){state++;}//wait 2 seconds
        {indexer.DirectDrive(.5,.3,1.0);} 
        break;
    case 6: //start rev traj
        drive.StartMotionProfile(right_bufferedStream, left_bufferedStream, ControlMode::MotionProfile); 
        timer.Reset();
        state++;
        break;
    case 7: //start rev traj
        
        state++;
        break;
    default:
        break;
    }
}

void Robot::AutoTrenchNinja3()
{
    AutoStraight();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
