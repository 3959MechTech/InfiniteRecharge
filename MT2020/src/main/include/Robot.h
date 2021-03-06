#pragma once

#include <string>
#include <thread>
#include <mutex>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/SerialPort.h>
#include <frc/XboxController.h>
#include <frc/Notifier.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/DriverStation.h>

#include <rev/CANSparkMax.h>

#include "MechTechDifferential.hpp"
#include "MTPath.h"
#include "Indexer.hpp"
#include "Feeder.h"
#include "Periscope.h"
#include "MTTurretShooter.h"

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"



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
LeftMaster (Device ID 10)
LeftSlave (Device ID 12)
RightMaster (Device ID 11)
RightSlave (Device ID 13)

OTHER DEVICES
PCM (Device ID 5)
PDP (Device ID 0)

currently no probe ID
*/


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void DisabledInit() override;

  void sendData();

  void AutoStraight();
  void AutoTrench1();
  void AutoTrench2();
  void AutoOldTrench();
  void AutoTrenchNinja1();
  void AutoTrenchNinja2();
  void AutoTrenchNinja3();

 private:

  void updatePose();
  void autoTrack();

  void executeTasks();
  void devStick();

  void toggleFeeder();
  void setFeeder(bool down = true);
  void indexerS1();


  enum AutoRoutine {
    Straight, SideTrench, SideTrench8, TrenchRun6, TrenchRun8, TrenchNinja5, TrenchNinjaTrench, TrenchNinjaGenerator8, Dev 
  };
  AutoRoutine _routine;
  bool _autoWaitForShooter;

  frc::SendableChooser<std::string> m_fireModeChooser;
  const std::string kAutoFireOnRun = "Fire on the run";
  const std::string kAutoWait = "Wait for spin up";

  frc::SendableChooser<std::string> m_routineChooser;
  const std::string kAutoNameStraight = "Straight";
  const std::string kAutoNameTrenchRun1 = "Trench Run From Side";
  const std::string kAutoNameTrenchRun2 = "Trench Run From Target";
  const std::string kAutoNameTrenchRun3 = "Trench Run + 2 Generator from Side";
  const std::string kAutoNameTrenchRun4 = "Trench Run + 2 Generator from Target";
  const std::string kAutoNameTrenchNinja1 = "Trench Ninja";
  const std::string kAutoNameTrenchNinja2 = "Trench Ninja + trench";
  const std::string kAutoNameTrenchNinja3 = "Trench Ninja + 3 Generator";
  const std::string kAutoNameDev = "Dev - NOT FOR FLIGHT!!!";

  std::string m_autoSelected;
  std::string m_autoFireModeSelected;

  frc::Timer smTimer{};

  frc::XboxController stick{0};
  frc::XboxController stick2{1};
  frc::XboxController stick3{3};
  frc::XboxController stick4{4};

  int _driverPOVLast, _copilotPOVLast;
  bool _driverPOVChanged, _copilotPOVChanged; 

  frc::Compressor c{0};
  frc::Solenoid _feederPiston{0};
  bool _feederDown;
  frc::Timer _feederDownTime{};
  const double MinFeederDownTime = .5;

  MTDifferential drive{11,13,10,12,1};
  bool _lowSpeed;
  frc::Timer timer{};
  int state = 0;

  const double AutoFeederSpeed = .45;

  MTPath tragTool{6.0, 2048.0,10.3896};//6" wheel diameter, encoder resolution 2048, gear ratio= 10.3896
  BufferedTrajectoryPointStream left_bufferedStream{};
	BufferedTrajectoryPointStream right_bufferedStream{};


  std::thread *_telemetryThread;
  frc::Timer _telemetryTimer{};
  bool _stopTelemetryThread = false;

  frc::Notifier *_PoseThread;
  frc::Timer _PoseTimer{};
  std::shared_timed_mutex _poseThreadMutex;

  double _poseThreadPeriod;

  frc::Notifier *_shooterThread;
  frc::Timer _shooterTimer{};
  std::shared_timed_mutex _shooterThreadMutex;
  bool _autotrack ;
  bool _lastAutoTrack;
  double _shooterThreadPeriod;
  double _targetAngle;

  bool _climbing;

  Periscope periscope{50,51};

  Feeder feeder{60};

  Indexer indexer{30,31,32};
  frc::Timer indexerS1Timer{};
  bool _useAutoIndex;

  MTTurretShooter shooter{20,21,22,2};

  double shooterSpeedSelect;

  //frc::SerialPort cam{115200, frc::SerialPort::kMXP};
  //frc::SerialPort cam1{9600, frc::SerialPort::kUSB1};
  //frc::SerialPort cam2{9600, frc::SerialPort::kUSB2};

  std::shared_ptr<NetworkTable> NetTable;


};
