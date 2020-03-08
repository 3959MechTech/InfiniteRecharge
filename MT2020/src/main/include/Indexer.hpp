#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>


#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"

class Indexer
{
    public:
        //Constructor for the Indexer class
        Indexer(int frontIndex, int midIndex, int backIndex);
        //Deconstructor for the Indexer class
        ~Indexer();

        enum IndexState{
            FullStop,
            Eject,
            Fire,
            AutoLoad,
            Stage1Pulse
        };
        enum IndexLoadState{
            Empty,
            Stage3Loaded,
            FullyLoaded
        };

        void AutoIndex();
        
        void SetIndexState(IndexState state);

        IndexLoadState GetLoadState();
        IndexState GetIndexState();

        
        void DirectDrive(double m1, double m2, double m3);
        void SetM1(double speed);
        void SetM2(double speed);
        void SetM3(double speed);

        void Advance(double speed);
        void Retract(double speed);

        void SendData(std::string name = "Indexer");

    private:
    //Declares the Motor controller for the indexer
        rev::CANSparkMax _fI;
        rev::CANSparkMax _mI;
        rev::CANSparkMax _bI;

        frc::DigitalInput _stage3{1};
        frc::DigitalInput _stage2{0};

        frc::Timer _timer{};

        double _stage1Power;
        const double _pulseOffPeriod = .25;
        const double _pulseOnPeriod  = 1.0;

        IndexState _state;
        IndexLoadState _loadState;
};