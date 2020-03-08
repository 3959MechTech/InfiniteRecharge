
#include "Indexer.hpp"


//Constructor for the Indexer Class
Indexer::Indexer(int frontIndex, int midIndex, int backIndex) : _fI(frontIndex, rev::CANSparkMax::MotorType::kBrushless), 
    _mI(midIndex, rev::CANSparkMax::MotorType::kBrushless), 
    _bI(backIndex, rev::CANSparkMax::MotorType::kBrushless)
{

    _fI.RestoreFactoryDefaults();
    _mI.RestoreFactoryDefaults();
    _bI.RestoreFactoryDefaults();
    
    _fI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 50);
    _fI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 50);
    _fI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);
    _mI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 50);
    _mI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 50);
    _mI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);
    _bI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 50);
    _bI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 50);
    _bI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);
}

//Deconstructor for the Indexer class
Indexer::~Indexer()
{

}

void Indexer::Advance(double speed)
{
    _fI.Set(speed);
    _mI.Set(speed);
    _bI.Set(speed);
}

void Indexer::Retract(double speed)
{
    _fI.Set(-speed);
    _mI.Set(-speed);
    _bI.Set(-speed);
}

void Indexer::DirectDrive(double m1, double m2, double m3)
{
    _fI.Set(m1);
    _mI.Set(m2);
    _bI.Set(m3);
}

void Indexer::SetM1(double speed)
{
    _fI.Set(speed);
}

void Indexer::SetM2(double speed)
{
    _mI.Set(speed);
}

void Indexer::SetM3(double speed)
{
    _bI.Set(speed);
}

void Indexer::SendData(std::string name)
{
    frc::SmartDashboard::PutBoolean("Stage 3 Sensor", _stage3.Get());
    frc::SmartDashboard::PutBoolean("Stage 2 Sensor", _stage2.Get());
    switch (_state)
    {
    case FullStop:
        frc::SmartDashboard::PutString(name + " State","STOP");
        break;
    case Eject:
        frc::SmartDashboard::PutString(name + " State","Eject");
        break;
    case Fire:
        frc::SmartDashboard::PutString(name + " State","FIRE");
        break;
    case AutoLoad:
        frc::SmartDashboard::PutString(name + " State","AutoLoad");
        break;
    case Stage1Pulse:
        frc::SmartDashboard::PutString(name + " State","Stage1Only");
        break;
    default:
        frc::SmartDashboard::PutString(name + " State","Stage1Only");
        break;
    }

    switch (_loadState)
    {
    case Empty:
        frc::SmartDashboard::PutString(name + " Load State","Empty");
        break;
    case Stage3Loaded:
        frc::SmartDashboard::PutString(name + " Load State","Stage3Loaded");
        break;
    case FullyLoaded:
        frc::SmartDashboard::PutString(name + " Load State","FullyLoaded");
        break;
    default:
        break;
    }

}

void Indexer::AutoIndex()
{
    if(!_stage3.Get()&&!_stage2.Get())
    {
        _loadState = FullyLoaded;
    }else
    {
        if(!_stage3.Get())
        {
            _loadState = Stage3Loaded;
        }else
        {
            _loadState = Empty;
        }
        
    }
    

    switch (_state)
    {
    case FullStop:
        DirectDrive(0,0,0);
        break;
    case Eject:
        DirectDrive(-4.0,-6.0,-6.0);
        break;
    case Fire:
        DirectDrive(.5,.4,1.0);
        break;
    case AutoLoad:
        switch (_loadState)
        {
        case Empty:
            DirectDrive(0.4,0.25,.15);
            break;
        case Stage3Loaded:
            DirectDrive(0.5,0.1,0.0);
            break;
        case FullyLoaded:
            DirectDrive(0.05,0,0);
            break;
        default:
            break;
        }
        break;
    case Stage1Pulse:
        if(_timer.Get()< _pulseOnPeriod+_pulseOffPeriod)
        {
            _timer.Reset();
            SetM1(_stage1Power);
        }else
        {
            SetM1(0);
        }    
        break;
    default:
        break;
    }
}

void Indexer::SetIndexState(IndexState state)
{
    _state = state;
}

Indexer::IndexLoadState Indexer::GetLoadState()
{
    return _loadState;
}

Indexer::IndexState Indexer::GetIndexState()
{
    return _state;
}