#include "Robot.hxx"

#include <memory>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <iostream>

#include "lib173/StateEstimator.hxx"
#include "RageVision.hxx"
#include "Constants.hxx"

void Robot::RobotInit()
{
    std::shared_ptr<StateEstimator> stateEstimator = StateEstimator::instance();
    mLooper.add(stateEstimator);
    AddPeriodic([this]
                { mLooper.update(); },
                units::second_t{Constants::kLoopDt});

    mVision = std::make_shared<RageVision>();
    mVision->run(Constants::kVisionDataPort, [](double timestamp, int id, double tx, double ty, double tz, double qw, double qx, double qy, double qz, double processingLatency) {});
    mVisionInitialized = false;
}

void Robot::RobotPeriodic()
{
    if (!mVisionInitialized)
        mVisionInitialized = mVision->sync(Constants::kVisionIp, frc::Timer::GetFPGATimestamp().value()) == -1 ? false : true;

    frc::Pose2d position = StateEstimator::instance()->position();
    std::cout << "x: " << position.X().value() << ", y: " << position.Y().value() << ", theta: " << position.Rotation().Radians().value() << ", rate: " << mLooper.rate() << "\n";
}

void Robot::AutonomousInit()
{
}

void Robot::AutonomousPeriodic()
{
}

void Robot::TeleopInit()
{
}

void Robot::TeleopPeriodic()
{
}

void Robot::DisabledInit()
{
}

void Robot::DisabledPeriodic()
{
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
