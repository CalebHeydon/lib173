#include "Robot.hxx"

#include <memory>
#include <frc/geometry/Pose2d.h>
#include <iostream>

#include "lib173/StateEstimator.hxx"

void Robot::RobotInit()
{
    std::shared_ptr<StateEstimator> stateEstimator = StateEstimator::instance();
    mLooper.add(stateEstimator);
    AddPeriodic([this]
                { mLooper.update(); },
                units::second_t{Constants::kLoopDt});
}

void Robot::RobotPeriodic()
{
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
