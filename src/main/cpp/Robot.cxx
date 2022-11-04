#include "Robot.hxx"

#include <memory>
#include <frc/geometry/Pose2d.h>
#include <iostream>

#include "lib173/StateEstimator.hxx"

void Robot::RobotInit()
{
    std::shared_ptr<StateEstimator> stateEstimator = StateEstimator::getInstance();
    mLooper.add(stateEstimator);

    mLooper.run();
}

void Robot::RobotPeriodic()
{
    frc::Pose2d position = StateEstimator::getInstance()->position();
    std::cout << "x: " << position.X().value() << ", y: " << position.Y().value() << ", theta: " << position.Rotation().Radians().value() << ", rate: " << StateEstimator::getInstance()->rate() << "\n";
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
