#include "Robot.hxx"

void Robot::RobotInit()
{
}

void Robot::RobotPeriodic()
{
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
