#pragma once

#include <frc/TimedRobot.h>

#include "lib173/Looper.hxx"

class Robot : public frc::TimedRobot
{
private:
    Looper mLooper;

public:
    enum Mode
    {
        DISABLED,
        AUTONOMOUS,
        TELEOP
    };

    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
};
