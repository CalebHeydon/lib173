#pragma once

#include <memory>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/Trajectory.h>
#include <mutex>

#include "Loop.hxx"

class Drivetrain : public Loop
{
private:
    std::shared_ptr<frc::RamseteController> mRamseteController;
    double mPathFollowingTimestamp;

protected:
    std::shared_ptr<frc::Trajectory> mTrajectory;
    std::mutex mTrajectoryMutex;

public:
    Drivetrain();

    virtual double heading();
    virtual double leftVelocity();
    virtual double rightVelocity();
    virtual double leftDistance();
    virtual double rightDistance();
    virtual void driveVelocity(double left, double right);
    virtual void resetEncoders();

    void update(double timestamp) override;
};
