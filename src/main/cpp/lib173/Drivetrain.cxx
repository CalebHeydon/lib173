#include "lib173/Drivetrain.hxx"

#include <memory>
#include <frc/controller/RamseteController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include "Constants.hxx"
#include "lib173/StateEstimator.hxx"

Drivetrain::Drivetrain()
{
    mRamseteController = std::make_shared<frc::RamseteController>(Constants::kRamseteB, Constants::kRamseteZeta);
    mRamseteController->SetTolerance(frc::Pose2d{units::meter_t{Constants::kRamseteToleranceX}, units::meter_t{Constants::kRamseteTolaranceY}, frc::Rotation2d{}});

    mPathFollowingTimestamp = -1.0;
}

double Drivetrain::heading()
{
    return 0;
}

double Drivetrain::leftVelocity()
{
    return 0;
}

double Drivetrain::rightVelocity()
{
    return 0;
}

double Drivetrain::leftDistance()
{
    return 0;
}

double Drivetrain::rightDistance()
{
    return 0;
}

void Drivetrain::driveVelocity(double left, double right)
{
}

void Drivetrain::resetEncoders()
{
}

void Drivetrain::update(double timestamp)
{
    mTrajectoryMutex.lock();

    std::shared_ptr<StateEstimator> stateEstimator = StateEstimator::instance();

    if (mPathFollowingTimestamp < 0 && mTrajectory)
    {
        mPathFollowingTimestamp = timestamp;
        driveVelocity(0, 0);
    }
    else if (mPathFollowingTimestamp >= 0 && !mTrajectory)
    {
        mPathFollowingTimestamp = -1.0;
        driveVelocity(0, 0);
    }
    else if (mTrajectory)
    {
        if (mRamseteController->AtReference())
        {
            mTrajectory = nullptr;
            mPathFollowingTimestamp = -1.0;
            driveVelocity(0, 0);

            mTrajectoryMutex.unlock();
            return;
        }

        double time = timestamp - mPathFollowingTimestamp;
        frc::Trajectory::State goal = mTrajectory->Sample(units::second_t{time});

        frc::ChassisSpeeds wheelSpeeds = mRamseteController->Calculate(stateEstimator->pose(), goal);
        driveVelocity(wheelSpeeds.vx.value(), wheelSpeeds.vy.value());
    }

    mTrajectoryMutex.unlock();
}
