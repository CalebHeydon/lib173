#include "lib173/StateEstimator.hxx"

#include <memory>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include "lib173/Drivetrain.hxx"
#include "Constants.hxx"

std::shared_ptr<StateEstimator> StateEstimator::instance;

StateEstimator::StateEstimator()
{
    mPoseEstimator = std::make_unique<frc::DifferentialDrivePoseEstimator>(frc::Rotation2d{}, frc::Pose2d{}, Constants::kStateStdDevs, Constants::kLocalMeasurementStdDevs, Constants::kvisionMeasurementStdDevs, units::second_t{Constants::kLoopDt});
}

void StateEstimator::setDrivetrain(std::shared_ptr<Drivetrain> drivetrain)
{
    mDrivetrain = drivetrain;
}

void StateEstimator::reset(frc::Pose2d position, frc::Rotation2d heading)
{
    mPoseEstimatorMutex.lock();
    mPoseEstimator->ResetPosition(position, heading);
    mPoseEstimatorMutex.unlock();
}

void StateEstimator::update(double timestamp)
{
    frc::Rotation2d heading;
    units::meters_per_second_t leftVelocity{0}, rightVelocity{0};
    units::meter_t leftDistance{0}, rightDistance{0};

    if (mDrivetrain)
    {
        heading = frc::Rotation2d{units::radian_t{mDrivetrain->heading()}};
        leftVelocity = units::meters_per_second_t{mDrivetrain->leftVelocity()};
        rightVelocity = units::meters_per_second_t{mDrivetrain->rightVelocity()};
        leftDistance = units::meter_t{mDrivetrain->leftDistance()};
        rightDistance = units::meter_t{mDrivetrain->rightDistance()};
    }

    mPoseEstimatorMutex.lock();
    mPoseEstimator->Update(heading, frc::DifferentialDriveWheelSpeeds{leftVelocity, rightVelocity}, leftDistance, rightDistance);
    mPoseEstimatorMutex.unlock();
}

frc::Pose2d StateEstimator::position()
{
    mPoseEstimatorMutex.lock();
    frc::Pose2d position = mPoseEstimator->GetEstimatedPosition();
    mPoseEstimatorMutex.unlock();

    return position;
}
