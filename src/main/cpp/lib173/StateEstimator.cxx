#include "lib173/StateEstimator.hxx"

#include <memory>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include "lib173/Drivetrain.hxx"
#include "Constants.hxx"

StateEstimator::StateEstimator()
{
    mPoseEstimator = std::make_unique<frc::DifferentialDrivePoseEstimator>(frc::Rotation2d{}, frc::Pose2d{}, Constants::kStateStdDevs, Constants::kLocalMeasurementStdDevs, Constants::kVisionMeasurementStdDevs, units::second_t{Constants::kLoopDt});
    mDrivetrain = nullptr;
}

void StateEstimator::setDrivetrain(std::shared_ptr<Drivetrain> drivetrain)
{
    mPoseEstimatorMutex.lock();
    mDrivetrain = drivetrain;
    mPoseEstimatorMutex.unlock();
}

void StateEstimator::reset(frc::Pose2d pose)
{
    mPoseEstimatorMutex.lock();
    mPoseEstimator->ResetPosition(pose, mDrivetrain ? frc::Rotation2d{units::radian_t{mDrivetrain->heading()}} : frc::Rotation2d{});
    if (mDrivetrain)
        mDrivetrain->resetEncoders();
    mPoseEstimatorMutex.unlock();
}

void StateEstimator::update(double timestamp)
{
    frc::Rotation2d heading;
    units::meters_per_second_t leftVelocity{0}, rightVelocity{0};
    units::meter_t leftDistance{0}, rightDistance{0};

    mPoseEstimatorMutex.lock();

    if (mDrivetrain)
    {
        heading = frc::Rotation2d{units::radian_t{mDrivetrain->heading()}};
        leftVelocity = units::meters_per_second_t{mDrivetrain->leftVelocity()};
        rightVelocity = units::meters_per_second_t{mDrivetrain->rightVelocity()};
        leftDistance = units::meter_t{mDrivetrain->leftDistance()};
        rightDistance = units::meter_t{mDrivetrain->rightDistance()};
    }

    mPoseEstimator->Update(heading, frc::DifferentialDriveWheelSpeeds{leftVelocity, rightVelocity}, leftDistance, rightDistance);
    mPoseEstimatorMutex.unlock();
}

void StateEstimator::updateVision(frc::Pose2d pose, double timestamp)
{
    mPoseEstimatorMutex.lock();
    mPoseEstimator->AddVisionMeasurement(pose, units::second_t{timestamp});
    mPoseEstimatorMutex.unlock();
}

frc::Pose2d StateEstimator::pose()
{
    mPoseEstimatorMutex.lock();
    frc::Pose2d pose = mPoseEstimator->GetEstimatedPosition();
    mPoseEstimatorMutex.unlock();

    return pose;
}
