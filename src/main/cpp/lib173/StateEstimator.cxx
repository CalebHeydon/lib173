#include "lib173/StateEstimator.hxx"

#include <memory>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include "lib173/Drivetrain.hxx"

std::shared_ptr<StateEstimator> StateEstimator::instance;

StateEstimator::StateEstimator()
{
    mOdometry = std::make_unique<frc::DifferentialDriveOdometry>(frc::Rotation2d{units::radian_t{0}}, frc::Pose2d{frc::Translation2d{units::meter_t{0}, units::meter_t{0}}, frc::Rotation2d{units::radian_t{0}}});
}

void StateEstimator::setDrivetrain(std::shared_ptr<Drivetrain> drivetrain)
{
    mDrivetrain = drivetrain;
}

void StateEstimator::reset(frc::Pose2d position, frc::Rotation2d heading)
{
    mOdometryMutex.lock();
    mOdometry->ResetPosition(position, heading);
    mOdometryMutex.unlock();
}

void StateEstimator::update(double timestamp)
{
    units::meter_t leftDistance{0}, rightDistance{0};
    frc::Rotation2d heading{units::radian_t{0}};

    if (mDrivetrain)
    {
        leftDistance = units::meter_t{mDrivetrain->leftDistance()};
        rightDistance = units::meter_t{mDrivetrain->rightDistance()};
        heading = frc::Rotation2d{units::radian_t{mDrivetrain->heading()}};
    }

    mOdometryMutex.lock();
    mOdometry->Update(heading, leftDistance, rightDistance);
    mOdometryMutex.unlock();
}

frc::Pose2d StateEstimator::position()
{
    mOdometryMutex.lock();
    frc::Pose2d position = mOdometry->GetPose();
    mOdometryMutex.unlock();

    return position;
}
