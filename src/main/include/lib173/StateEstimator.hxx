#pragma once

#include <memory>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <mutex>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include "Loop.hxx"
#include "Drivetrain.hxx"

class StateEstimator : public Loop
{
private:
    std::unique_ptr<frc::DifferentialDrivePoseEstimator> mPoseEstimator;
    std::mutex mPoseEstimatorMutex;
    std::shared_ptr<Drivetrain> mDrivetrain;

public:
    static std::shared_ptr<StateEstimator> getInstance()
    {
        static std::shared_ptr<StateEstimator> instance = std::make_shared<StateEstimator>();
        return instance;
    }

    StateEstimator();

    void setDrivetrain(std::shared_ptr<Drivetrain> drivetrain);
    void reset(frc::Pose2d position, frc::Rotation2d heading);
    void update(double timestamp) override;
    void updateVision(frc::Pose2d position, double timestamp);
    frc::Pose2d position();
};
