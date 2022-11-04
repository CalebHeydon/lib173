#pragma once

#include <memory>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <mutex>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include "Loop.hxx"
#include "Drivetrain.hxx"

class StateEstimator : public Loop
{
private:
    static std::shared_ptr<StateEstimator> instance;

    std::unique_ptr<frc::DifferentialDriveOdometry> mOdometry;
    std::mutex mOdometryMutex;
    std::shared_ptr<Drivetrain> mDrivetrain;

public:
    static std::shared_ptr<StateEstimator> getInstance()
    {
        if (!instance)
            instance = std::make_shared<StateEstimator>();
        return instance;
    }

    StateEstimator();

    void setDrivetrain(std::shared_ptr<Drivetrain> drivetrain);
    void reset(frc::Pose2d position, frc::Rotation2d heading);
    void update(double timestamp) override;
    frc::Pose2d position();
};
