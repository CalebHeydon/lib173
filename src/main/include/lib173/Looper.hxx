#pragma once

#include <memory>
#include <frc/Notifier.h>
#include <vector>
#include <mutex>
#include <frc/Timer.h>

#include "Loop.hxx"
#include "../Constants.hxx"

class Looper
{
private:
    std::unique_ptr<frc::Notifier> mNotifier;

public:
    double mLastTimestamp;
    std::vector<std::shared_ptr<Loop>> mLoops;
    std::mutex mLoopsMutex;
    double mRate;
    std::mutex mRateMutex;

    Looper()
    {
        mLastTimestamp = -1;
        mRate = -1;
    }

    void add(std::shared_ptr<Loop> loop)
    {
        mLoopsMutex.lock();
        mLoops.push_back(loop);
        mLoopsMutex.unlock();
    }

    void run()
    {
        mNotifier = std::make_unique<frc::Notifier>(0, [this]
                                                    {
            double currentTimestamp = frc::Timer::GetFPGATimestamp().value();
            if (mLastTimestamp == -1)
                mLastTimestamp = currentTimestamp;
            double dt = currentTimestamp - mLastTimestamp;
            mLastTimestamp = currentTimestamp;

            mRateMutex.lock();
            mRate = 1 / dt;
            mRateMutex.unlock();

            this->mLoopsMutex.lock();
            for (std::shared_ptr<Loop> loop : mLoops)
                loop->update(currentTimestamp);
            this->mLoopsMutex.unlock(); });
        mNotifier->StartPeriodic(units::second_t{Constants::kLoopDt});
    }

    void stop()
    {
        mNotifier->Stop();
    }

    double rate()
    {
        mRateMutex.lock();
        double rate = mRate;
        mRateMutex.unlock();
        return rate;
    }
};
