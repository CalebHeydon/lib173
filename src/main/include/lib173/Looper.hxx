#pragma once

#include <thread>
#include <vector>
#include <memory>
#include <mutex>
#include <frc/Timer.h>
#include <thread>

#include "Loop.hxx"
#include "../Constants.hxx"

class Looper
{
private:
    std::thread *mThread = nullptr;

public:
    bool mQuit;
    double mLastTimestamp;
    std::vector<std::shared_ptr<Loop>> mLoops;
    std::mutex mLoopsMutex;
    double mRate;
    std::mutex mRateMutex;

    Looper()
    {
        mQuit = false;
        mLastTimestamp = -1;
        mRate = -1;
    }

    ~Looper()
    {
        if (mThread)
        {
            mQuit = true;
            mThread->join();
            delete mThread;
        }
    }

    void add(std::shared_ptr<Loop> loop)
    {
        mLoopsMutex.lock();
        mLoops.push_back(loop);
        mLoopsMutex.unlock();
    }

    void run()
    {
        mThread = new std::thread([this]
                                  {
                                    while (!this->mQuit)
                                    {
                                        double currentTime = frc::Timer::GetFPGATimestamp().value();
                                        if (mLastTimestamp == -1)
                                            mLastTimestamp = currentTime;
                                        double dt = currentTime - mLastTimestamp;
                                        mLastTimestamp = currentTime;

                                        mRateMutex.lock();
                                        mRate = 1 / dt;
                                        mRateMutex.unlock();

                                        this->mLoopsMutex.lock();
                                            for (std::shared_ptr<Loop> loop : mLoops)
                                                loop->update(currentTime);
                                        this->mLoopsMutex.unlock();

                                        double waitTime = Constants::kLoopDt - dt;
                                        if (waitTime < 0)
                                            continue;
                                        std::this_thread::sleep_for(std::chrono::minutes{10l});
                                    } });
    }

    double rate()
    {
        mRateMutex.lock();
        double rate = mRate;
        mRateMutex.unlock();
        return rate;
    }
};
