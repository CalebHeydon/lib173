#pragma once

#include <thread>
#include <vector>
#include <memory>
#include <mutex>
#include <frc/Timer.h>

#include "Loop.hxx"

class Looper
{
private:
    std::thread *mThread = nullptr;

public:
    bool mQuit;
    std::vector<std::shared_ptr<Loop>> mLoops;
    std::mutex mLoopsMutex;

    Looper()
    {
        mQuit = false;
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
                                    double lastTime = (double) frc::Timer::GetFPGATimestamp().value();
                                    while (!this->mQuit)
                                    {
                                        double currentTime = (double) frc::Timer::GetFPGATimestamp().value();
                                        double dt = currentTime - lastTime;
                                        lastTime = currentTime;

                                        this->mLoopsMutex.lock();
                                            for (std::shared_ptr<Loop> loop : mLoops)
                                                loop->update(dt);
                                        this->mLoopsMutex.unlock();
                                    } });
    }
};
