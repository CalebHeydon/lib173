#pragma once

#include <mutex>

class Loop
{
private:
    double mLastTimestamp;
    double mRate;
    std::mutex mRateMutex;

    void _update(double timestamp)
    {
        if (mLastTimestamp < 0)
            mLastTimestamp = timestamp;
        double rate = 1.0 / (timestamp - mLastTimestamp);
        mLastTimestamp = timestamp;

        mRateMutex.lock();
        mRate = rate;
        mRateMutex.unlock();

        update(timestamp);
    }

    friend class Looper;

public:
    Loop()
    {
        mLastTimestamp = -1;
        mRate = 0;
    }

    virtual ~Loop()
    {
    }

    virtual void update(double timestamp)
    {
    }

    double rate()
    {
        mRateMutex.lock();
        double rate = mRate;
        mRateMutex.unlock();

        return rate;
    }
};
