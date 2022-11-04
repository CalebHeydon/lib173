#pragma once

#include "Robot.hxx"

class System
{
public:
    virtual void update(double timestamp, Robot::Mode mode);
};
