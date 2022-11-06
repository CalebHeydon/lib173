#pragma once

class Drivetrain
{
public:
    virtual double heading();
    virtual double leftVelocity();
    virtual double rightVelocity();
    virtual double leftDistance();
    virtual double rightDistance();
};
