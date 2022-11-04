#pragma once

class Drivetrain
{
public:
    virtual double leftDistance();
    virtual double rightDistance();
    virtual double heading();
};
