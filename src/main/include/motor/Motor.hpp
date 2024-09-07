#pragma once

class Motor
{
public:
    enum class ControlRequest
    {
        VOLTAGE,
        CURRENT,
        VELOCITY,
        POSITION,
        POSITION_VEL_ACCEL
    };

    Motor() {};
    virtual void setPID(double p, double i, double d) = 0;
    virtual void setControlRequest(double request, ControlRequest request_type) = 0;
};