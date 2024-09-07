#pragma once

#include <string>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include "motor/Motor.hpp"

class TalonSRX : public Motor
{
public:
    TalonSRX(int canid, std::string canbus);
    ~TalonSRX();

    void setPID(double p, double i, double d) override;
    void setControlRequest(double request, ControlRequest request_type) override;

protected:
    ctre::phoenix::motorcontrol::can::TalonSRX m_motor;
};