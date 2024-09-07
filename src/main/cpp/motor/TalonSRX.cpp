#include "motor/TalonSRX.hpp"
#include <fmt/core.h>
#include <frc/DriverStation.h>
#include <iostream>

TalonSRX::TalonSRX(int canid, std::string canbus)
    : m_motor(canid)
{
}

TalonSRX::~TalonSRX()
{
}

void TalonSRX::setPID(double p, double i, double d)
{
}

void TalonSRX::setControlRequest(double request, ControlRequest request_type)
{
    switch (request_type)
    {
    case ControlRequest::VOLTAGE:
        m_motor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, request / 12.0);
        std::cout << request << std::endl;
        fmt::println("%d", request);
        break;
    default:
        break;
    }
}
