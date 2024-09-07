#pragma once

#include <string>
#include <ctre/phoenix6/TalonFX.hpp>
#include "motor/Motor.hpp"

class Falcon500 : public Motor
{
public:
    Falcon500(int canid, std::string canbus);
    ~Falcon500();

    void setPID(double p, double i, double d) override;
    void setControlRequest(double request, ControlRequest request_type) override;

protected:
    ctre::phoenix6::hardware::TalonFX m_motor;
};