#include "motor/Falcon500.hpp"

Falcon500::Falcon500(int canid, std::string canbus)
    : m_motor(canid, canbus)
{
}

Falcon500::~Falcon500()
{
}

void Falcon500::setPID(double p, double i, double d)
{
}

void Falcon500::setControlRequest(double request, ControlRequest request_type)
{
    switch (request_type)
    {
    case ControlRequest::VOLTAGE:
        m_motor.SetVoltage(units::voltage::volt_t{request});
        break;
    default:
        fmt::println("Recieved non implememented control request");
    }
}
