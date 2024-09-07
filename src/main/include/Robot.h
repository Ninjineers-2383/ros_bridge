// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <memory>
#include <vector>

#include "zmq/zmq.hpp"
#include "motor/Motor.hpp"

class Robot : public frc::TimedRobot
{
public:
  ~Robot() override;
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  zmq::context_t context;
  std::unique_ptr<zmq::socket_t> socketPub{nullptr};
  std::unique_ptr<zmq::socket_t> socketRep{nullptr};

  zmq::pollitem_t *poll_items = nullptr;

  std::vector<std::unique_ptr<Motor>> motors;
};
