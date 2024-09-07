// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <zmq/zmq.hpp>
#include <iostream>
#include <frc/DriverStation.h>
#include "motor/Falcon500.hpp"
#include "motor/TalonSRX.hpp"

#include "fms_status.pb.h"
#include "motor_init.pb.h"
#include "motor_control.pb.h"

Robot::~Robot()
{
  delete[] poll_items;
}

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  frc::SmartDashboard::PutNumber("value", 0);

  socketPub = std::make_unique<zmq::socket_t>(context, ZMQ_PUB);
  socketRep = std::make_unique<zmq::socket_t>(context, ZMQ_REP);

  poll_items = new zmq::pollitem_t[1]{
      {*socketRep, 0, ZMQ_POLLIN, 0}};

  socketPub->bind("tcp://0.0.0.0:8090");
  socketRep->bind("tcp://0.0.0.0:8091");
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  {
    // std::string topic_str = "MOTOR1";
    // zmq::message_t topic(topic_str.size());
    // memcpy(topic.data(), topic_str.c_str(), topic_str.size());

    zmq::message_t topic(std::string("MOTOR1"));
    zmq::message_t data(8);
    *(double *)data.data() = frc::SmartDashboard::GetNumber("value", 0);
    socketPub->send(topic, ZMQ_SNDMORE);
    socketPub->send(data);
  }

  {
    zmq::message_t topic(std::string("FMS"));
    socketPub->send(topic, ZMQ_SNDMORE);

    messages::FMS_Status fms_status;
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value())
    {
      switch (alliance.value())
      {
      case frc::DriverStation::Alliance::kRed:
        fms_status.set_alliance(messages::Alliance::ALLIANCE_RED);
        break;
      case frc::DriverStation::Alliance::kBlue:
        fms_status.set_alliance(messages::Alliance::ALLIANCE_BLUE);
        break;
      }
    }
    else
    {
      fms_status.set_alliance(messages::Alliance::ALLIANCE_NONE);
    }
    auto match_type = frc::DriverStation::GetMatchType();
    fms_status.set_match_type((messages::Match_Type)match_type);
    auto driver_station = frc::DriverStation::GetLocation();
    if (driver_station.has_value())
    {
      fms_status.set_driver_station((messages::Driver_Station)driver_station.value());
    }
    else
    {
      fms_status.set_driver_station(messages::Driver_Station::DS_NONE);
    }
    fms_status.set_battery_volts(frc::DriverStation::GetBatteryVoltage());
    fms_status.set_event_name(frc::DriverStation::GetEventName());
    fms_status.set_game_message(frc::DriverStation::GetGameSpecificMessage());
    fms_status.set_match_numer(frc::DriverStation::GetMatchNumber());
    fms_status.set_match_seconds(frc::DriverStation::GetMatchTime().value());
    fms_status.set_replay_number(frc::DriverStation::GetReplayNumber());
    fms_status.set_is_ds_attached(frc::DriverStation::IsDSAttached());
    fms_status.set_is_fms_attached(frc::DriverStation::IsFMSAttached());
    fms_status.set_is_enabled(frc::DriverStation::IsEnabled());
    fms_status.set_is_disabled(frc::DriverStation::IsDisabled());
    fms_status.set_is_auto(frc::DriverStation::IsAutonomous());
    fms_status.set_is_teleop(frc::DriverStation::IsTeleop());
    fms_status.set_is_test(frc::DriverStation::IsTest());
    fms_status.set_is_e_stopped(frc::DriverStation::IsEStopped());

    std::string msg_str;
    fms_status.SerializeToString(&msg_str);

    zmq::message_t data(msg_str.size());
    memcpy((void *)data.data(), msg_str.c_str(), msg_str.size());
    socketPub->send(data);
  }

  {
    zmq_poll(&poll_items[0], 1, 0);
    if (poll_items[0].revents & ZMQ_POLLIN)
    {
      fmt::println("Recieved request");
      zmq::message_t header;
      socketRep->recv(header);
      zmq::message_t message;
      socketRep->recv(message);

      std::string header_string = header.to_string();

      if (header_string == "INIT_MOTOR")
      {
        messages::motor::Motor_Init_Req motor_init;
        motor_init.ParseFromArray(message.data(), message.size());

        messages::motor::Motor_Init_Rep motor_resp;

        switch (motor_init.motor_type())
        {
        case messages::motor::MotorType::Motor_TalonSRX:
          motors.emplace_back(std::make_unique<TalonSRX>(motor_init.can_id(), motor_init.can_bus()));
          motor_resp.set_success(true);
          motor_resp.set_motorindex(motors.size() - 1);
          motor_resp.set_motor_topic(fmt::format("MOTOR%d", motors.size() - 1));
          break;
        default:
          fmt::println("Unimplemented motor type");
          motor_resp.set_success(false);
          break;
        }

        std::string motor_res;
        motor_resp.SerializeToString(&motor_res);
        zmq::message_t buff(motor_res);
        socketRep->send(buff);
      }
      else if (header_string == "MOTOR_CONTROL")
      {
        fmt::println("motor control");

        messages::motor::Motor_Control_Req motor_control;
        motor_control.ParseFromArray(message.data(), message.size());

        messages::motor::Motor_Control_Rep motor_rep;

        switch (motor_control.control())
        {
        case messages::motor::MotorControlType::CONTROL_VOLTAGE:
          motors[motor_control.motor_id()]->setControlRequest(motor_control.voltage(), Motor::ControlRequest::VOLTAGE);
          break;
        default:
          break;
        }

        std::string response;
        motor_rep.set_success(true);
        motor_rep.SerializeToString(&response);
        zmq::message_t buff(response);
        socketRep->send(buff);
      }
      else if (header_string == "INIT_ENCODER")
      {
        fmt::print("Encoders not setup");
      }
    }
  }

  // std::cout << "sending data" << std::endl;
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic()
{
  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
