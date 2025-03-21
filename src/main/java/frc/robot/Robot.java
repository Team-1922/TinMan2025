// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LedSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  private final RobotContainer m_robotContainer;
  private final EndEffector m_EE;
  private final ElevatorSubsystem m_Elevator;
  private final LedSubsystem m_LED;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_EE = m_robotContainer.m_EE;
    m_Elevator = m_robotContainer.m_ElevatorSubsystem;
    m_LED = m_robotContainer.m_LED;
  //  m_EE = new EndEffector();
   // m_Elevator = new ElevatorSubsystem();
    //m_LED = new LedSubsystem(m_EE);
  
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    m_LED.disabledAnimation();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    m_EE.stopEE();
    m_Elevator.StopElevator();
    m_LED.stopAnimation(0);
    m_LED.stopAnimation(1);

  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_EE.ConfigEeCoast();
  }

  @Override
  public void autonomousPeriodic() {
    m_LED.LEDControl();
  }

  @Override
  public void autonomousExit() {
    m_EE.stopEE();
    m_Elevator.StopElevator();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_EE.ConfigEeCoast();
  }

  @Override
  public void teleopPeriodic() {
    m_LED.LEDControl();
  }

  @Override
  public void teleopExit() {
    m_EE.ConfigEeBrake();
    m_EE.stopEE(); 
    m_Elevator.StopElevator();
     
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
