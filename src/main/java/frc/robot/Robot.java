// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  private final RobotContainer m_robotContainer;
  private final EndEffector m_EE;
  private final ElevatorSubsystem m_Elevator;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_EE = new EndEffector();
    m_Elevator = new ElevatorSubsystem();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    // m_EE.disabledAnimation();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    // m_EE.stopAnimation(0);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
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