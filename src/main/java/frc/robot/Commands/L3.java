// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L3 extends Command {
  ElevatorSubsystem m_Elevator;
  EndEffector m_EE;
  /** Creates a new GotoL4. */
  public L3( ElevatorSubsystem elevatorSubsystem,EndEffector EE) {
    m_Elevator = elevatorSubsystem;
    m_EE = EE;
    addRequirements(m_Elevator, m_EE);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // m_Elevator.GoToL3();
    m_EE.L2();}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // m_Elevator.StopElevator();
    m_EE.stopEE();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 
    Math.abs(m_EE.getCurrentWristAngle() - EndEffectorConstants.L2WristAngle) <0.05 &&
    Math.abs(m_EE.getCurrentArmAngle() - EndEffectorConstants.L2ArmAngle) <0.05 ;
  }
}
