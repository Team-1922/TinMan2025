// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToStation extends Command {
  ElevatorSubsystem m_ElevatorSubsystem;
  /** Creates a new GoToStation. <p>
   * raises elevator so the EndEffector is at the height of the station
  */
  public GoToStation(ElevatorSubsystem elevator) {

    m_ElevatorSubsystem = elevator;
    addRequirements(m_ElevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ElevatorSubsystem.GoToStation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 //   m_ElevatorSubsystem.StopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return   Math.abs(m_ElevatorSubsystem.getElevatorPos() - ElevatorConstants.StationPosition) <0.1;
  }
}
