// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutoScoringSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LimelightSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScoreCommandFORAUTO extends Command {
  private AutoScoringSubsystem m_AutoScoringSubsystem;


  EndEffector m_EE;
  ElevatorSubsystem m_Elevator;
  AprilTagAim m_Aim;
  String m_side;
  /** Creates a new AutoScoreCommand <p> This one is specifically for auto as it only scores in L4, and does not change after incrementing target value */
  public AutoScoreCommandFORAUTO(AutoScoringSubsystem AutoScore, ElevatorSubsystem Elevator,EndEffector EE, String side) {
    m_AutoScoringSubsystem = AutoScore;
    m_side = side;
    m_EE = EE;
    m_Elevator = Elevator;
    addRequirements(m_AutoScoringSubsystem, m_Elevator,m_EE);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_AutoScoringSubsystem.TargetAndAim(
      m_AutoScoringSubsystem.GetTargetCommandGroup(2), m_side).schedule();
  //  m_AutoScoringSubsystem.GetTargetCommandGroup(target).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_AutoScoringSubsystem.TargetAndAim(m_AutoScoringSubsystem.GetTargetCommandGroup(target),m_side);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
