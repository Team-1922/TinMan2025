// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.AutoScoringSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScoreCommand extends Command {
  private AutoScoringSubsystem m_AutoScoringSubsystem;
  int target;
  LimelightSubsystem m_Limelight;
  /** Creates a new AutoScoreCommand. */
  public AutoScoreCommand(AutoScoringSubsystem AutoScore, LimelightSubsystem LL) {
    m_AutoScoringSubsystem = AutoScore;
    m_Limelight = LL;
    addRequirements(m_AutoScoringSubsystem, m_Limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = m_AutoScoringSubsystem.GetTargetLevel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_AutoScoringSubsystem.TargetAndAim(m_AutoScoringSubsystem.GetTargetCommandGroup(target), m_Limelight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
