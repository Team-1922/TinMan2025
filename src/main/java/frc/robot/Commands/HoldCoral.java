// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoldCoral extends Command {
  private EndEffector m_EE;
  /** Creates a new HoldCoral. */
  public HoldCoral(EndEffector EE ) {
    m_EE = EE;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_EE.stopCollector();
    m_EE.m_leftCollect.setPosition(m_EE.m_leftCollect.getPosition().getValueAsDouble());
    m_EE.m_rightCollect.setPosition(m_EE.m_rightCollect.getPosition().getValueAsDouble());
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
