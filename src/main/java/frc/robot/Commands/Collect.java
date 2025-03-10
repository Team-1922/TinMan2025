// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Collect extends Command {
  EndEffector m_EE = new EndEffector();
  boolean m_StartedWithCoral;
  /** Creates a new Collect. */
  public Collect( EndEffector collector) {
    m_EE = collector;
    
    m_StartedWithCoral = m_EE.HasCoral();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  m_EE.collect();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_EE.stopCollector();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_EE.HasCoral() != m_StartedWithCoral; 
    // because this command will be used for collecting and scoring, this should end the command when the opposite happens
  }
}
