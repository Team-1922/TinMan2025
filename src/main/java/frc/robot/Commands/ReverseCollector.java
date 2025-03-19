// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReverseCollector extends Command {
  EndEffector m_EE;
  boolean m_StartedWithCoral;
  Timer m_Timer = new Timer();
  /** spins collector so it would spit out coral if on ground*/
  public ReverseCollector(EndEffector endEffector) {
    m_EE = endEffector;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_StartedWithCoral = m_EE.HasCoral();
    m_EE.ReverseCollector(0.4);
    m_Timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (m_EE.HasCoral() != m_StartedWithCoral){
    //  m_Timer.start();
   // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_EE.stopCollector();
    m_Timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //m_Timer.hasElapsed(0.25);
  }
}
