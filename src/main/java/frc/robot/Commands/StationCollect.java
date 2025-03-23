// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutoScoringSubsystem;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StationCollect extends Command {
 
  EndEffector m_EE = new EndEffector();
  double m_speed;
  /** Creates a new StationCollect. */
  public StationCollect( EndEffector Collector, double speed) {
    m_EE = Collector;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_EE.collect(m_speed);
    //m_AutoScoringSubsystem.StationPickupGroup();
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
    return 
    m_EE.HasCoral() == true ;
  //  m_EE.HasStationCoral() == true;
  }
}
