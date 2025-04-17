// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import static edu.wpi.first.units.Units.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilTagAimFORAUTO extends Command {
  LimelightSubsystem m_LimelightSubsystem;

  Timer TimeSinceLastSeenTag ;//= new Timer();
 PPHolonomicDriveController AutoDriveTrain;
 
  /** the apriltag aim for auto, WILL NOT WORK IN TELEOP
   <p>might need parameter for what tag to aim at/what location
   */
  public AprilTagAimFORAUTO(
LimelightSubsystem LimeLightSub) {
  m_LimelightSubsystem = LimeLightSub;

  TimeSinceLastSeenTag = new Timer();
  addRequirements(m_LimelightSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LimelightSubsystem.AimTargetYawDutyCycle();
    TimeSinceLastSeenTag.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(!m_LimelightSubsystem.HasTarget()){
      TimeSinceLastSeenTag.reset();
    }
    PPHolonomicDriveController.overrideXYFeedback(null, null); // put field relitive XY speeds (meters/sec)
    PPHolonomicDriveController.overrideRotationFeedback(null);// put field relitive rotation 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  PPHolonomicDriveController.clearFeedbackOverrides();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 
        (Math.abs(m_LimelightSubsystem.targetXError()) < 0.0041 //0.05 // maybe edit these to be more accurate to the localization numbers 
        && Math.abs(m_LimelightSubsystem.targetZError()) < 0.015 //0.108
        && Math.abs(m_LimelightSubsystem.targetYawError()) < 0.025//0.05
        ) || TimeSinceLastSeenTag.hasElapsed(1);
  }
}
