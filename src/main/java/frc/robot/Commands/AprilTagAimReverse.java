// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import static edu.wpi.first.units.Units.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilTagAimReverse extends Command {
  LimelightSubsystem m_LimelightSubsystem;
  CommandSwerveDrivetrain m_Drivetrain;
 
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  /** Creates a new LeftAim. */
  public AprilTagAimReverse(
LimelightSubsystem LimeLightSub, CommandSwerveDrivetrain drivetrain) {
  m_LimelightSubsystem = LimeLightSub;
  m_Drivetrain = drivetrain;
  addRequirements(m_LimelightSubsystem,m_Drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LimelightSubsystem.AimTargetYawDutyCycle();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    m_Drivetrain.applyRequest(() ->
    new SwerveRequest.RobotCentric().withVelocityX(m_LimelightSubsystem.L2RobotXDutyCycle() * LimelightConstants.MaxAimSpeed) // Drive forward with negative Y (forward)
   
    .withVelocityY(m_LimelightSubsystem.RobotYDutyCycle()* LimelightConstants.MaxAimSpeed) // Drive left with negative X (left)
        .withRotationalRate(
          
        m_LimelightSubsystem.AimTargetYawDutyCycle()
          
          * MaxAngularRate) // Drive counterclockwise with negative X (left)
).execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.applyRequest(() ->
    new SwerveRequest.RobotCentric().withVelocityX(0) // Drive forward with negative Y (forward)
    .withVelocityY(0) // Drive left with negative X (left)
    .withRotationalRate(0) // Drive counterclockwise with negative X (left)
).execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 
        Math.abs(m_LimelightSubsystem.targetXError()) < 0.005 //0.05
        && Math.abs(m_LimelightSubsystem.reverseTargetZError()) < 0.014 //0.108
        && Math.abs(m_LimelightSubsystem.targetYawError()) < 0.04//0.05
    ;
  }
}
