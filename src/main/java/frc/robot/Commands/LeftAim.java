// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import static edu.wpi.first.units.Units.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LeftAim extends Command {
  LimelightSubsystem m_LimelightSubsystem;
  CommandSwerveDrivetrain m_Drivetrain = TunerConstants.createDrivetrain();
  CommandXboxController m_DriveController;
  double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  /** Creates a new LeftAim. */
  public LeftAim(
LimelightSubsystem LimeLightSub, CommandSwerveDrivetrain drivetrain,CommandXboxController driveController
 ) {
  m_LimelightSubsystem = LimeLightSub;
  m_Drivetrain = drivetrain;
  m_DriveController = driveController;
  addRequirements(m_LimelightSubsystem,m_Drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LimelightSubsystem.getLeftTx();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     /* Setting up bindings for necessary control of the swerve drive platform */
       SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
     .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
     .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  
 m_LimelightSubsystem.PutTXonDashboard();

    m_Drivetrain.applyRequest(() ->
    drive.withVelocityX(-MathUtil.applyDeadband(m_LimelightSubsystem.getLeftLimelightTargetValue(),0.05) * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-MathUtil.applyDeadband(m_DriveController.getLeftX(),0.15) * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-MathUtil.applyDeadband(m_DriveController.getRightX(),0.15) * MaxAngularRate) // Drive counterclockwise with negative X (left)
).execute(); // if .execute doesn't work try .initialize
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
