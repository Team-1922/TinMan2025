// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.Pigeon2;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LimelightSubsystem.*;
//import frc.robot.Constants.DriveTrainConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
private CommandSwerveDrivetrain m_drivetrain;
private CommandXboxController m_driveController;
private Pigeon2 m_pigeon2;
private double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond);
private LimelightSubsystem m_LL;
private double m_yawOffset;
private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() 
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
 
  public DriveCommand(CommandSwerveDrivetrain Drivetrain, CommandXboxController driveController, Pigeon2 Pigeon, LimelightSubsystem limelightSubsystem) {
    m_drivetrain = Drivetrain;
    m_driveController = driveController;
    m_pigeon2 = Pigeon;
    m_LL = limelightSubsystem;
    addRequirements(m_drivetrain);
    m_yawOffset = -(m_pigeon2.getYaw().getValueAsDouble() * Math.PI/180);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double roll = m_pigeon2.getPitch().getValueAsDouble();
    double pitch = m_pigeon2.getRoll().getValueAsDouble();
    double yaw = ((m_pigeon2.getYaw().getValueAsDouble() * Math.PI/180) + m_yawOffset)%(Math.PI*2);
    double xAdjustment = 0;
    double yAdjustment = 0;
    SmartDashboard.putNumber("yaw", yaw);

    if(pitch > 3){
      xAdjustment = .1 * Math.sin(yaw); //placeholder
      yAdjustment = .1 * Math.cos(yaw);
    }
    else if(pitch < -3){
      xAdjustment = -.1 * Math.sin(yaw); //placeholder
      yAdjustment = -.1 * Math.cos(yaw);
    }
     else if(roll > 3){
      xAdjustment = .1 * Math.sin(yaw + Math.PI/2); //placeholder
      yAdjustment = .1 * Math.cos(yaw + Math.PI/2);
    }
    else if(roll < -3){
      xAdjustment = -.1 * Math.sin(yaw + Math.PI/2); //placeholder
      yAdjustment = -.1 * Math.cos(yaw + Math.PI/2);
    }
    final double xAdj = xAdjustment;
    final double yAdj = yAdjustment;
    m_drivetrain.applyRequest(() ->
        drive.withVelocityX((-MathUtil.applyDeadband(m_driveController.getLeftY(),0.15) + xAdj) * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY((-MathUtil.applyDeadband(m_driveController.getLeftX(),0.15) + yAdj) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-MathUtil.applyDeadband(m_driveController.getRightX(),0.15) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
