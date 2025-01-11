// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {  }

NetworkTable LeftLimelight = NetworkTableInstance.getDefault().getTable("Left");
NetworkTableEntry tx = LeftLimelight.getEntry("tx");


/** @return Left limelight <b>tx</b> */
 public double getLeftTx(){
  return tx.getDouble(0.0);
 }


 public double LeftLimelightTargetValue(){
  return (getLeftTx()/LimelightConstants.LeftMaxAngle);
 }


  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
