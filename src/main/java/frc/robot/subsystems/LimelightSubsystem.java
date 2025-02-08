// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {  }

NetworkTable LeftLimelight = NetworkTableInstance.getDefault().getTable("Left");
NetworkTable RightLimelight = NetworkTableInstance.getDefault().getTable("right");
NetworkTableEntry ltx = LeftLimelight.getEntry("ltx");
NetworkTableEntry rtx = RightLimelight.getEntry("rtx");


/** @return Left limelight <b>tx</b> */
 public double getLeftTx(){
  return ltx.getDouble(0.0);
 }

/** @return Right limelight <b>tx</b> */
 public double getRightTx(){
  return rtx.getDouble(0.0);
 }


 public double getLeftLimelightTargetValue(){
  return (getLeftTx()/LimelightConstants.LeftMaxAngle);
 }

 public double getRightLimelightTargetValue(){
  return (getRightTx()/LimelightConstants.RightMaxAngle);
 }

 /** */
 public void PutTXonDashboard(){
  SmartDashboard.putNumber("LeftTarget", ltx.getDouble(0)/LimelightConstants.LeftMaxAngle);
 }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
