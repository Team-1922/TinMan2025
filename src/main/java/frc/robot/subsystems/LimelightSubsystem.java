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

NetworkTable LeftLimelight = NetworkTableInstance.getDefault().getTable("limelight-left");
NetworkTable RightLimelight = NetworkTableInstance.getDefault().getTable("limelight-right");
NetworkTableEntry ltx = LeftLimelight.getEntry("tx");
NetworkTableEntry rtx = RightLimelight.getEntry("tx");
NetworkTableEntry ltv = LeftLimelight.getEntry("tv");
NetworkTableEntry rtv = RightLimelight.getEntry("tv");

/** @return Left limelight <b>tx</b> */
 public double getLeftTx(){
  return ltx.getDouble(0.0);
 }

/** @return Right limelight <b>tx</b> */
 public double getRightTx(){
  return rtx.getDouble(0.0);
 }

 public boolean rightHasTarget(){
  return rtv.getDouble(0) ==1;
 }

 public boolean leftHasTarget(){
  return ltv.getDouble(0) ==1;
 }

 public double getLeftLimelightTargetValue(){
  return (getLeftTx()/LimelightConstants.LeftMaxAngle/2);// divide by 2 so it doesn't go so fast, tune this later
 }

 public double getRightLimelightTargetValue(){
  return (getRightTx()/LimelightConstants.RightMaxAngle/2);
 }

 /** */
 public void PutTXonDashboard(){
  SmartDashboard.putNumber("LeftTarget", ltx.getDouble(0)/LimelightConstants.LeftMaxAngle/2);
 }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
