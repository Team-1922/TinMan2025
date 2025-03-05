// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {

  String m_LimelightSide;
  NetworkTable m_LLNetworkTable;
  double[] m_Pos;
  NetworkTableEntry m_tv;
    /** Creates a new LimelightSubsystem.
     * @param LimelightSide the limelight called, either <b>"left"</b> or <b>"right"
     */
  public LimelightSubsystem(String LimelightSide) { 
    m_LimelightSide = LimelightSide;
    m_LLNetworkTable =NetworkTableInstance.getDefault().getTable("limelight-"+m_LimelightSide);
    
   }

/** updates the target values */ 
   private void UpdateData(){
     m_tv = m_LLNetworkTable.getEntry("tv"); // 0 if no target, 1 if it has a target

   m_Pos = 
      m_LLNetworkTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]); // tx,ty,tz,pitch,yaw,roll
   }

/** @return limelight <b>tx</b> (meters) */
 public double getTx(){
  return m_Pos[0];
 }

   /** @return limelight <b>ty</b> (meters) */
 public double getTy(){
  return m_Pos[1];
  }

/** @return limelight <b>Yaw</b> (deg) */
 public double getYaw(){
  return m_Pos[4];
   }

  /** checks if limelight sees an apriltag */
 public boolean HasTarget(){
  return m_tv.getDouble(0) ==1;
 }


 // -3 for the left LL, -4 for the right one...
 /** the value to use for apriltag aiming, applies a deadband in method */
 public double AimTargetYDutyCycle(){
  return (MathUtil.applyDeadband(getTy(),LimelightConstants.TargetYDeadband));
 }

 /** the value to use for apriltag aiming rotation, applies a deadband in method */
 public double AimTargetYawDutyCycle(){
  return (MathUtil.applyDeadband(getYaw(),LimelightConstants.TargetYawDeadband));
 }

  @Override
  public void periodic() {
    UpdateData();
    // This method will be called once per scheduler run
  }
}
