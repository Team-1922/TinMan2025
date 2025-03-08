// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;


public class LimelightSubsystem extends SubsystemBase {

  String m_LimelightSide;
  NetworkTable m_LLNetworkTable;
  double[] m_Pos;
  NetworkTableEntry m_tv;
  double m_TargetLeftEdge;
  double m_TargetRightEdge;
  double m_TargetCenter;
  double m_AimingSpeedMultiplier;

    /** Creates a new LimelightSubsystem.
     * @param LimelightSide the limelight called, either <b>"left"</b> or <b>"right"
     */
  public LimelightSubsystem(String LimelightSide ) { 
    m_LimelightSide = LimelightSide;
    m_LLNetworkTable =NetworkTableInstance.getDefault().getTable("limelight-"+m_LimelightSide);
    if(LimelightSide == "left")
    {
      m_TargetLeftEdge = LimelightConstants.rightTargetLeftEdge; 
      m_TargetRightEdge = LimelightConstants.rightTargetRightEdge;
      m_TargetCenter = LimelightConstants.rightTargetCenter;
      m_AimingSpeedMultiplier = LimelightConstants.AimingSpeedMultiplier;
    }
    else{
      m_TargetLeftEdge = LimelightConstants.leftTargetLeftEdge; 
      m_TargetRightEdge = LimelightConstants.leftTargetRightEdge;
      m_TargetCenter = LimelightConstants.leftTargetCenter;
      m_AimingSpeedMultiplier = LimelightConstants.AimingSpeedMultiplier;
    }

   }

/** updates the target values */ 
   private void UpdateData(){
     m_tv = m_LLNetworkTable.getEntry("tv"); // 0 if no target, 1 if it has a target

   m_Pos = 
      m_LLNetworkTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]); // tx,ty,tz,pitch,yaw,roll (meters, deg)
   }

/** @return limelight <b>tx</b> (meters) */
 public double getTx(){
  return m_Pos[0];
 }

   /** @return limelight <b>ty</b> (meters) */
 public double getTy(){
  return m_Pos[1];
  }

/** @return limelight <b>Yaw</b> (Rad) */
 public double getYaw(){
  return m_Pos[4] * Math.PI/180;
   }

  /** checks if limelight sees an apriltag */
 public boolean HasTarget(){
  return m_tv.getDouble(0) ==1;
 }

 public double GetTz(){
  return m_Pos[2];

 }

 /** the value to use for apriltag aiming lattarlly  */
 public double AimTargetXDutyCycle(){
  double target;
  double Error =  m_TargetCenter-getTx();
  double FF;
  if(m_LimelightSide == "left"){
    FF= LimelightConstants.RightLateralFF;
  }else{
    FF= LimelightConstants.LeftLateralFF;
  }

    target= MathUtil.clamp(
 (
    Error
 //     /(m_TargetLeftEdge-m_TargetRightEdge)
      *m_AimingSpeedMultiplier
  + Math.copySign(FF, Error)

  ),-0.8,0.8); 


  SmartDashboard.putNumber("LLtarget", target);



 if(HasTarget()){
 return 
 target;
 } else{
  return 0; // so it wont move if you cant see an apriltag
 }
 
 }

 /** the value to use for apriltag aiming rotation, applies a deadband in method, Target Angle is 0 */
 public double AimTargetYawDutyCycle(){
  double target = MathUtil.clamp( ((LimelightConstants.TargetYaw-getYaw())*LimelightConstants.AimingTurnSpeedMultiplier),-0.8,0.8);
  SmartDashboard.putNumber("YeeHaw Target", target);
  return target;
 
 }

 public double AimTargetZDutyCycle(){
double target =
 MathUtil.clamp(  GetTz()
 /(LimelightConstants.MaxTargetZ/LimelightConstants.TargetZ) * LimelightConstants.TargetZSpeedMultiplier,-0.8,0.8);

 SmartDashboard.putNumber("ZSpeed", target);
  return 
   
  target;
 }

public double RobotXDutyCycle(){
 double target = MathUtil.clamp((-Math.sin(getYaw())*AimTargetXDutyCycle())+(Math.cos(getYaw())
 
 *AimTargetZDutyCycle()),-.8,.8);
SmartDashboard.putNumber("LlRobotX", target);
 return target;

}


public double RobotYDutyCycle(){
  double target = MathUtil.clamp((Math.cos(getYaw())*AimTargetXDutyCycle())+(Math.sin(getYaw())*AimTargetZDutyCycle()),-.8,.8);
 SmartDashboard.putNumber("LlRobotY", target);
  return 
   target;
}

  @Override
  public void periodic() {
    UpdateData();
    // This method will be called once per scheduler run
  }
}
