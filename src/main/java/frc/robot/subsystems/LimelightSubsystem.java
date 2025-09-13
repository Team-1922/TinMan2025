// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.scoringPositions;

public class LimelightSubsystem extends SubsystemBase {

  String m_LimelightSide;
  NetworkTable m_LLNetworkTable;
  double[] m_Pos;
  NetworkTableEntry m_tv;
  double m_TargetCenter;
  double m_AimingSpeedMultiplier;
  Pose2d m_Pos2D;
  Rotation2d m_Rotation;
  Field2d m_Field2d = new Field2d();
  CommandSwerveDrivetrain m_driveTrain;
  int m_targetID;

  /**
   * Creates a new LimelightSubsystem.
   * 
   * @param LimelightSide the limelight called, either <b>"left"</b> or <b>"right"
   */
  public LimelightSubsystem(String LimelightSide) {
    m_LimelightSide = LimelightSide;
    m_LLNetworkTable = NetworkTableInstance.getDefault().getTable("limelight-" + m_LimelightSide);
    if (LimelightSide == "left") {
      m_TargetCenter = LimelightConstants.rightTargetCenter;
      m_AimingSpeedMultiplier = LimelightConstants.AimingSpeedMultiplier;
    } else {
      m_TargetCenter = LimelightConstants.leftTargetCenter;
      m_AimingSpeedMultiplier = LimelightConstants.AimingSpeedMultiplier;

    }
    SmartDashboard.putData("Field", m_Field2d);
  }

  /** updates the target values */
  public void UpdateData() {
        m_tv = m_LLNetworkTable.getEntry("tv"); // 0 if no target, 1 if it has a target
    if(m_tv.getInteger(0) == 1){
      m_targetID = (int)m_LLNetworkTable.getEntry("tid").getDouble(0);
    }
    // SmartDashboard.putBoolean("HasTarget");
    m_Pos = m_LLNetworkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[12]); // tx,ty,tz,pitch,yaw,roll (meters, deg)
    m_Rotation = new Rotation2d(getYaw());
    m_Pos2D = new Pose2d(getTx(), getTy(), m_Rotation);
    
    if(HasTarget()){
      m_Field2d.setRobotPose(m_Pos2D);
    }
  }

  public String getLimelightSide(){
    return m_LimelightSide;
  }

/** @return limelight <b>tx</b> (meters) */
  public double getTx(){
    return m_Pos[0];
  }

   /** @return limelight <b>ty</b> (meters) */
  public double getTy(){
    return m_Pos[1];
  }
  
  public double getYaw(){
    return m_Pos[5] * Math.PI/180;
  }

  /** checks if limelight sees an apriltag */
  public boolean HasTarget(){
    return m_tv.getDouble(0) ==1;
  }

  public double GetTz(){
    return m_Pos[2];
  }

  public double targetXError() {
    return m_TargetCenter-getTx();
  }

  public double targetZError() {
    return -LimelightConstants.TargetZ+GetTz();
  }

  public double reverseTargetZError(){
    return -LimelightConstants.L2TargetZ+GetTz();
  }

  public double targetYawError() {
    return LimelightConstants.TargetYaw-getYaw();
  }

  public Pose2d getScoringPose2d(){
    if(m_targetID == 17){
      return scoringPositions.BLUE_FRONT_RIGHT;
    }
    if(m_targetID == 18){
      return scoringPositions.BLUE_FRONT;
    }
    if(m_targetID == 19){
      return scoringPositions.BLUE_FRONT_LEFT;
    }
    if(m_targetID == 20){
      return scoringPositions.BLUE_BACK_LEFT;
    }
    if(m_targetID == 21){
      return scoringPositions.BLUE_BACK;
    }
    if(m_targetID == 22){
      return scoringPositions.BLUE_BACK_RIGHT;
    }
    if(m_targetID == 6){
      return scoringPositions.RED_FRONT_LEFT;
    }
    if(m_targetID == 7){
      return scoringPositions.RED_FRONT;
    }
    if(m_targetID == 8){
      return scoringPositions.RED_FRONT_RIGHT;
    }
    if(m_targetID == 9){
      return scoringPositions.RED_BACK_RIGHT;
    }
    if(m_targetID == 10){
      return scoringPositions.RED_BACK;
    }
    if(m_targetID == 11){
      return scoringPositions.RED_BACK_LEFT;
    }
    return null;
  }
  public double AimTargetXDutyCycle(){
    if (! HasTarget()) {
      return 0;
    }
    double target;
    double Error =  targetXError();
    double FF;
    if(m_LimelightSide == "left"){
      FF= LimelightConstants.RightLateralFF;
    }else{
      FF= LimelightConstants.LeftLateralFF;
    }
    target= MathUtil.clamp(
      (
        Error
 //     (m_TargetLeftEdge-m_TargetRightEdge)
        * m_AimingSpeedMultiplier
        + Math.copySign(FF, Error)
      ),
    -0.8,0.8); 
    SmartDashboard.putNumber("LLtarget", Error);
    return target;
  }

 /** the value to use for apriltag aiming rotation, applies a deadband in method, Target Angle is 0 */
  public double AimTargetYawDutyCycle(){
    if (! HasTarget()) return 0;
    double target = MathUtil.clamp( ((targetYawError())*LimelightConstants.AimingTurnSpeedMultiplier),-0.8,0.8);
    SmartDashboard.putNumber("YeeHaw Target", targetYawError());
    return target;
  }


  public double L2AimTargetZDutyCycle(){
    if (! HasTarget()) return 0;
    double target =
      MathUtil.clamp(reverseTargetZError()* LimelightConstants.TargetZSpeedMultiplier,-0.8,0.8);
    SmartDashboard.putNumber("ZSpeed", reverseTargetZError());
    return target;
  }

  public double L2RobotXDutyCycle(){
    if (!HasTarget()) return 0;
    double target = MathUtil.clamp((-Math.sin(getYaw())*AimTargetXDutyCycle())+(Math.cos(getYaw())*L2AimTargetZDutyCycle()),-.8,.8);
    SmartDashboard.putNumber("LlRobotX", target);
    return target;
  }

  public double AimTargetZDutyCycle(){
    if (! HasTarget()) return 0;
    double target =
      MathUtil.clamp(targetZError()* LimelightConstants.TargetZSpeedMultiplier,-0.8,0.8);
    SmartDashboard.putNumber("ZSpeed", targetZError());
    return target;
  }

  public double RobotXDutyCycle(){
    if (!HasTarget()) return 0;
    double target = MathUtil.clamp((-Math.sin(getYaw())*AimTargetXDutyCycle())+(Math.cos(getYaw())*AimTargetZDutyCycle()),-.8,.8);
    SmartDashboard.putNumber("LlRobotX", target);
    return target;
  }


  public double RobotYDutyCycle(){
    if (!HasTarget()) return 0;
    double target = MathUtil.clamp((Math.cos(getYaw())*AimTargetXDutyCycle())+(Math.sin(getYaw())*AimTargetZDutyCycle()),-.8,.8);
    SmartDashboard.putNumber("LlRobotY", target);
    return target;
  }

  @Override
  public void periodic() {
    UpdateData();
    // This method will be called once per scheduler run
  }
}