// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Localization extends SubsystemBase {

  String m_LimelightSide;
  NetworkTable m_LLNetworkTable;
  double[] m_Pos;
  NetworkTableEntry m_tv;
  double m_TargetCenter;
  double m_AimingSpeedMultiplier;
  Pose2d m_Pos2D;
  Rotation2d m_Rotation2d;
  Field2d m_Field2d = new Field2d();
  Telemetry m_Telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
  CommandSwerveDrivetrain m_driveTrain;
  private final Field2d m_field = new Field2d();

    /** Creates a new Localization.
     * @param LimelightSide the limelight called, either <b>"left"</b> or <b>"right"
     */
  public Localization( Telemetry telemetry, CommandSwerveDrivetrain commandSwerveDrivetrain
    //String LimelightSide
    ) { 
      m_driveTrain = commandSwerveDrivetrain;
      m_Telemetry = telemetry;
      m_LimelightSide = "left";
      // =NetworkTableInstance.getDefault().getTable("limelight-"+m_LimelightSide);
      m_LLNetworkTable = NetworkTableInstance.getDefault().getTable("limelight-"+m_LimelightSide);
      SmartDashboard.putData("Field", m_field);
      if(true)
        {
          m_TargetCenter = LimelightConstants.rightTargetCenter;
          m_AimingSpeedMultiplier = LimelightConstants.AimingSpeedMultiplier;
        }
        else{
          m_TargetCenter = LimelightConstants.leftTargetCenter;
          m_AimingSpeedMultiplier = LimelightConstants.AimingSpeedMultiplier;
        };
    }

/** updates the target values */ 
  private void UpdateData(){
    m_tv = m_LLNetworkTable.getEntry("tv"); // 0 if no target, 1 if it has a target
    //SmartDashboard.putBoolean("HasTarget");
    m_Pos = m_LLNetworkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[12]); // tx,ty,tz,pitch,yaw,roll (meters, deg)
    m_Rotation2d = new Rotation2d(getYaw());
    m_Pos2D = new Pose2d(getTx(), getTy(), m_Rotation2d);

    m_driveTrain.addVisionMeasurement(m_Pos2D, m_Telemetry.getTimeStamp());
    m_Field2d.setRobotPose(m_Telemetry.getPose2d());
    SmartDashboard.putData("Field", m_Field2d);
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

 /** the value to use for apriltag aiming lattarlly  */
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
 //     /(m_TargetLeftEdge-m_TargetRightEdge)
      *m_AimingSpeedMultiplier
  + Math.copySign(FF, Error)

  ),-0.8,0.8); 


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
  return 
   
  target;
 }

 public double L2RobotXDutyCycle(){
  if (!HasTarget()) return 0;
 double target = MathUtil.clamp((-Math.sin(getYaw())*AimTargetXDutyCycle())+(Math.cos(getYaw())
 
 *L2AimTargetZDutyCycle()),-.8,.8);
SmartDashboard.putNumber("LlRobotX", target);
 return target;

}


 public double AimTargetZDutyCycle(){
  if (! HasTarget()) return 0;
double target =
 MathUtil.clamp(targetZError()* LimelightConstants.TargetZSpeedMultiplier,-0.8,0.8);

 SmartDashboard.putNumber("ZSpeed", targetZError());
  return 
   
  target;
 }

public double RobotXDutyCycle(){
  if (!HasTarget()) return 0;
 double target = MathUtil.clamp((-Math.sin(getYaw())*AimTargetXDutyCycle())+(Math.cos(getYaw())
 *AimTargetZDutyCycle()),-.8,.8);
SmartDashboard.putNumber("LlRobotX", target);
 return target;
}


public double RobotYDutyCycle(){
  if (!HasTarget()) return 0;
  double target = MathUtil.clamp((Math.cos(getYaw())*AimTargetXDutyCycle())+
  (Math.sin(getYaw())*AimTargetZDutyCycle()),-.8,.8);
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
