// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;


public class EndEffector extends SubsystemBase {
  TalonFX m_leftCollect = new TalonFX(EndEffectorConstants.leftCollectorMotorID, "Elevator");
  TalonFX m_rightCollect = new TalonFX(EndEffectorConstants.rightCollectorMotorID, "Elevator");
  TalonFX m_WristMotor = new TalonFX(EndEffectorConstants.endEffectorWristMotorID, "Elevator");
  TalonFX m_ArmMotor = new TalonFX(EndEffectorConstants.endEffectorArmMotorID, "Elevator");


  TimeOfFlight m_TOF = new TimeOfFlight(TOFConstants.TOFID);
  TimeOfFlight m_TOF2 = new TimeOfFlight(TOFConstants.TOFID2);
  CANcoder m_armEncoder = new CANcoder(EndEffectorConstants.endEffectorArmEncoderID,"Elevator");
  CANcoder m_WristEncoder = new CANcoder(EndEffectorConstants.endEffectorWristEncoderID,"Elevator");  
  NetworkTableInstance m_networkTable = NetworkTableInstance.getDefault();
  DoublePublisher m_armPos = m_networkTable.getDoubleTopic("ArmPos").publish();
  DoublePublisher m_wristPos = m_networkTable.getDoubleTopic("WristPos").publish();
  double m_TOFmeasurement;
  double m_TOF2Measurement;

  /** Creates a new EndEffector. */
  public EndEffector() {
 
    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.ArmFeedbackConfigs);  
    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.ArmSlot0Configs);
    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.ArmMotorConfig);
  //  m_ArmMotor.getConfigurator().apply(EndEffectorConstants.M_ArmOutputConfigs);
        
    m_WristMotor.getConfigurator().apply(EndEffectorConstants.WristSlot0Configs);
  
    m_WristMotor.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
    m_WristMotor.getConfigurator().apply(EndEffectorConstants.WristFeedbackConfigs);
    

    m_WristEncoder.getConfigurator().apply(EndEffectorConstants.WristCanCoderConfig);
    m_armEncoder.getConfigurator().apply(EndEffectorConstants.ArmCanCoderConfig);

    m_leftCollect.getConfigurator().apply(EndEffectorConstants.CollectorCurrentLimitConfigs);
    m_rightCollect.getConfigurator().apply(EndEffectorConstants.CollectorCurrentLimitConfigs);    
    m_leftCollect.setControl(new Follower(EndEffectorConstants.rightCollectorMotorID, true));
    
    m_TOF.setRangingMode(RangingMode.Short, 25);
    m_TOF2.setRangingMode(RangingMode.Short, 25);

    
    /*
    //m_ArmMotor.getConfigurator().apply(EndEffectorConstants.closedLoopRampConfigs);
    m_WristMotor.getConfigurator().apply(EndEffectorConstants.closedLoopRampConfigs);
    m_leftCollect.getConfigurator().apply(EndEffectorConstants.closedLoopRampConfigs);
    m_rightCollect.getConfigurator().apply(EndEffectorConstants.closedLoopRampConfigs);

    //m_ArmMotor.getConfigurator().apply(EndEffectorConstants.openLoopRampConfigs);
    m_WristMotor.getConfigurator().apply(EndEffectorConstants.openLoopRampConfigs);
    m_leftCollect.getConfigurator().apply(EndEffectorConstants.openLoopRampConfigs);
    m_rightCollect.getConfigurator().apply(EndEffectorConstants.openLoopRampConfigs);
    */

    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.ArmMotionMagicConfigs);
    m_WristMotor.getConfigurator().apply(EndEffectorConstants.WristMotionMagicConfigs);
  }

  public void ConfigEeCoast(){
    m_WristMotor.getConfigurator().apply(EndEffectorConstants.WristMotorConfig);
    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.ArmMotorConfig);
  }



  public void ConfigEeBrake(){
    m_WristMotor.getConfigurator().apply(EndEffectorConstants.EEClimbedConfigs);
    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.EEClimbedConfigs);    

  }


            // COLLECTOR CODE
 /** spins motor at speed given, percent output */
  public void collect(double speed){
    m_rightCollect.set(speed);
  }

  public void stopCollector(){
    m_rightCollect.set(0);

  }

  public void stopWrist(){
    m_WristMotor.set(0);
  }

  public void stopArm(){
    m_ArmMotor.set(0);
  }

  /** stops the arm, wrist and collector */
  public void stopEE(){
    stopCollector();
    stopWrist();
    stopArm();
  }


  public void ReverseCollector(double speed){
  
    m_rightCollect.set(speed);
  }

            // WRIST CODE

  


  /** gets current angle of the end effector */
  public double getCurrentWristAngle(){
    double angle =  m_WristMotor.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("CurrentWristAngle", angle);
    return angle;
    //m_ArmMotor.getpo
  }

  /**
   * moves wrist to given location
   * @param targetAngle the target position
   */
  public void ToWristAngle(double targetAngle){
    SmartDashboard.putNumber("WristTarget", targetAngle);
    m_WristMotor.setControl(new MotionMagicExpoDutyCycle(targetAngle));
  }





           // ARM CODE




  /** gets current angle of the arm on the end effector */
  public double getCurrentArmAngle(){
   double angle= m_ArmMotor.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("currentArmAngle",angle );
    return angle;

  }

  public void ToArmAngle(double TargetPos){
    SmartDashboard.putNumber("ArmTarget", TargetPos);
    m_ArmMotor.setControl(new MotionMagicExpoDutyCycle(TargetPos));
  }


/** logging method for this subsystem it logs 
 * <ul>
 * <p> Wrist Position
 * <p> Arm Position
 */
  public void EELogging(){
    SignalLogger.writeDouble("WristPosition", getCurrentWristAngle());
    SignalLogger.writeDouble("ArmPosition", getCurrentArmAngle());
  }

 
public double GetTofMeasurement(){
return m_TOFmeasurement;
}
  
public double GetTof2Measurement(){
 return m_TOF2Measurement;
} 
 
public void SetTofMeasurement(){
  if(m_TOF.isRangeValid()){
  m_TOFmeasurement = m_TOF.getRange();}

 // if(m_TOF2.isRangeValid()){
 //   m_TOF2Measurement = m_TOF2.getRange();
 // }
}


public void PutTOFonSmartdashboard(){
  SmartDashboard.putNumber("TOFtestValues", m_TOFmeasurement);
}

            // LED+LazerCan CODE
  

  /** @return if something is within the TOF target range
   */
  public boolean HasCoral(){

// m_TOF.getRange();

  return 
  m_TOFmeasurement >= TOFConstants.TOFMinDistance &&
  m_TOFmeasurement <= TOFConstants.TOFMaxDistance;
  }

  /** checks the 2nd TOF if it sees something, only used for station pickup */
  public boolean HasStationCoral(){
  //  double measurement = m_TOF2.getRange();
    return 
    m_TOF2Measurement >= TOFConstants.TOF2MinDistance &&
    m_TOF2Measurement <= TOFConstants.TOF2MaxDistance;

  }

  public void putTOFTargetOnDashboard(){
   //double measurement = m_TOF.getRange();
   //double measurement2 = m_TOF2.getRange();
    if(m_TOF.isRangeValid()){
    SmartDashboard.putNumber("TOFValue", m_TOFmeasurement);}
   // if(m_TOF2.isRangeValid()){
   //   SmartDashboard.putNumber("TOF2Value", m_TOF2Measurement);}
      
  }








  @Override
  public void periodic() {
   putTOFTargetOnDashboard();
    SetTofMeasurement();
   // PutTOFonSmartdashboard();
    getCurrentArmAngle();
    getCurrentWristAngle();
    
    // This method will be called once per scheduler run
  }
}