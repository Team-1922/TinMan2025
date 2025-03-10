// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.ConfigurationFailedException;
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
  CANdle m_Candle = new CANdle(LEDConstants.CandleID);// candle is on RIO canbus
  LaserCan m_CollectorSensor = new LaserCan(LazerCanConstants.LcID);
  
  
  CANcoder m_armEncoder = new CANcoder(EndEffectorConstants.endEffectorArmEncoderID,"Elevator");
  CANcoder m_WristEncoder = new CANcoder(EndEffectorConstants.endEffectorWristEncoderID,"Elevator");
  
  NetworkTableInstance m_networkTable = NetworkTableInstance.getDefault();
  DoublePublisher m_armPos = m_networkTable.getDoubleTopic("ArmPos").publish();
  DoublePublisher m_wristPos = m_networkTable.getDoubleTopic("WristPos").publish();
  

  /** Creates a new EndEffector. */
  public EndEffector() {
 
    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.ArmFeedbackConfigs);  
    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.ArmSlot0Configs);
    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.ArmMotorConfig);
    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.ArmClosedLoopRampConfigs);
    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.ArmOpenLoopRampConfigs);
    
    m_WristMotor.getConfigurator().apply(EndEffectorConstants.WristSlot0Configs);
    m_WristMotor.getConfigurator().apply(EndEffectorConstants.WristMotorConfig);
    m_WristMotor.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
    m_WristMotor.getConfigurator().apply(EndEffectorConstants.WristFeedbackConfigs);
    
    m_WristEncoder.getConfigurator().apply(EndEffectorConstants.WristCanCoderConfig);
    m_armEncoder.getConfigurator().apply(EndEffectorConstants.ArmCanCoderConfig);

    m_leftCollect.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
    m_rightCollect.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);    
    m_leftCollect.setControl(new Follower(EndEffectorConstants.rightCollectorMotorID, true));



  }

            // COLLECTOR CODE

  public void collect(){
 
   // m_rightCollect.setControl(new VelocityDutyCycle(EndEffectorConstants.collectorRPM));
    m_rightCollect.set(-0.2);
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


  public void ReverseCollector(){
  //  m_leftCollect.setControl(new VelocityDutyCycle(-EndEffectorConstants.collectorRPM));// maybe make them a seperate constant, if needed
    m_rightCollect.set(0.2);
  }

            // WRIST CODE

  /** sets the current angle of the end effector wrist as the reference angle */
  public void setWristAngleAsReference(){
   // SmartDashboard.putNumber("EEReference", m_WristEncoder.getPosition().getValueAsDouble());
  }

  /** gets current angle of the end effector */
  public double getCurrentWristAngle(){
    return m_WristMotor.getPosition().getValueAsDouble();
    //m_ArmMotor.getpo
  }

  /**angles the end effector at the floor */
  public void ToFloorWristAngle(){
    
    m_WristMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.FloorWristAngle));
  }

  public void ToL1WristAngle(){
    m_WristMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.L1WristAngle));
  }

  public void ToL2WristAngle(){
    m_WristMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.L2WristAngle));
  }

  public void ToL3WristAngle(){
    m_WristMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.L3WristAngle));
  }

  public void ToL4WristAngle(){
    m_WristMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.L4WristAngle));
  }

  public void ToStationWristAngle(){
    m_WristMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.StationWristAngle));
  }

  public void ToStowedWristAngle(){
    m_WristMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.StowedWristAngle));
  }

  public void ToAlgaeWristAngle(){
    m_WristMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.AlgaeWristAngle));
  }

  public void ToVerticalWristAngle(){
    m_WristMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.VerticalWristAngle));
  }


           // ARM CODE




  /** gets current angle of the arm on the end effector */
  public double getCurrentArmAngle(){
    return m_ArmMotor.getPosition().getValueAsDouble();

  }

  
  /**angles the end effector arm to the floor */
  public void ToFloorArmAngle(){
    m_ArmMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.FloorArmAngle));
  }

  public void ToL1ArmAngle(){
    m_ArmMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.L1ArmAngle));
  }

  public void ToL2ArmAngle(){
    m_ArmMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.L2ArmAngle));
  }
  
  public void ToL3ArmAngle(){
    m_ArmMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.L3ArmAngle));
  }

  public void ToL4ArmAngle(){
    m_ArmMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.L4ArmAngle));
  }

  /**  angles arm to collect at station 
  */
  public void ToStationArmAngle(){
    m_ArmMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.StationArmAngle));
  }

  /** angles arm to starting configuration */
  public void ToStowedArmAngle(){
    m_ArmMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.StowedArmAngle));
  }
  
  public void ToAlgaeArmAngle(){
    m_ArmMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.AlgaeArmAngle));
  }

  public void ToVerticalArmAngle(){
    m_ArmMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.VerticalArmAngle));
  }

  /** angles EE at <b>L1 */
  public void L1(){
    ToL1ArmAngle();
    ToL1WristAngle();
  }

  /** angles EE at <b>L2</b>*/
  public void L2(){
    ToL2ArmAngle();
    ToL2WristAngle();
  }

   /** angles EE at <b>L3</b>*/
  public void L3(){
    ToL3ArmAngle();
    ToL3WristAngle();
  }

  /** angles EE at <b>L4 */
  public void L4(){
    ToL4ArmAngle();
    ToL4WristAngle();
  }

    /** angles EE at <b>Algae */
    public void Algae(){
      ToAlgaeArmAngle();
      ToAlgaeWristAngle();
    }

  /**  angles EE at the station, to pickup */
  public void EEStation(){
    ToStationArmAngle();
    ToStationWristAngle();
  }

  /** angles EE at the floor */
  public void Floor(){
    ToFloorArmAngle();
    ToFloorWristAngle();
  }

  public void stowe(){
    ToStowedArmAngle();
    ToStowedWristAngle();
  }
  
  public void Vertical(){
    ToVerticalArmAngle();
    ToVerticalWristAngle();
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




            // LED+LazerCan CODE
  
  /** clears animation running in given animation slot */
  public void stopAnimation(int AnimationSlot){
    m_Candle.clearAnimation(AnimationSlot);
  }

  /** @return if something is within the Lc target range
   */
  public boolean HasCoral(){
Measurement measurement = m_CollectorSensor.getMeasurement();
if(measurement != null)
{
  return
  measurement.distance_mm > LazerCanConstants.LcMinDistance 
  && measurement.distance_mm < LazerCanConstants.LcMaxDistance;
  // do it the right way
}
else
{
  return false;
  // do it the failsafe way
}
  

  }

  /** turns LEDs green, mainly for testing if the wiring is correct */
  public void LEDGreen(){
    int LedCount = (int) SmartDashboard.getNumber("LedCount", 8);
    m_Candle.setLEDs(0, 255, 0, 0, 0,LedCount);
  }

  private void LEDControl(){ // placeholder
    if(HasCoral()){
      
      m_Candle.setLEDs(0,255,0,0,0,LEDConstants.TotalLEDs); // does have coral, turn LEDs green 
    } else{
      m_Candle.setLEDs(255,0,0,0,0,LEDConstants.TotalLEDs); // doesn't have coral, turn LEDs red
    }
  }

  public void disabledAnimation(){
    m_Candle.animate( new RainbowAnimation());
  }


  @Override
  public void periodic() {
   // LEDGreen();
    //EELogging();
    //LEDControl();
    // This method will be called once per scheduler run
  }
}