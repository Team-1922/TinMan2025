// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

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
  TimeOfFlight m_TOF = new TimeOfFlight(TimeOfFlightConstants.TOFID);
  CANcoder m_armEncoder = new CANcoder(EndEffectorConstants.endEffectorArmEncoderID,"Elevator");
  CANcoder m_WristEncoder = new CANcoder(EndEffectorConstants.endEffectorWristEncoderID,"Elevator");
  
  NetworkTableInstance m_networkTable = NetworkTableInstance.getDefault();
  
  /** Creates a new EndEffector. */
  public EndEffector() {
    m_leftCollect.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
    m_rightCollect.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
    m_WristMotor.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.ArmFeedbackConfigs);
    m_WristMotor.getConfigurator().apply(EndEffectorConstants.WristFeedbackConfigs);

  }

            // COLLECTOR CODE

  public void collect(){
    m_leftCollect.setControl(new VelocityDutyCycle(EndEffectorConstants.collectorRPM));
    m_rightCollect.setControl(new VelocityDutyCycle(EndEffectorConstants.collectorRPM));
  }

  public void stopCollector(){
    m_rightCollect.set(0);
    m_leftCollect.set(0);
  }

  public void stopWrist(){
    m_WristMotor.set(0);
  }

  public void stopArm(){
    m_ArmMotor.set(0);
  }

  public void stopEE(){
    stopCollector();
    stopWrist();
    stopArm();
  }


  public void ReverseCollector(){
    m_leftCollect.setControl(new VelocityDutyCycle(-EndEffectorConstants.collectorRPM));// maybe make them a seperate constant, if needed
    m_rightCollect.setControl(new VelocityDutyCycle(-EndEffectorConstants.collectorRPM));
  }

            // WRIST CODE

  /** sets the current angle of the end effector wrist as the reference angle */
  public void setWristAngleAsReference(){
    SmartDashboard.putNumber("EEReference", m_WristEncoder.getPosition().getValueAsDouble());
  }

  /** gets current angle of the end effector */
  public double getCurrentWristAngle(){
    return m_WristMotor.getPosition().getValueAsDouble();
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

  public void ToL4WristAngle(){
    m_WristMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.L4WristAngle));
  }

  public void ToStationWristAngle(){
    m_WristMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.StationWristAngle));
  }

  public void ToStartingWristAngle(){
    m_WristMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.StartingWristAngle));
  }

  public void ToAlgaeWristAngle(){
    m_WristMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.AlgaeWristAngle));
  }

           // ARM CODE

      /** checks if the arm encoders absolute position has overflowed, this is to prevent the arm going to far back and thinking it's in the floor 
       * @return if the absolute position of the arm has overflowed to 0
      */
  public boolean checkPositionOverflow(){

    if(m_armEncoder.getAbsolutePosition().getValueAsDouble() < 0.1){
      return true;
    }
    else{return false;}
  }


  /** sets the current angle of the end effector as the reference angle */
  public void setArmAngleAsReference(){
    SmartDashboard.putNumber("EEArmReference", m_armEncoder.getPosition().getValueAsDouble());
  }

  /** gets current angle of the arm on the end effector */
  public double getCurrentArmAngle(){
    
    return m_armEncoder.getAbsolutePosition().getValueAsDouble()-SmartDashboard.getNumber("EEArmReference", 0);
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

  public void ToL4ArmAngle(){
    m_ArmMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.L4ArmAngle));
  }

  /**  angles arm to collect at station 
  */
  public void ToStationArmAngle(){
    m_ArmMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.StationArmAngle-SmartDashboard.getNumber("EEArmReference", 0)));
  }

  /** angles arm to starting configuration */
  public void ToStartingArmAngle(){
    m_ArmMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.StartingArmAngle-SmartDashboard.getNumber("EEArmReference", 0)));
  }
  
  public void ToAlgaeArmAngle(){
    m_ArmMotor.setControl( new MotionMagicExpoDutyCycle(EndEffectorConstants.AlgaeArmAngle-SmartDashboard.getNumber("EEArmReference", 0)));
  }

  /** angles EE at <b>L1 */
  public void L1(){
    ToL1ArmAngle();
    ToL1WristAngle();
  }

  /** angles EE at <b>L2</b>, can also be used for <b>L3</b> */
  public void L2(){
    ToL2ArmAngle();
    ToL2WristAngle();
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

            // LED+TOF CODE
  
  /** clears animation running in given animation slot */
  public void stopAnimation(int AnimationSlot){
    m_Candle.clearAnimation(AnimationSlot);
  }

  /** @return if something is within the TOF target range
   */
  public boolean HasCoral(){
    return TimeOfFlightConstants.MaxRange > m_TOF.getRange() && m_TOF.getRange() >TimeOfFlightConstants.MinRange;
  }

  public void LEDGreen(){
    m_Candle.setLEDs(0, 255, 0, 0, 0, 99);
  }

  private void LEDControl(){ // placeholder
    if(HasCoral()){
      m_Candle.setLEDs(0,255,0,0,0,LEDConstants.TotalLEDs); // does have coral, turn LEDs green 
    } else{
      m_Candle.setLEDs(255,0,0,0,0,LEDConstants.TotalLEDs); // doesn't have coral, turn LEDs red
    }
  }

  public void disabledAnimation(){
    m_Candle.animate( new RainbowAnimation()

    );
  }


  @Override
  public void periodic() {
    LEDGreen();
    //LEDControl();
    // This method will be called once per scheduler run
  }
}