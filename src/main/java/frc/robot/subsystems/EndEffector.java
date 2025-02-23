// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
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
  TalonFX m_leftCollect = new TalonFX(EndEffectorConstants.leftCollectorMotorID);
  TalonFX m_rightCollect = new TalonFX(EndEffectorConstants.rightCollectorMotorID); // change these ids later
  TalonFX m_WristMotor = new TalonFX(EndEffectorConstants.endEffectorWristMotorID);
  TalonFX m_ArmMotor = new TalonFX(EndEffectorConstants.endEffectorArmMotorID);
  CANdle m_Candle = new CANdle(LEDConstants.CandleID);
  TimeOfFlight m_TOF = new TimeOfFlight(TimeOfFlightConstants.TOFID);
  CANcoder m_armEncoder = new CANcoder(EndEffectorConstants.endEffectorArmEncoderID);
  NetworkTableInstance m_networkTable = NetworkTableInstance.getDefault();
  
  /** Creates a new EndEffector. */
  public EndEffector() {
    m_leftCollect.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
    m_rightCollect.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
    m_WristMotor.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
    m_ArmMotor.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
    //m_armEncoder.getAbsolutePosition();
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

  public void ReverseCollector(){
    m_leftCollect.setControl(new VelocityDutyCycle(-EndEffectorConstants.collectorRPM));// maybe make them a seperate constant, if needed
    m_rightCollect.setControl(new VelocityDutyCycle(-EndEffectorConstants.collectorRPM));
  }

            // WRIST CODE

  /** sets the current angle of the end effector wrist as the reference angle */
  public void setWristAngleAsReference(){
    SmartDashboard.putNumber("EEReference", m_WristMotor.getPosition().getValueAsDouble());
  }

  /** gets current angle of the end effector */
  public double getCurrentWristAngle(){
    return m_WristMotor.getPosition().getValueAsDouble()-SmartDashboard.getNumber("EEWristReference", 0);
  }

  /**angles the end effector at the floor */
  public void ToFloorWristAngle(){
    m_WristMotor.setControl( new PositionDutyCycle(EndEffectorConstants.FloorWristAngle-SmartDashboard.getNumber("EEWristReference",0)));
  }

  public void ToL1WristAngle(){
    m_WristMotor.setControl( new PositionDutyCycle(EndEffectorConstants.L1WristAngle-SmartDashboard.getNumber("EEWristReference", 0)));
  }

  public void ToL2WristAngle(){
    m_WristMotor.setControl( new PositionDutyCycle(EndEffectorConstants.L2WristAngle-SmartDashboard.getNumber("EEWristReference",0)));
  }

  public void ToL4WristAngle(){
    m_WristMotor.setControl( new PositionDutyCycle(EndEffectorConstants.L4WristAngle-SmartDashboard.getNumber("EEWristReference",0)));
  }

  public void ToStationWristAngle(){
    m_WristMotor.setControl( new PositionDutyCycle(EndEffectorConstants.StationWristAngle-SmartDashboard.getNumber("EEWristReference", 0)));
  }

  public void ToStartingWristAngle(){
    m_WristMotor.setControl( new PositionDutyCycle(EndEffectorConstants.StartingWristAngle-SmartDashboard.getNumber("EEWristReference", 0)));
  }

  public void ToAlgaeWristAngle(){
    m_WristMotor.setControl( new PositionDutyCycle(EndEffectorConstants.AlgaeWristAngle-SmartDashboard.getNumber("EEWristReference", 0)));
  }

           // ARM CODE

  /** sets the current angle of the end effector as the reference angle */
  public void setArmAngleAsReference(){
    SmartDashboard.putNumber("EEArmReference", m_ArmMotor.getPosition().getValueAsDouble());
  }

  /** gets current angle of the arm on the end effector */
  public double getCurrentArmAngle(){
    return m_ArmMotor.getPosition().getValueAsDouble()-SmartDashboard.getNumber("EEArmReference", 0);
  }

  /**angles the end effector arm to the floor */
  public void ToFloorArmAngle(){
    m_ArmMotor.setControl( new PositionDutyCycle(EndEffectorConstants.FloorArmAngle-SmartDashboard.getNumber("EEArmReference",0)));
  }

  public void ToL1ArmAngle(){
    m_ArmMotor.setControl( new PositionDutyCycle(EndEffectorConstants.L1ArmAngle-SmartDashboard.getNumber("EEArmReference", 0)));
  }

  public void ToL2ArmAngle(){
    m_ArmMotor.setControl( new PositionDutyCycle(EndEffectorConstants.L2ArmAngle-SmartDashboard.getNumber("EEArmReference",0)));
  }

  public void ToL4ArmAngle(){
    m_ArmMotor.setControl( new PositionDutyCycle(EndEffectorConstants.L4ArmAngle-SmartDashboard.getNumber("EEArmReference",0)));
  }

  /**  angles arm to collect at station */
  public void ToStationArmAngle(){
    m_ArmMotor.setControl( new PositionDutyCycle(EndEffectorConstants.StationArmAngle-SmartDashboard.getNumber("EEArmReference", 0)));
  }

  /** angles arm to starting configuration */
  public void ToStartingArmAngle(){
    m_ArmMotor.setControl( new PositionDutyCycle(EndEffectorConstants.StartingArmAngle-SmartDashboard.getNumber("EEArmReference", 0)));
  }
  
  public void ToAlgaeArmAngle(){
    m_ArmMotor.setControl( new PositionDutyCycle(EndEffectorConstants.AlgaeArmAngle-SmartDashboard.getNumber("EEArmReference", 0)));
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

  private void LEDControl(){ // placeholder
    if(HasCoral()){
      m_Candle.setLEDs(0,255,0,0,0,LEDConstants.TotalLEDs); // does have coral, turn LEDs green 
    } else{
      m_Candle.setLEDs(255,0,0,0,0,LEDConstants.TotalLEDs); // doesn't have coral, turn LEDs red
    }
  }

  @Override
  public void periodic() {
    //LEDControl();
    // This method will be called once per scheduler run
  }
}