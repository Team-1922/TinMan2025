// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.TimeOfFlightConstants;
import frc.robot.Constants.EndEffectorConstants;


public class EndEffector extends SubsystemBase {
  TalonFX m_leftCollect = new TalonFX(EndEffectorConstants.leftCollectorMotorID);
  TalonFX m_rightCollect = new TalonFX(EndEffectorConstants.rightCollectorMotorID); // change these ids later
  TalonFX m_AngleMotor = new TalonFX(EndEffectorConstants.endEffectorAngleMotorID);
  CANdle m_Candle = new CANdle(LEDConstants.CandleID);
  TimeOfFlight m_TOF = new TimeOfFlight(TimeOfFlightConstants.TOFID);
  /** Creates a new EndEffector. */
  public EndEffector() {
    m_leftCollect.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
    m_rightCollect.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
    m_AngleMotor.getConfigurator().apply(EndEffectorConstants.EECurrentLimitConfigs);
  }

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

  public void setAngleAsReference(){
    SmartDashboard.putNumber("EEReference", m_AngleMotor.getPosition().getValueAsDouble());
  }

  public double getCurrentAngle(){
    return m_AngleMotor.getPosition().getValueAsDouble()-SmartDashboard.getNumber("EEReference", 0);
  }

  public void ToFloorAngle(){
    m_AngleMotor.setPosition(EndEffectorConstants.FloorAngle-SmartDashboard.getNumber("EEReference",0));
  }

  public void ToL1Angle(){
    m_AngleMotor.setPosition(EndEffectorConstants.L1Angle-SmartDashboard.getNumber("EEReference", 0));
  }

  public void ToL2Angle(){
    m_AngleMotor.setPosition(EndEffectorConstants.L2Angle-SmartDashboard.getNumber("EEReference",0));
  }

  public void ToL4Angle(){
    m_AngleMotor.setPosition(EndEffectorConstants.L4Angle-SmartDashboard.getNumber("EEReference",0));
  }

  public void ToStationAngle(){
    m_AngleMotor.setPosition(EndEffectorConstants.StationAngle-SmartDashboard.getNumber("EEReference", 0));
  }

  public void ToStartingAngle(){
    m_AngleMotor.setPosition(EndEffectorConstants.StationAngle-SmartDashboard.getNumber("EEReference", 0));
  }


  
  /** clears animation running in given animation slot */
  public void stopAnimation(int AnimationSlot){
    m_Candle.clearAnimation(AnimationSlot);
  }

  /** @return if something is within the TOF target range
   */
  public boolean HasCoral(){
    return TimeOfFlightConstants.MaxRange > m_TOF.getRange() && m_TOF.getRange() >TimeOfFlightConstants.MinRange;
  }

  private void LEDControl(){
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