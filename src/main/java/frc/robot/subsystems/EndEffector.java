// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.EndEffectorConstants;


public class EndEffector extends SubsystemBase {
  TalonFX m_leftCollect = new TalonFX(EndEffectorConstants.leftCollectorMotorID);
  TalonFX m_rightCollect = new TalonFX(EndEffectorConstants.rightCollectorMotorID); // change these ids later
  TalonFX m_AngleMotor = new TalonFX(EndEffectorConstants.endEffectorAngleMotorID);
  /** Creates a new EndEffector. */
  public EndEffector() {}

  public void collect(){
    m_leftCollect.setControl(new VelocityDutyCycle(EndEffectorConstants.collectorRPM));
    m_rightCollect.setControl(new VelocityDutyCycle(EndEffectorConstants.collectorRPM));
    //spin motor relly fast
  }

  public void stopCollector(){
    m_rightCollect.set(0);
    m_leftCollect.set(0);
  }


  public void ReverseCollector(){
    m_leftCollect.setControl(new VelocityDutyCycle(-EndEffectorConstants.collectorRPM));// maybe make them a seperate constant, if needed
    m_rightCollect.setControl(new VelocityDutyCycle(-EndEffectorConstants.collectorRPM));
  }

  public void setAngleAsReference(){}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}