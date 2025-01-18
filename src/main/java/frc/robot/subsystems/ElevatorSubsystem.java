// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  TalonFX m_leftElevatorMotor = new TalonFX(ElevatorConstants.leftElevatatorMotorID); // lead
  TalonFX m_rightElevatorMotor = new TalonFX(ElevatorConstants.rightElevatorMotorID); // follower

 
  /** Creates a new Elevator. */
  public ElevatorSubsystem() {
    m_rightElevatorMotor.setControl(new Follower(ElevatorConstants.leftElevatatorMotorID, false));
  }

  public void GoToFloor(){
    m_leftElevatorMotor.setControl(new PositionDutyCycle(ElevatorConstants.FloorPosition - SmartDashboard.getNumber("ElevatorReference", 0)));
  }

  public void GoToL1(){
    m_leftElevatorMotor.setControl(new PositionDutyCycle(ElevatorConstants.L1Position - SmartDashboard.getNumber("ElevatorReference", 0)));
  }

  public void GoToL2(){
    m_leftElevatorMotor.setControl(new PositionDutyCycle(ElevatorConstants.L2Position - SmartDashboard.getNumber("ElevatorReference", 0)));
  }

  public void GoToL3(){
    m_leftElevatorMotor.setControl(new PositionDutyCycle(ElevatorConstants.L3Position - SmartDashboard.getNumber("ElevatorReference", 0)));
  }

  public void GoToL4(){
    m_leftElevatorMotor.setControl(new PositionDutyCycle(ElevatorConstants.L4Position -SmartDashboard.getNumber("ElevatorReference", 0)));
  }

  public void GoToStation(){
    m_leftElevatorMotor.setControl(new PositionDutyCycle(ElevatorConstants.StationPosition - SmartDashboard.getNumber("ElevatorReference", 0)));
  }


  /** Stops elevator */
  public void StopElevator(){
    m_rightElevatorMotor.set(0);
    m_leftElevatorMotor.set(0);
  }

  public double getElevatorPos(){

    return m_leftElevatorMotor.getPosition().getValueAsDouble()- SmartDashboard.getNumber("ElevatorReference", 0);
  };

  public void setPosAsReference(){

    SmartDashboard.putNumber("ElevatorReference", m_leftElevatorMotor.getPosition().getValueAsDouble());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
