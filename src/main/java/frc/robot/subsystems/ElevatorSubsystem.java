// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  TalonFX m_leftElevatorMotor = new TalonFX(ElevatorConstants.leftElevatatorMotorID);
  TalonFX m_rightElevatorMotor = new TalonFX(ElevatorConstants.rightElevatorMotorID);


  /** Creates a new Elevator. */
  public ElevatorSubsystem() {}

  public void GoToFloor(){}

  public void GoToL1(){}

  public void GoToL2(){}

  public void GoToL3(){}

  public void GoToL4(){}

  public void StopElevator(){
    m_rightElevatorMotor.set(0);
    m_leftElevatorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
