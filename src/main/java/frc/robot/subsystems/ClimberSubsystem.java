// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  SparkMax m_climberMotor = new SparkMax(ClimberConstants.climberMotorID, MotorType.kBrushless);
 // m_climberMotor = new ;
  /** Creates a new Climber. */
  public ClimberSubsystem() {
  }

  /**   @param directionMultiplier Put a joystick axis here*/
  public void climb(double directionMultiplier){
    m_climberMotor.setVoltage(ClimberConstants.ClimberMaxVoltage*directionMultiplier);
  }


  public void stopClimber(){
    m_climberMotor.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
