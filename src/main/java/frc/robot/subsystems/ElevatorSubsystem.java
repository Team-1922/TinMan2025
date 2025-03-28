// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ElevatorSubsystem extends SubsystemBase {
  TalonFX m_RightElevatorMotor = new TalonFX(ElevatorConstants.rightElevatorMotorID, "Elevator"); // lead
  TalonFX m_LeftElevatorMotor = new TalonFX(ElevatorConstants.leftElevatatorMotorID,"Elevator"); // follower
  
  // logging values
  NetworkTableInstance m_networkTable = NetworkTableInstance.getDefault();
 // DoublePublisher m_elevatorTarget = m_networkTable.getDoubleTopic("ElevatorTarget").publish();
  DoublePublisher m_elevatorPos = m_networkTable.getDoubleTopic("ElevatorPos").publish();

  private double m_ElevatorOffset;
  /** Creates a new Elevator. */
  public ElevatorSubsystem() {
   
    m_LeftElevatorMotor.getConfigurator().apply(ElevatorConstants.ElevatorCurrentLimitConfigs);
    m_RightElevatorMotor.getConfigurator().apply(ElevatorConstants.ElevatorCurrentLimitConfigs);
    m_LeftElevatorMotor.getConfigurator().apply(ElevatorConstants.ElevatorMotionMagicConfigs);
    m_RightElevatorMotor.getConfigurator().apply(ElevatorConstants.ElevatorMotionMagicConfigs);
    m_LeftElevatorMotor.getConfigurator().apply(ElevatorConstants.ElevatorSlot0Configs);
    m_RightElevatorMotor.getConfigurator().apply(ElevatorConstants.ElevatorSlot0Configs);
    m_LeftElevatorMotor.getConfigurator().apply(ElevatorConstants.ElevatorMotorOutputConfigs);
    m_RightElevatorMotor.getConfigurator().apply(ElevatorConstants.ElevatorMotorOutputConfigs);

    m_LeftElevatorMotor.setControl(new Follower(ElevatorConstants.rightElevatorMotorID, true));

    m_LeftElevatorMotor.getConfigurator().apply(ElevatorConstants.ElevatorClosedLoopRampConfigs);
    m_RightElevatorMotor.getConfigurator().apply(ElevatorConstants.ElevatorClosedLoopRampConfigs);
   // m_LeftElevatorMotor.getConfigurator().apply(EndEffectorConstants.openLoopRampConfigs);
  //  m_RightElevatorMotor.getConfigurator().apply(EndEffectorConstants.openLoopRampConfigs);
  

    m_RightElevatorMotor.setPosition(ElevatorConstants.FloorPosition);
  }

  /** position relitive to reference
   * @return the position the elevator should target (in rotations)
    */
  public double TargetPosition(double pos){
    return pos;
  };


  public void GoToPosition(double TargetPos){
    m_RightElevatorMotor.setControl(new MotionMagicExpoDutyCycle(TargetPos));
    SmartDashboard.putNumber("ETarget", TargetPos);
  }

  public void GoToFloor(){
    m_RightElevatorMotor.setControl(new MotionMagicExpoDutyCycle(TargetPosition(ElevatorConstants.FloorPosition)));
  SmartDashboard.putNumber("ETarget", TargetPosition(ElevatorConstants.FloorPosition)); 
  }

  public void GoToL1(){
  //  m_RightElevatorMotor.setControl(new MotionMagicDutyCycle(TargetPosition(ElevatorConstants.L1Position)));
    m_RightElevatorMotor.setControl(new MotionMagicExpoDutyCycle(TargetPosition(ElevatorConstants.L1Position)));
   SmartDashboard.putNumber("ETarget", TargetPosition(ElevatorConstants.L1Position));
  }

  public void GoToL2(){
    m_RightElevatorMotor.setControl(new MotionMagicExpoDutyCycle(TargetPosition(ElevatorConstants.L2Position)));
    SmartDashboard.putNumber("ETarget", TargetPosition(ElevatorConstants.L2Position));
  }

  public void GoToL3(){
     m_RightElevatorMotor.setControl(new MotionMagicExpoDutyCycle(TargetPosition(ElevatorConstants.L3Position)));
      SmartDashboard.putNumber("ETarget", TargetPosition(ElevatorConstants.L3Position));
    }

  public void GoToL4(){
    SmartDashboard.putNumber("ETarget", TargetPosition(ElevatorConstants.L4Position));
    m_RightElevatorMotor.setControl(new MotionMagicExpoDutyCycle(TargetPosition(ElevatorConstants.L4Position)));
  }
  public void GoToStation(){
  m_RightElevatorMotor.setControl(new MotionMagicExpoDutyCycle(TargetPosition(ElevatorConstants.StationPosition)));
    SmartDashboard.putNumber("ETarget", TargetPosition(ElevatorConstants.StationPosition));
  }

  public void GoToLowAlgae(){
    m_RightElevatorMotor.setControl(new MotionMagicExpoDutyCycle(TargetPosition(ElevatorConstants.AlgaeLowPosition)));
    SmartDashboard.putNumber("ETarget", TargetPosition(ElevatorConstants.AlgaeLowPosition));
  }

  public void GoToHighAlgae(){

    m_RightElevatorMotor.setControl(new MotionMagicExpoDutyCycle(TargetPosition(ElevatorConstants.AlgaeHighPosition)));
    SmartDashboard.putNumber("ETarget", TargetPosition(ElevatorConstants.AlgaeHighPosition));
    
  }

  /** moves elevator up to a position so the limelights can see the apriltags */
  public void GoToAimingPosition(){

    m_RightElevatorMotor.setControl(new MotionMagicExpoDutyCycle(TargetPosition(ElevatorConstants.LlAimPosition)));
    SmartDashboard.putNumber("ETarget", TargetPosition(ElevatorConstants.LlAimPosition));
  }



  /** Stops elevator */
  public void StopElevator(){
  //  m_rightElevatorMotor.set(0);
    m_RightElevatorMotor.set(0);
  }

  public double getElevatorPos(){
    double Position = m_RightElevatorMotor.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("ElevatorPosition", Position);
    return Position;
  };

  public void setPosAsReference(){
    SmartDashboard.putNumber("ElevatorReference", m_RightElevatorMotor.getPosition().getValueAsDouble());

  }


  public void ElevatorLogging(){
   // m_elevatorPos = getElevatorPos();
    SignalLogger.writeDouble("ElevatorPosition", getElevatorPos());
    //m_elevatorTarget.set(getElevatorPos());
  };

  @Override
  public void periodic() {
  //  ElevatorLogging();
  getElevatorPos();
    // This method will be called once per scheduler run
  }
}
