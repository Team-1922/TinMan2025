// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Commands.EEVertical;
import frc.robot.Commands.EeL1;
import frc.robot.Commands.EeL2;
import frc.robot.Commands.EeL3;
import frc.robot.Commands.EeL4;
import frc.robot.Commands.GotoL2;
import frc.robot.Commands.GotoL3;
import frc.robot.Commands.GotoL4;
import frc.robot.generated.TunerConstants;

public class AutoScoringSubsystem extends SubsystemBase {
  
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final EndEffector m_EE = new EndEffector();
  private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem();
   CommandSwerveDrivetrain m_Drivetrain = TunerConstants.createDrivetrain();
     SwerveRequest.RobotCentric m_drive;
  int TargetLevel; // target for the elevator/EE, also known as the main reason this subsystem exists
  /** Creates a new AutoScoringSubsystem. */
  public AutoScoringSubsystem() {
    TargetLevel = 0;
  }

  /** increments target level by 1 */
  public void incrementTarget(){
    TargetLevel=(TargetLevel+1)%3;
  }



  public int GetTargetLevel(){
    return TargetLevel;
  }

  public void PutTargetOnDashboard(){
    SmartDashboard.putNumber("TargetLevel", TargetLevel+2);
  }

  /** @return
   * <ul>
   * <p> 0 = L2
   * <p> 1 = L3
   * <p> 2 = L4
   */
  public SequentialCommandGroup GetTargetCommandGroup(int Target){
    
    if(Target ==0){
   return new SequentialCommandGroup(
        new EEVertical( m_EE),    
        new GotoL2(m_Elevator),
        new EeL2(m_EE))
    ;}else if(Target == 1){
     return new SequentialCommandGroup(
        new EEVertical( m_EE),
        new GotoL3(m_Elevator),
        new EeL3(m_EE)
    );} else {

    return new SequentialCommandGroup(
        new EEVertical( m_EE),
        new GotoL4(m_Elevator),
        new EeL4(m_EE)
    );}


  }

  public ParallelCommandGroup TargetAndAim(SequentialCommandGroup TargetCommandGroup,LimelightSubsystem m_LimelightSubsystem){
    return new ParallelCommandGroup(

    m_Drivetrain.applyRequest(() ->
    m_drive.withVelocityX(m_LimelightSubsystem.RobotXDutyCycle() * LimelightConstants.MaxAimSpeed) // Drive forward with negative Y (forward)
   
    .withVelocityY(m_LimelightSubsystem.RobotYDutyCycle()* LimelightConstants.MaxAimSpeed) // Drive left with negative X (left)
        .withRotationalRate(
          
        m_LimelightSubsystem.AimTargetYawDutyCycle()
          
          * MaxAngularRate) // Drive counterclockwise with negative X (left)
)
    ,
    TargetCommandGroup
    );
  }



  @Override
  public void periodic() {
    PutTargetOnDashboard();
    // This method will be called once per scheduler run
  }
}
