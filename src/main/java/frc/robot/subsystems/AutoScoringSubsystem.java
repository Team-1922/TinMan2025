// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AprilTagAim;
import frc.robot.Commands.Collect;
import frc.robot.Commands.EEVertical;
import frc.robot.Commands.EeL2;
import frc.robot.Commands.EeL3;
import frc.robot.Commands.EeL4;
import frc.robot.Commands.GotoFloor;
import frc.robot.Commands.GotoL2;
import frc.robot.Commands.GotoL3;
import frc.robot.Commands.GotoL4;
import frc.robot.Commands.ReverseCollector;
import frc.robot.Commands.StoweEE;
import frc.robot.generated.TunerConstants;

public class AutoScoringSubsystem extends SubsystemBase {
  
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final EndEffector m_EE = new EndEffector();
  private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem();
   CommandSwerveDrivetrain m_Drivetrain;
     LimelightSubsystem m_LimelightSubsystemLeft = new LimelightSubsystem("left");
     LimelightSubsystem m_LimelightSubsystemRight = new LimelightSubsystem("right");

  int TargetLevel; // target for the elevator/EE, also known as the main reason this subsystem exists
  /** Creates a new AutoScoringSubsystem. 
   * @param side "left" to score left side, "right" to score right side 
  */
  public AutoScoringSubsystem(CommandSwerveDrivetrain drivetrain) {
    TargetLevel = 0;
    m_Drivetrain = drivetrain;
  }

  /** increments target level by 1 */
  public void incrementTarget(){
    TargetLevel=(TargetLevel+1)%3;
    PutTargetOnDashboard();
  }


/** returns the current target level
  * <p> 0 = L2
   * <p> 1 = L3
   * <p> 2 = L4
 */
  public int GetTargetLevel(){
    return TargetLevel;
  }

  public void PutTargetOnDashboard(){
    SmartDashboard.putNumber("TargetLevel", TargetLevel+2);
  }

  /** @param Target put {@code GetTargetLevel()} here
   *  @return sequential command group for scoring on L2-4, as scoring on L1 is manual
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

  /**
   * 
   * @param TargetCommandGroup use {@code GetTargetCommandGroup()}
   * @param side "left" or "right"
   * @return parralel command group that will both aim and move the EE to the position for scoring
   */
  public SequentialCommandGroup TargetAndAim(SequentialCommandGroup TargetCommandGroup,String side){
    LimelightSubsystem LL;
    if (side == "left") {
      LL = m_LimelightSubsystemLeft;
    } else {
      LL = m_LimelightSubsystemRight;
    }
    if (GetTargetLevel() == 0) {
      return
          new SequentialCommandGroup(
              TargetCommandGroup, 
              new AprilTagAim(LL, m_Drivetrain),
              new ParallelDeadlineGroup(new WaitCommand(0.25),new ReverseCollector(m_EE))//,
              // new EEVertical( m_EE),
              // new GotoFloor(m_Elevator),
              // new StoweEE(m_EE)
      );
    } else {
      return new SequentialCommandGroup(
          new ParallelCommandGroup(
              new AprilTagAim(m_LimelightSubsystemLeft, m_Drivetrain),
              TargetCommandGroup
          ),
           new ParallelDeadlineGroup(new WaitCommand(0.25),new Collect(m_EE)),
          new EEVertical( m_EE),
          new GotoFloor(m_Elevator),
          new StoweEE(m_EE)
    );
    }
  }



  @Override
  public void periodic() {
  //  PutTargetOnDashboard();
    // This method will be called once per scheduler run
  }
}
