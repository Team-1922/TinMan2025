// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.AprilTagAim;
import frc.robot.Commands.AprilTagAimReverse;
import frc.robot.Commands.Collect;
import frc.robot.Commands.EEVertical;
import frc.robot.Commands.EeAlgae;
import frc.robot.Commands.EeL2;
import frc.robot.Commands.EeL3;
import frc.robot.Commands.EeL4;
import frc.robot.Commands.GotoFloor;
import frc.robot.Commands.GotoL2;
import frc.robot.Commands.GotoL3;
import frc.robot.Commands.GotoL4;
import frc.robot.Commands.GotoLowerAlgae;
import frc.robot.Commands.GotoUpperAlgae;
import frc.robot.Commands.ReverseCollector;
import frc.robot.Commands.StoweEE;
import frc.robot.generated.TunerConstants;

public class AutoScoringSubsystem extends SubsystemBase {
  
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final EndEffector m_EE = new EndEffector();
  private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem();
   CommandSwerveDrivetrain m_Drivetrain;
     LimelightSubsystem m_LimelightSubsystemLeft = new LimelightSubsystem("right");
     LimelightSubsystem m_LimelightSubsystemRight = new LimelightSubsystem("left");

  int TargetLevel; // target for the elevator/EE, also known as the main reason this subsystem exists
  /** Creates a new AutoScoringSubsystem. */
  public AutoScoringSubsystem(CommandSwerveDrivetrain drivetrain) {
    TargetLevel = 2;
    m_Drivetrain = drivetrain;
  }

  /** increments target level by 1 
  */
  public void incrementTarget(){
    TargetLevel=((TargetLevel+1)%3); // adding 2 would not work because to get from L2 to L3 it would go, (0+1)%3, which = 0, then add 2, so it would be stuck at 2 
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

  public SequentialCommandGroup GetAlgaeTargetCommandGroup(int Target){
    if (Target == 1){ // between L2/3
     return new SequentialCommandGroup( // don't forget to get the target numbers before testing this :)
        new EEVertical(m_EE),
        new GotoLowerAlgae(m_Elevator),
        new EeAlgae(m_EE)
    );} else {// between L3/4
       return new SequentialCommandGroup(
       new EEVertical(m_EE),
      new GotoUpperAlgae(m_Elevator),
      new EeAlgae(m_EE)
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
      return //new SequentialCommandGroup(new WaitCommand(0));
             new SequentialCommandGroup( // L2
               TargetCommandGroup, 
               new AprilTagAim(LL, m_Drivetrain),
               new ParallelRaceGroup(new WaitCommand(0.75),new ReverseCollector(m_EE)),
               new AprilTagAimReverse(LL, m_Drivetrain),
                new EEVertical( m_EE),
                new GotoFloor(m_Elevator),
                new StoweEE(m_EE)
      );
    } else {
      return new SequentialCommandGroup( // L3 and L4
          new ParallelCommandGroup(
              new AprilTagAim(LL, m_Drivetrain),
              TargetCommandGroup
          ),
          new WaitCommand(0.15),
           new ParallelRaceGroup(new WaitCommand(1.5),
           new Collect(m_EE)),
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
