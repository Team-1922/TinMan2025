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
import frc.robot.Commands.CloseToReef;
import frc.robot.Commands.Collect;
import frc.robot.Commands.MoveArm;
import frc.robot.Commands.MoveArmAndWrist;
import frc.robot.Commands.MoveElevator;
import frc.robot.Commands.MoveWrist;
import frc.robot.Commands.ReverseCollector;
import frc.robot.Constants.*;

import frc.robot.generated.TunerConstants;

public class AutoScoringSubsystem extends SubsystemBase {
  
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final EndEffector m_EE = new EndEffector();
  private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem();
   CommandSwerveDrivetrain m_Drivetrain;
   public  LimelightSubsystem m_LimelightSubsystemLeft = new LimelightSubsystem("right");
   public  LimelightSubsystem m_LimelightSubsystemRight = new LimelightSubsystem("left");
  
  
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

   return new SequentialCommandGroup(// L2
    new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
    new MoveElevator(m_Elevator, ElevatorConstants.L2Position),
    new MoveArmAndWrist(m_EE, EndEffectorConstants.L2ArmAngle, EndEffectorConstants.L2WristAngle))
    ;}else if(Target == 1){

     return new SequentialCommandGroup( // L3
        new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
        new MoveElevator(m_Elevator, ElevatorConstants.L3Position),
        new MoveArmAndWrist(m_EE, EndEffectorConstants.L3ArmAngle, EndEffectorConstants.L3WristAngle) 
    );} else {

    return new SequentialCommandGroup(// L4
      new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
      new MoveElevator(m_Elevator, ElevatorConstants.L4Position),
      new MoveArmAndWrist(m_EE, EndEffectorConstants.L4ArmAngle, EndEffectorConstants.L4WristAngle)
    );}
  }

/*   public SequentialCommandGroup GetAlgaeTargetCommandGroup(int Target){
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

  }*/



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
              new ParallelRaceGroup( new AprilTagAim(LL, m_Drivetrain), new WaitCommand(3.5)),
               new ParallelRaceGroup(new WaitCommand(0.75),new ReverseCollector(m_EE)),
               new AprilTagAimReverse(LL, m_Drivetrain),
               new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
                new MoveElevator(m_Elevator, ElevatorConstants.FloorPosition),
                new MoveArmAndWrist(m_EE, EndEffectorConstants.StowedArmAngle, EndEffectorConstants.StowedWristAngle)
      );
    } else {
      return new SequentialCommandGroup( // L3 and L4
          new ParallelCommandGroup(
             new ParallelRaceGroup( new AprilTagAim(LL, m_Drivetrain),new WaitCommand(3.5)),
              TargetCommandGroup
          ),
          new ParallelRaceGroup(
            new CloseToReef(LL), // checks if the robot is close enough to the reef to score 
            new SequentialCommandGroup( 
              new WaitCommand(0.25),
              new ParallelRaceGroup(new WaitCommand(0.65),
              new Collect(m_EE,-0.4)))),

           new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
           new MoveElevator(m_Elevator, ElevatorConstants.FloorPosition),
           new MoveArmAndWrist(m_EE, EndEffectorConstants.StowedArmAngle, EndEffectorConstants.StowedWristAngle)
    );
    }
  }

  

 /* public SequentialCommandGroup StationPickupGroup(){
    return new SequentialCommandGroup(

      new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
      new MoveElevator(m_Elevator, ElevatorConstants.FloorPosition),
      new MoveWrist(m_EE, EndEffectorConstants.L3WristAngle),// l3 angle_EE)
   
      new ParallelCommandGroup(  
        new MoveElevator(m_Elevator, ElevatorConstants.StationPosition),
        new MoveArm(m_EE, EndEffectorConstants.StationArmAngle)),
      new MoveWrist(m_EE, EndEffectorConstants.StationWristAngle),
      new Collect(m_EE, -0.2),
      //new WaitCommand(5), // to act as collect for now
      new MoveWrist(m_EE, EndEffectorConstants.L3WristAngle),
      new ParallelCommandGroup(
        new MoveArm(m_EE, EndEffectorConstants.StowedArmAngle),
        new MoveElevator(m_Elevator, ElevatorConstants.FloorPosition)),
      new MoveWrist(m_EE, EndEffectorConstants.StowedWristAngle)
      // send arm and wrist back , not all the way to the position?
      // send elevator up
      // move arm and wrist to position
      // collect
      // move arm and wrist up a little so they don't hit climber/elevator
      // move elevator down
      // move arm and wrist back into vertical position
    );
  }
*/

  @Override
  public void periodic() {
  //  PutTargetOnDashboard();
    // This method will be called once per scheduler run
  }
}
