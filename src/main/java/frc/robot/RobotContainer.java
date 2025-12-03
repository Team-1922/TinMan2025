// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import edu.wpi.first.math.MathUtil;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Collect;
import frc.robot.Commands.IncrementTargetLocation;
import frc.robot.Commands.MoveArm;
import frc.robot.Commands.MoveArmAndWrist;
import frc.robot.Commands.MoveElevator;
import frc.robot.Commands.MoveWrist;
import frc.robot.Commands.ReverseCollector;
import frc.robot.Commands.StationCollect;
import frc.robot.Commands.StopElevatorAndEE;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AutoScoringSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants.*;
import frc.robot.Commands.HoldCoral;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Commands.DriveCommand;

public class RobotContainer {
    private Pigeon2 m_Pigeon2 = new Pigeon2(0, "Drivebase");
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    // was 0.75
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() 
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric RcDrive = new SwerveRequest.RobotCentric() 
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController m_driveController = new CommandXboxController(0); // DRIVER CONTROLLER
    private final CommandXboxController m_operatorController = new CommandXboxController(1); // operator
    final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    final EndEffector m_EE = new EndEffector();
    private final Collect m_FloorCollect = new Collect(m_EE,-0.4);
    private final Collect m_L1Shoot = new Collect(m_EE,-0.2);
    private final ReverseCollector m_ReverseCollector = new ReverseCollector(m_EE);
    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    private final LimelightSubsystem m_LimelightSubsystemLeft = new LimelightSubsystem("left");
    private final LimelightSubsystem m_LimelightSubsystemRight = new LimelightSubsystem("right");
    private final AutoScoringSubsystem m_AutoScoringSubsystem = new AutoScoringSubsystem(m_drivetrain, m_LimelightSubsystemLeft, m_LimelightSubsystemRight);
    private final Command m_RightL4AutoScoreForAuto = m_AutoScoringSubsystem.TargetAndAim("right");
    private final Command m_LeftL4AutoScoreForAuto = m_AutoScoringSubsystem.TargetAndAim("left");
    private final Command m_LeftL3AutoScoreForAuto = m_AutoScoringSubsystem.TargetAndAim("left", 1);
    private final Command m_RightL3AutoScoreForAuto = m_AutoScoringSubsystem.TargetAndAim("right", 1);
    private final IncrementTargetLocation m_IncrementTargetLocation = new IncrementTargetLocation(m_AutoScoringSubsystem);
    private final StationCollect m_StationCollect = new StationCollect(m_EE, 0.075);
    final LedSubsystem m_LED = new LedSubsystem(m_EE, m_AutoScoringSubsystem, m_LimelightSubsystemLeft, m_LimelightSubsystemRight);
    // elavator commands
    private final StopElevatorAndEE m_StopElevatorAndEE = new StopElevatorAndEE(m_EE, m_ElevatorSubsystem);
    private final HoldCoral m_holdCoral = new HoldCoral(m_EE);

    // EE commands
    public final DriveCommand m_DriveCommand = new DriveCommand(m_drivetrain, m_driveController, m_Pigeon2, m_LimelightSubsystemLeft);
    private final SendableChooser<Command> autoChooser;

    // EE+elevator commands
    private final MoveArmAndWrist m_StoweEE =  new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle);
    
    // sequential command groups for the elevator/EE, used for testing.
    private final SequentialCommandGroup m_L1Group = new SequentialCommandGroup(
        new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.L1Position),
        new MoveArmAndWrist(m_EE, EndEffectorConstants.L1ArmAngle, EndEffectorConstants.L1WristAngle)
    );

    private final SequentialCommandGroup m_L2Group = new SequentialCommandGroup(
        new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
        new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.L2Position),
        new MoveArmAndWrist(m_EE, EndEffectorConstants.L2ArmAngle, EndEffectorConstants.L2WristAngle)
    );
    private final SequentialCommandGroup m_L3Group = new SequentialCommandGroup(
        new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
        new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.L3Position),
        new MoveArmAndWrist(m_EE, EndEffectorConstants.L3ArmAngle, EndEffectorConstants.L3WristAngle)
    );

    private final SequentialCommandGroup m_L4Group = new SequentialCommandGroup(
        new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
        new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.L4Position),
        new MoveArmAndWrist(m_EE, EndEffectorConstants.L4ArmAngle, EndEffectorConstants.L4WristAngle)
    );

    private final SequentialCommandGroup m_FloorGroup = new SequentialCommandGroup(
        new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
        new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.FloorPosition),
        new MoveArmAndWrist(m_EE, EndEffectorConstants.FloorCollectArmAngle, EndEffectorConstants.ZeroWristAngle),
        new MoveWrist(m_EE, EndEffectorConstants.FloorCollectWristAngle)
    );


    private final SequentialCommandGroup m_CORALSTUCKgroup = new SequentialCommandGroup(// sends elevator up, for if a coral is stuck
        new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
        new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.CoralStuckPosition)
    );

    private final SequentialCommandGroup m_verticalStowGroup = new SequentialCommandGroup(
        new MoveArm(m_EE, EndEffectorConstants.VerticalArmAngle),
        new MoveWrist(m_EE, EndEffectorConstants.VerticalWristAngle),
        new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.FloorPosition)
    );

    private final SequentialCommandGroup m_AutoL4Group = new SequentialCommandGroup(
        new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
        new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.L4Position)
    );

    private final SequentialCommandGroup m_stationCollect = new SequentialCommandGroup(
        new MoveArmAndWrist(m_EE, EndEffectorConstants.StationArmAngle, EndEffectorConstants.StationWristAngle),
        m_StationCollect
    );

    /** if the arm is stuck at the station position from letting go of the button, this should send it back */
    public final SequentialCommandGroup m_backFromStation = new SequentialCommandGroup(

    new MoveWrist(m_EE,EndEffectorConstants.L3WristAngle),
    new ParallelCommandGroup(
        new MoveElevator(m_ElevatorSubsystem,ElevatorConstants.FloorPosition),
        new MoveArm(m_EE, EndEffectorConstants.VerticalArmAngle)
     ),
     new MoveArmAndWrist(m_EE,EndEffectorConstants.VerticalArmAngle,EndEffectorConstants.VerticalWristAngle)
    );

    public final SequentialCommandGroup m_algaeRemove = new SequentialCommandGroup(
        new MoveArmAndWrist(m_EE, EndEffectorConstants.AlgaeArmAngle, EndEffectorConstants.VerticalWristAngle),
        new MoveArmAndWrist(m_EE, EndEffectorConstants.L4ArmAngle, EndEffectorConstants.L4WristAngle),
        new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.L3Position+9)
    );

    public final SequentialCommandGroup m_L2algaeRemove = new SequentialCommandGroup(
        new MoveArmAndWrist(m_EE, EndEffectorConstants.AlgaeArmAngle, EndEffectorConstants.VerticalWristAngle),
        new MoveArmAndWrist(m_EE, EndEffectorConstants.L4ArmAngle, EndEffectorConstants.L4WristAngle),
        new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.AlgaeL2)
    );

    public RobotContainer() {
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true); // uncomment this when testing with only 1 controller, this turns off the joystick unplugged warning
    m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric());

    NamedCommands.registerCommand("VerticalStow", new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle));
    NamedCommands.registerCommand("Collect", m_FloorCollect); // put pathplanner commands here
    NamedCommands.registerCommand("LeftL4", m_LeftL4AutoScoreForAuto);
    NamedCommands.registerCommand("RightL4", m_RightL4AutoScoreForAuto);
    NamedCommands.registerCommand("LeftL3", m_LeftL3AutoScoreForAuto);
    NamedCommands.registerCommand("RightL3", m_RightL3AutoScoreForAuto);
    NamedCommands.registerCommand("AimPrep", m_AutoL4Group); // Re-get these numbers and test this before adding into autos
    NamedCommands.registerCommand("StationCollect", m_stationCollect);
    // the try catch loop makes the code not error, all this is doing is loading the paths into pathplanner
    try {
        PathPlannerPath TestPath = PathPlannerPath.fromChoreoTrajectory("choreoTest");
    } catch (FileVersionException e) {
        e.printStackTrace();
    } catch (IOException e) {
        e.printStackTrace();
    } catch (ParseException e) {
        e.printStackTrace();
    }

    autoChooser = AutoBuilder.buildAutoChooser("1 Piece Center Right");
    SmartDashboard.putData("autoChooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand(
            new SequentialCommandGroup(
                m_drivetrain.applyRequest(() -> drive
                    .withVelocityX(-MathUtil.applyDeadband(m_driveController.getLeftY(),0.15) * (MaxSpeed*.4)) // Drive forward with negative Y (forward)
                    .withVelocityY(-MathUtil.applyDeadband(m_driveController.getLeftX(),0.15) * (MaxSpeed*.4)) // Drive left with negative X (left)
                    .withRotationalRate(-MathUtil.applyDeadband(m_driveController.getRightX(),0.15) * (MaxAngularRate*.8)) // Drive counterclockwise with negative X (left)
                    ),
                    m_holdCoral

            )
        );

        m_drivetrain.registerTelemetry(logger::telemeterize); // commented out to reduce RIO CPU usage 


       // DRIVER CONTROLS
       
        // reset the field-centric heading on Y press
        m_driveController.y().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));//.andThen(new NeedYawOffsetTrue()));

        m_driveController.button(6).and(() -> m_AutoScoringSubsystem.GetTargetLevel() == 2).whileTrue(m_AutoScoringSubsystem.TargetAndAim( "right", 2)); // Right Bumper 

        m_driveController.button(6).and(() -> m_AutoScoringSubsystem.GetTargetLevel() == 1).whileTrue(m_AutoScoringSubsystem.TargetAndAim("right", 1)); // Right Bumper
                
        m_driveController.button(6).and(() -> m_AutoScoringSubsystem.GetTargetLevel() == 0).whileTrue(m_AutoScoringSubsystem.TargetAndAim("right", 0)); // Right Bumper 

        m_driveController.button(5).and(() -> m_AutoScoringSubsystem.GetTargetLevel() == 2).whileTrue(m_AutoScoringSubsystem.TargetAndAim("left", 2)); // left Bumper 
    
        m_driveController.button(5).and(() -> m_AutoScoringSubsystem.GetTargetLevel() == 1).whileTrue(m_AutoScoringSubsystem.TargetAndAim("left", 1)); // left Bumper
                    
        m_driveController.button(5).and(() -> m_AutoScoringSubsystem.GetTargetLevel() == 0).whileTrue(m_AutoScoringSubsystem.TargetAndAim("left", 0)); // left Bumper 
          
   
        m_driveController.leftTrigger().whileTrue(m_FloorCollect); // Left Trigger
        m_driveController.rightTrigger().whileTrue(m_ReverseCollector); // right trigger 
        m_driveController.button(2).onTrue(m_L1Group); //B
        m_driveController.button(3).whileTrue(m_L1Shoot); // X
        

        // OPERATOR CONTROLS

        m_operatorController.button(5).onTrue(m_IncrementTargetLocation); // Left Bumper
        m_operatorController.button(6).onTrue(m_L1Group); // Right Bumper
        m_operatorController.button(1).onTrue(m_FloorGroup); // A
        m_operatorController.button(3).onTrue(m_StoweEE); // X
        m_operatorController.button(2).onTrue(m_StopElevatorAndEE);// B, the motors are not in brake mode, so the end effector might fall down if you do this before climbing. 
        m_operatorController.pov(180).onTrue(m_L4Group); // manual L4 just incase LL fails 
        m_operatorController.rightTrigger().whileTrue(m_stationCollect);// station pickup, hold the whole time
        m_operatorController.leftTrigger().whileTrue(m_backFromStation); // incase we get stuck at the station position 
        m_operatorController.pov(0).onTrue(m_CORALSTUCKgroup);// incase coral gets stuck or elevator gets stuck
        m_operatorController.pov(270).onTrue(m_L2algaeRemove);
        m_operatorController.pov(90).onTrue(m_algaeRemove);
        m_operatorController.button(10).onTrue(m_verticalStowGroup);
            //m_L3Group); 
        /*
     DRIVER
        drive  -  both joysticks
        zero field orientation - Y
        Auto Aim Left  - LB
        Auto Aim Right - RB
        Collect - LT
        reverse collector - RT



      OPERATOR
         chose scoring target   - LB
         end effector to L1 - Yu3e                                          
         move arm to Collect position/the floor - A
         move arm to Vertical position (for defence/moving around) - X
         stop elevator/arm/collector   - B
         collect from station - RT
         move arm back from station (incase it gets stuck there) - LT
    */


    }

    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
    }
}
