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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.ClimbCommand;
import frc.robot.Commands.Collect;
import frc.robot.Commands.EEVertical;
import frc.robot.Commands.EeFloor;
import frc.robot.Commands.GoToStation;
import frc.robot.Commands.GotoFloor;
import frc.robot.Commands.GotoL1;
import frc.robot.Commands.GotoL2;
import frc.robot.Commands.GotoL3;
import frc.robot.Commands.GotoL4;
import frc.robot.Commands.IncrementTargetLocation;
import frc.robot.Commands.EeL1;
import frc.robot.Commands.EeL2;
import frc.robot.Commands.EeL3;
import frc.robot.Commands.EeL4;
import frc.robot.Commands.GoToLimelightPos;
import frc.robot.Commands.LEDtest;
import frc.robot.Commands.AprilTagAim;
import frc.robot.Commands.AutoScoreCommand;
import frc.robot.Commands.StopArm;
import frc.robot.Commands.StopElevator;
import frc.robot.Commands.StoweEE;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AutoScoringSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() 
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.RobotCentric RcDrive = new SwerveRequest.RobotCentric() 
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driveController = new CommandXboxController(0); // DRIVER CONTROLLER
    private final CommandXboxController m_operatorController = new CommandXboxController(1); // operator

    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    private final LimelightSubsystem m_LeftLimelightSubsystem = new LimelightSubsystem("left"); // left limelight subsystem
    private final LimelightSubsystem m_RightLimelightSubsystem = new LimelightSubsystem("right"); // right limelight subsystem
    private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    private final EndEffector m_EE = new EndEffector();
    private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

    private final AprilTagAim m_RightAim = new AprilTagAim(m_LeftLimelightSubsystem, m_drivetrain, m_driveController,RcDrive);
    private final AprilTagAim m_LeftAim = new AprilTagAim(m_RightLimelightSubsystem, m_drivetrain,m_driveController,RcDrive);
    private final Collect m_Collect = new Collect(m_EE);
    private final ClimbCommand m_ClimbCommand = new ClimbCommand(m_ClimberSubsystem, m_operatorController);
   

    private final AutoScoringSubsystem m_AutoScoringSubsystem = new AutoScoringSubsystem();
    private final AutoScoreCommand m_AutoScoreCommand = new AutoScoreCommand(m_AutoScoringSubsystem, m_LeftLimelightSubsystem);
    private final IncrementTargetLocation m_IncrementTargetLocation = new IncrementTargetLocation(m_AutoScoringSubsystem);
    //elevator commands
    
    private final GotoFloor m_GotoFloor = new GotoFloor(m_ElevatorSubsystem);
    private final GotoL1 m_ElevatorL1 = new GotoL1(m_ElevatorSubsystem);
    private final GotoL2 m_ElevatorL2 = new GotoL2(m_ElevatorSubsystem);
    private final GotoL3 m_ElevatorL3 = new GotoL3(m_ElevatorSubsystem);
    private final GotoL4 m_ElevatorL4 = new GotoL4(m_ElevatorSubsystem);
    private final GoToStation m_GoToStation = new GoToStation(m_ElevatorSubsystem);
    private final StopElevator m_StopElevator = new StopElevator(m_ElevatorSubsystem);
    private final GoToLimelightPos m_ElevatorLL = new GoToLimelightPos(m_ElevatorSubsystem);
    private final LEDtest m_LeDtest = new LEDtest(m_EE);


    // EE commands

 

    private final StopArm m_StopArm = new StopArm(m_EE);

    private final SendableChooser<Command> autoChooser;



    // EE+elevator commands


    private final EeL1 m_L1 = new EeL1(m_EE);
    private final EeL2 m_L2 = new EeL2( m_EE);
    private final EeL3 m_L3 = new EeL3(m_EE);
    private final EeL4 m_L4 = new EeL4( m_EE);
    private final EeFloor m_Floor = new EeFloor( m_EE);
    private final StoweEE m_StoweEE = new StoweEE(m_ElevatorSubsystem, m_EE);
    private final EEVertical m_EeVertical = new EEVertical( m_EE);


    // sequential command groups
    private final SequentialCommandGroup m_L1Group = new SequentialCommandGroup(
        new EEVertical( m_EE),
        new GotoL1(m_ElevatorSubsystem),
        new EeL1(m_EE)
    );

    private final SequentialCommandGroup m_L2Group = new SequentialCommandGroup(
        new EEVertical( m_EE),    
        new GotoL2(m_ElevatorSubsystem),
        new EeL2(m_EE)
    );
    private final SequentialCommandGroup m_L3Group = new SequentialCommandGroup(
        new EEVertical( m_EE),
        new GotoL3(m_ElevatorSubsystem),
        new EeL3(m_EE)
    );

    private final SequentialCommandGroup m_L4Group = new SequentialCommandGroup(
        new EEVertical( m_EE),
        new GotoL4(m_ElevatorSubsystem),
        new EeL4(m_EE)
    );

    private final SequentialCommandGroup m_FloorGroup = new SequentialCommandGroup(
        new EEVertical( m_EE),
        new GotoFloor(m_ElevatorSubsystem),
        new EeFloor(m_EE)
    );

    private final SequentialCommandGroup m_AimGroup = new SequentialCommandGroup(

        new EEVertical( m_EE),
        new GoToLimelightPos(m_ElevatorSubsystem)
    );





    public RobotContainer() {
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric());


    NamedCommands.registerCommand("Collect", m_Collect); // put pathplanner commands here
    NamedCommands.registerCommand("GoL4", m_L4Group);

    
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
 
 

    autoChooser = AutoBuilder.buildAutoChooser("1 piece center");
    SmartDashboard.putData("autoChooser", autoChooser);
    }

    private void configureBindings() {
        m_ClimberSubsystem.setDefaultCommand(m_ClimbCommand);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand(
            // m_drivetrain will execute this command periodically
            m_drivetrain.applyRequest(() ->
                drive.withVelocityX(-MathUtil.applyDeadband(m_driveController.getLeftY(),0.15) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-MathUtil.applyDeadband(m_driveController.getLeftX(),0.15) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-MathUtil.applyDeadband(m_driveController.getRightX(),0.15) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        m_driveController.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
        m_driveController.b().whileTrue(m_drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driveController.getLeftY(), -m_driveController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driveController.back().and(m_driveController.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        m_driveController.back().and(m_driveController.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        m_driveController.start().and(m_driveController.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driveController.start().and(m_driveController.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on Y press
        m_driveController.y().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

        m_drivetrain.registerTelemetry(logger::telemeterize);

        m_driveController.button(5).whileTrue(m_LeftAim); // left bumper
      m_driveController.button(6).whileTrue(m_RightAim); // right bumper
        
     //  m_driveController.pov(180).onTrue(m_AutoScoreCommand);
        m_operatorController.button(1).onTrue(m_L1Group); // a
        m_operatorController.button(2).onTrue(m_L2Group); // b
       // m_operatorController.button(2).onTrue(m_ElevatorL2);

       m_driveController.pov(0).onTrue(m_AimGroup);
        m_operatorController.pov(180).onTrue(m_L3Group);
        m_operatorController.pov(90).onTrue(m_L4Group);
        m_operatorController.pov(270).onTrue(m_FloorGroup);
    //    m_operatorController.pov(90).onTrue(m_IncrementTargetLocation);
        m_operatorController.button(3).onTrue(m_StoweEE); // x 
        m_operatorController.button(4).onTrue(m_EeVertical); // y 
        m_operatorController.button(4).onTrue(m_L1); // y 
        m_operatorController.button(5).onTrue(m_L3);
        
   //     m_operatorController.button(5).onTrue(m_Floor); // left bumper
   //     m_operatorController.button(3).onTrue(m_AngleL4); // x 
        //m_operatorController.button(8).onTrue(m_StopArm); // 3 lines button
      
        m_operatorController.button(6).whileTrue(m_Collect); // RB
//        m_operatorController.button(5).onTrue();
      //  m_operatorController.button()

       m_operatorController.button(10).onTrue(m_GotoFloor);
    //    m_operatorController.button(7).onTrue(m_ElevatorL2);
   //     m_operatorController.button(8).onTrue(m_ElevatorL3);
     //   m_operatorController.button(4).onTrue(m_ElevatorL4);
//m_operatorController.button(7).onTrue(m_ElevatorReference);
      // m_operatorController.button(8).onTrue(m_StopElevator); // 3 lines button
   //     m_operatorController.button(5).onTrue(m_LeDtest);
    }

    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
    }
}
