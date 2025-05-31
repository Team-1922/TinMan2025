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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Collect;
import frc.robot.Commands.IncrementTargetLocation;
import frc.robot.Commands.MoveArm;
import frc.robot.Commands.MoveArmAndWrist;
import frc.robot.Commands.MoveElevator;
import frc.robot.Commands.MoveWrist;
import frc.robot.Commands.ReverseCollector;
import frc.robot.Commands.StationCollect;
import frc.robot.Commands.AutoScoreCommandFORAUTO;
import frc.robot.Commands.StopArm;
import frc.robot.Commands.StopElevator;
import frc.robot.Commands.StopElevatorAndEE;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AutoScoringSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.Constants.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    // was 0.75
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.RobotCentric RcDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driveController = new CommandXboxController(0); // DRIVER CONTROLLER
    private final CommandXboxController m_operatorController = new CommandXboxController(1); // operator
    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    final EndEffector m_EE = new EndEffector();

    private final Collect m_FloorCollect = new Collect(m_EE, -0.4);
    // private final Collect m_StationCollect = new Collect(m_EE,-0.2);
    private final ReverseCollector m_ReverseCollector = new ReverseCollector(m_EE);

    private final AutoScoringSubsystem m_AutoScoringSubsystem = new AutoScoringSubsystem(m_drivetrain);
    private final AutoScoreCommandFORAUTO m_RightAutoScoreForAuto = new AutoScoreCommandFORAUTO(m_AutoScoringSubsystem,
            m_ElevatorSubsystem, m_EE, "right");
    private final AutoScoreCommandFORAUTO m_LeftAutoScoreForAuto = new AutoScoreCommandFORAUTO(m_AutoScoringSubsystem,
            m_ElevatorSubsystem, m_EE, "left");
    private final IncrementTargetLocation m_IncrementTargetLocation = new IncrementTargetLocation(
            m_AutoScoringSubsystem);
    private final StationCollect m_StationCollect = new StationCollect(m_EE, -0.2);
    // elevator commands
    private final StopElevator m_StopElevator = new StopElevator(m_ElevatorSubsystem);
    private final StopElevatorAndEE m_StopElevatorAndEE = new StopElevatorAndEE(m_EE, m_ElevatorSubsystem);

    final LedSubsystem m_LED = new LedSubsystem(m_EE, m_AutoScoringSubsystem,
            m_AutoScoringSubsystem.m_LimelightSubsystemLeft, m_AutoScoringSubsystem.m_LimelightSubsystemRight);
    // EE commands

    private final StopArm m_StopArm = new StopArm(m_EE);

    private final SendableChooser<Command> autoChooser;

    // EE+elevator commands

    // private final EeL1 m_L1 = new EeL1(m_EE);
    // private final EeL2 m_L2 = new EeL2( m_EE);
    // private final EeL3 m_L3 = new EeL3(m_EE);
    // private final EeL4 m_L4 = new EeL4( m_EE);
    // private final EeFloor m_Floor = new EeFloor( m_EE);
    private final MoveArmAndWrist m_StoweEE = new MoveArmAndWrist(m_EE, EndEffectorConstants.StowedArmAngle,
            EndEffectorConstants.StowedWristAngle);
    private final MoveArmAndWrist m_EeVertical = new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle,
            EndEffectorConstants.VerticalWristAngle);

    // sequential command groups for the elevator/EE, used for testing.
    private final SequentialCommandGroup m_L1Group = new SequentialCommandGroup(
            new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
            new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.L1Position),
            new MoveArmAndWrist(m_EE, EndEffectorConstants.L1ArmAngle, EndEffectorConstants.L1WristAngle));

    private final SequentialCommandGroup m_L2Group = new SequentialCommandGroup(
            new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
            new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.L2Position),
            new MoveArmAndWrist(m_EE, EndEffectorConstants.L2ArmAngle, EndEffectorConstants.L2WristAngle));
    private final SequentialCommandGroup m_L3Group = new SequentialCommandGroup(
            new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
            new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.L3Position),
            new MoveArmAndWrist(m_EE, EndEffectorConstants.L3ArmAngle, EndEffectorConstants.L3WristAngle));

    private final SequentialCommandGroup m_L4Group = new SequentialCommandGroup(
            new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
            new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.L4Position),
            new MoveArmAndWrist(m_EE, EndEffectorConstants.L4ArmAngle, EndEffectorConstants.L4WristAngle));

    private final SequentialCommandGroup m_FloorGroup = new SequentialCommandGroup(
            new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
            new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.FloorPosition),
            new MoveArmAndWrist(m_EE, EndEffectorConstants.FloorArmAngle, EndEffectorConstants.FloorWristAngle));

    private final SequentialCommandGroup m_CORALSTUCKgroup = new SequentialCommandGroup(// sends elevator up, for if a
                                                                                        // coral is stuck
            new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
            new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.CoralStuckPosition)

    );

    private final SequentialCommandGroup m_verticalStowGroup = new SequentialCommandGroup(
            new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
            new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.FloorPosition));

    // private final SequentialCommandGroup m_AimGroup = new SequentialCommandGroup(

    // new EEVertical( m_EE),
    // new GoToLimelightPos(m_ElevatorSubsystem)
    // );

    private final SequentialCommandGroup m_AutoL4Group = new SequentialCommandGroup(
            new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle, EndEffectorConstants.VerticalWristAngle),
            new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.L4Position));

    private final SequentialCommandGroup m_stationCollect = new SequentialCommandGroup(
            new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.FloorPosition),
            new MoveArmAndWrist(m_EE, EndEffectorConstants.StationArmAngle, EndEffectorConstants.StationWristAngle),
            new StationCollect(m_EE, 0.1)

    );

    /**
     * if the arm is stuck at the station position from letting go of the button,
     * this should send it back
     */
    public final SequentialCommandGroup m_backFromStation = new SequentialCommandGroup(

    );

    public final SequentialCommandGroup m_algaeRemove = new SequentialCommandGroup(
            new MoveArmAndWrist(m_EE, EndEffectorConstants.AlgaeArmAngle, EndEffectorConstants.VerticalWristAngle),

            new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.L3Position),
            new MoveArmAndWrist(m_EE, EndEffectorConstants.L4ArmAngle, EndEffectorConstants.L4WristAngle)

    );

    public final SequentialCommandGroup m_L2algaeRemove = new SequentialCommandGroup(
            new MoveArmAndWrist(m_EE, EndEffectorConstants.AlgaeArmAngle, EndEffectorConstants.VerticalWristAngle),

            new MoveElevator(m_ElevatorSubsystem, ElevatorConstants.AlgaeL2),
            new MoveArmAndWrist(m_EE, EndEffectorConstants.L4ArmAngle, EndEffectorConstants.L4WristAngle)

    );

    public RobotContainer() {
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true); // uncomment this when testing with only 1 controller,
                                                              // this turns off the joystick unplugged warning
        m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric());

        NamedCommands.registerCommand("VerticalStow", new MoveArmAndWrist(m_EE, EndEffectorConstants.VerticalArmAngle,
                EndEffectorConstants.VerticalWristAngle));
        NamedCommands.registerCommand("Collect", m_FloorCollect); // put pathplanner commands here
        NamedCommands.registerCommand("LeftL4", m_LeftAutoScoreForAuto);
        NamedCommands.registerCommand("RightL4", m_RightAutoScoreForAuto);
        // NamedCommands.registerCommand("EeFloor", ); // the end effector to floor,
        // does not controll elevator
        NamedCommands.registerCommand("AimPrep", m_AutoL4Group); // Re-get these numbers and test this before adding
                                                                 // into autos
        NamedCommands.registerCommand("StationCollect", m_stationCollect);
        // the try catch loop makes the code not error, all this is doing is loading the
        // paths into pathplanner
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
                // m_drivetrain will execute this command periodically
                m_drivetrain.applyRequest(() -> drive
                        .withVelocityX(-MathUtil.applyDeadband(m_driveController.getLeftY(), 0.15) * MaxSpeed) // Drive
                                                                                                               // forward
                                                                                                               // with
                                                                                                               // negative
                                                                                                               // Y
                                                                                                               // (forward)
                        .withVelocityY(-MathUtil.applyDeadband(m_driveController.getLeftX(), 0.15) * MaxSpeed) // Drive
                                                                                                               // left
                                                                                                               // with
                                                                                                               // negative
                                                                                                               // X
                                                                                                               // (left)
                        .withRotationalRate(
                                -MathUtil.applyDeadband(m_driveController.getRightX(), 0.15) * MaxAngularRate) // Drive
                                                                                                               // counterclockwise
                                                                                                               // with
                                                                                                               // negative
                                                                                                               // X
                                                                                                               // (left)
                ));

        // m_driveController.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
        // m_driveController.b().whileTrue(m_drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-m_driveController.getLeftY(),
        // -m_driveController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // m_driveController.back().and(m_driveController.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        // m_driveController.back().and(m_driveController.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        // m_driveController.start().and(m_driveController.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        // m_driveController.start().and(m_driveController.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // m_drivetrain.registerTelemetry(logger::telemeterize); // commented out to
        // reduce RIO CPU usage

        // DRIVER CONTROLS

        // reset the field-centric heading on Y press
        m_driveController.y().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));
        // m_driveController.a().whileTrue(m_FloorCollect);

        m_driveController.button(6).and(() -> m_AutoScoringSubsystem.GetTargetLevel() == 2)
                .whileTrue(m_AutoScoringSubsystem.TargetAndAim(
                        m_AutoScoringSubsystem.GetTargetCommandGroup(2), "right", 2)); // Right Bumper

        m_driveController.button(6).and(() -> m_AutoScoringSubsystem.GetTargetLevel() == 1)
                .whileTrue(m_AutoScoringSubsystem.TargetAndAim(
                        m_AutoScoringSubsystem.GetTargetCommandGroup(1), "right", 1)); // Right Bumper

        m_driveController.button(6).and(() -> m_AutoScoringSubsystem.GetTargetLevel() == 0)
                .whileTrue(m_AutoScoringSubsystem.TargetAndAim(
                        m_AutoScoringSubsystem.GetTargetCommandGroup(0), "right", 0)); // Right Bumper

        m_driveController.button(5).and(() -> m_AutoScoringSubsystem.GetTargetLevel() == 2)
                .whileTrue(m_AutoScoringSubsystem.TargetAndAim(
                        m_AutoScoringSubsystem.GetTargetCommandGroup(2), "left", 2)); // left Bumper

        m_driveController.button(5).and(() -> m_AutoScoringSubsystem.GetTargetLevel() == 1)
                .whileTrue(m_AutoScoringSubsystem.TargetAndAim(
                        m_AutoScoringSubsystem.GetTargetCommandGroup(1), "left", 1)); // left Bumper

        m_driveController.button(5).and(() -> m_AutoScoringSubsystem.GetTargetLevel() == 0)
                .whileTrue(m_AutoScoringSubsystem.TargetAndAim(
                        m_AutoScoringSubsystem.GetTargetCommandGroup(0), "left", 0)); // left Bumper

        // m_driveController.button(6).whileTrue(m_RightAutoScore); // right bumper
        // m_driveController.button(5).whileTrue(m_LeftAutoScore); // left bumper

        m_driveController.leftTrigger().whileTrue(m_FloorCollect); // Left Trigger
        m_driveController.rightTrigger().whileTrue(m_ReverseCollector); // right trigger
        m_driveController.button(3).onTrue(m_StoweEE); // X
        // m_driveController.a().whileTrue(m_FloorGroup);
        // m_driveController.b().whileTrue(m_L4Group);
        // m_driveController.pov(0).whileTrue(m_L3Group);
        // m_driveController.x().whileTrue(m_stationCollect);

        // OPERATOR CONTROLS

        m_operatorController.button(5).onTrue(m_IncrementTargetLocation); // Left Bumper
        m_operatorController.button(1).onTrue(m_FloorGroup); // A
        m_operatorController.button(3).onTrue(m_StoweEE); // X
        m_operatorController.button(2).onTrue(m_StopElevatorAndEE);// B, the motors are not in brake mode, so the end
                                                                   // effector might fall down if you do this before
                                                                   // climbing .
        m_operatorController.pov(180).onTrue(m_L4Group); // manual L4 just incase LL fails
        m_operatorController.rightTrigger().whileTrue(m_stationCollect);// station pickup, hold the whole time
        m_operatorController.pov(0).onTrue(m_CORALSTUCKgroup);// incase coral gets stuck or elevator gets stuck
        m_operatorController.pov(270).onTrue(m_verticalStowGroup);
        m_operatorController.pov(90).onTrue(m_algaeRemove);
        m_operatorController.button(10).onTrue(m_L2algaeRemove);
        // m_L3Group);

        /*
         * DRIVER
         * drive - both joysticks
         * zero field orientation - Y
         * Auto Aim Left - LB
         * Auto Aim Right - RB
         * Collect - LT
         * reverse collector - RT
         * 
         * 
         * 
         * OPERATOR
         * climbing - Left Joystick
         * chose scoring target - LB
         * end effector to L1 - Y
         * move arm to Collect position/the floor - A
         * move arm to stowed position (for defence/moving around) - X
         * stop elevator/arm/collector - B
         * collect from station - RT
         * move arm back from station (incase it gets stuck there) - LT
         */

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
