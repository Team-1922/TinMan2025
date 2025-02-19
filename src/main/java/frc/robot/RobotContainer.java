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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.ClimbCommand;
import frc.robot.Commands.Collect;
import frc.robot.Commands.LeftAim;
import frc.robot.Commands.RightAim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

   

    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_joystick = new CommandXboxController(0); // DRIVER CONTROLLER
    private final CommandXboxController m_operatorController = new CommandXboxController(1); // operator

    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    private final LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();
    private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    private final EndEffector m_EE = new EndEffector();
    private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

    private final RightAim m_RightAim = new RightAim(m_LimelightSubsystem, m_drivetrain, m_joystick,drive);
    private final LeftAim m_LeftAim = new LeftAim(m_LimelightSubsystem, m_drivetrain,m_joystick,drive);
    private final Collect m_Collect = new Collect(m_EE);
    private final ClimbCommand m_ClimbCommand = new ClimbCommand(m_ClimberSubsystem, m_operatorController);
   
    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        configureBindings();
        
    m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric());


    NamedCommands.registerCommand("Collect", m_Collect); // put pathplanner commands here
    
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
                drive.withVelocityX(-MathUtil.applyDeadband(m_joystick.getLeftY(),0.15) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-MathUtil.applyDeadband(m_joystick.getLeftX(),0.15) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-MathUtil.applyDeadband(m_joystick.getRightX(),0.15) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        m_joystick.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
        m_joystick.b().whileTrue(m_drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_joystick.getLeftY(), -m_joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_joystick.back().and(m_joystick.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        m_joystick.back().and(m_joystick.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        m_joystick.start().and(m_joystick.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        m_joystick.start().and(m_joystick.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        m_joystick.y().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

        m_drivetrain.registerTelemetry(logger::telemeterize);
        m_joystick.button(5).whileTrue(m_LeftAim); // left bumper
    }

    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
    }
}
