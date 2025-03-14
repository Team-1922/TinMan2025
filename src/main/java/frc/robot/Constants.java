// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class Constants {
  /** Creates a new Constants. */

    public static class ClimberConstants{
      public static final int climberMotorID = 2; 
     // public static final int climber2MotorID = 2; 
      public static final double ClimberMaxVoltage = 5; // 8 is too high
      public static final int ClimbControllerAxis = 1;

    }

    public static class ElevatorConstants{


      public static final int leftElevatatorMotorID = 6; 
      public static final int rightElevatorMotorID = 5; 

      public static final double FloorPosition = -0.15;
      public static final double L1Position = 0.25;
      public static final double L2Position = 41.6 ; // placeholder?
      public static final double L3Position = 28.8;
      public static final double L4Position = 52.4;
      public static final double AlgaeLowPosition = 31.08; // placeholder 
      public static final double AlgaeHighPosition = 38.22; // placeholder 
      public static final double StationPosition = 10; // placeholder
      public static final double LlAimPosition = 23; // position used while aiming using the limelights 


      // motion magic configs
      
      public static final double MotionMagicAcceleration = 1;
      public static final double MotionMagicCruiseVelocity = 3;
      
      // motor configs (power/motion magic configs)

      public static final CurrentLimitsConfigs ElevatorCurrentLimitConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(40)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40);

      public static final MotionMagicConfigs ElevatorMotionMagicConfigs = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(0)
      .withMotionMagicExpo_kA(0.1)
      .withMotionMagicExpo_kV(0.1); // rotations per second, velocity control doens't use this, expo and position do.    ;


      public static final Slot0Configs ElevatorSlot0Configs = new Slot0Configs()
      .withKP(0.5)
      .withKG(0.022);

      public static final MotorOutputConfigs ElevatorMotorOutputConfigs = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.CounterClockwise_Positive);

    }

    /** constants for the Arm and Wrist of End Effector */
    public static class EndEffectorConstants{


      // motor ids 
      public static final int rightCollectorMotorID = 7;
      public static final int leftCollectorMotorID = 8; 
      public static final int endEffectorWristMotorID = 26; 
      public static final int endEffectorArmMotorID = 19;

      public static final int endEffectorArmEncoderID = 5; // remember that the swerve encoders are the same device type so can't have the same ID
      public static final int endEffectorWristEncoderID = 6;


      public static final double collectorRPM = 10; // check this 

      // wrist angles
      public static final double FloorWristAngle = 0.04; //  angle to collect off the floor   
      public static final double L1WristAngle = 0.23; // angle to score L1
      public static final double L2WristAngle = 0.09; // 
      public static final double L3WristAngle = -0.08;
      public static final double L4WristAngle = -0.06; 
      public static final double AlgaeWristAngle = 0.12; // placeholder
      public static final double StowedWristAngle =  .03; // the angle for defence
      public static final double VerticalWristAngle = .13 ;
      public static final double StationWristAngle = 23; // placeholder angle for collecting at station

      //arm angles
      public static final double FloorArmAngle = -0.075; 
      public static final double L1ArmAngle = 0.138; 
      public static final double L2ArmAngle = -0.15; // underflows at -0.2
      public static final double L3ArmAngle = 0.13;
      public static final double L4ArmAngle = .13; 
      public static final double AlgaeArmAngle = -0.06; // placeholder
      public static final double StowedArmAngle = 0.25; // the angle for starting configuration
      public static final double VerticalArmAngle = 0.22;
      public static final double StationArmAngle = .43; // placeholder angle for collecting at the station

      public static final FeedbackConfigs ArmFeedbackConfigs = new FeedbackConfigs()
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
      .withFeedbackRemoteSensorID(endEffectorArmEncoderID)
      .withRotorToSensorRatio(50)
      .withSensorToMechanismRatio(1.5)
      ;

      public static final FeedbackConfigs WristFeedbackConfigs = new FeedbackConfigs()
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
      .withFeedbackRemoteSensorID(endEffectorWristEncoderID)
      .withRotorToSensorRatio(20)
      ;

      public static final CurrentLimitsConfigs EECurrentLimitConfigs = new CurrentLimitsConfigs()
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(40)
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(40)  
      ;

      public static final CurrentLimitsConfigs CollectorCurrentLimitConfigs = new CurrentLimitsConfigs()
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(30)
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(30)  
      ;

      public static final Slot0Configs ArmSlot0Configs = new Slot0Configs()
      .withGravityType(GravityTypeValue.Arm_Cosine)
      .withKP(4) //
      .withKG(0.02); // 

      public static final MotorOutputConfigs M_ArmOutputConfigs = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Coast);

      public static final Slot0Configs WristSlot0Configs = new Slot0Configs()
      .withKP(1.5)
      ;


      public static final CANcoderConfiguration WristCanCoderConfig =
       new CANcoderConfiguration()
      .withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(0.49560546875));

      public static final CANcoderConfiguration ArmCanCoderConfig = 
      new CANcoderConfiguration()
      .withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(-0.28564453125)
      .withAbsoluteSensorDiscontinuityPoint(0.8));
    
      public static final MotorOutputConfigs ArmMotorConfig = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.CounterClockwise_Positive);

      public static final MotorOutputConfigs WristMotorConfig = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.CounterClockwise_Positive);

    }


    public static class LEDConstants{
      public static final int CandleID = 0;
      public static final int TotalLEDs = 56; // placeholder
    }

    public static class TOFConstants{
      public static final int TOFID = 0; // placeholder
      public static final double TOFMaxDistance = 50; // mm
      public static final double TOFMinDistance = 0; //mm

    }

    public static class LimelightConstants{



// -0.25, 0.07
     public static final double rightTargetLeftEdge = 0.07; // meters, farthest left the left limelight sees the apriltag (for aiming right side)
     public static final double rightTargetRightEdge = -0.25; //meters 
     public static final double rightTargetCenter = -0.18; // meters 
    

     public static final double leftTargetLeftEdge = 0.73; // meters, farthest left the left limelight sees the apriltag (for aiming right side)
     public static final double leftTargetRightEdge = -0.46; //meters 
     public static final double leftTargetCenter = 0.15; // meters
    
     public static final double AimingSpeedMultiplier = 0.7; // kP for lateral aiming        0.84 for the right target/left limelight/LL3
     public static final double AimingTurnSpeedMultiplier = 1.05;
     public static final double RightLateralFF = 0.005; // feed forward on aiming right target
     public static final double LeftLateralFF = 0.002;

     public static final double MaxAimSpeed = 3;// meters per second

     public static final double TargetYDeadband = 0.05; // meters
     public static final double TargetYawDeadband = 4; // deg
     public static final double TargetYaw = 0;// RAD
      public static final double TargetZ = 0.4; // meters
      public static final double L2TargetZ = 0.78; // target for backing up from the reef to put the arm into a stowed position
      public static final double MaxTargetZ = 1.85; // meters
      public static final double TargetZSpeedMultiplier = 0.475;

    }


  


}
