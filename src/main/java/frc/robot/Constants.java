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
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class Constants {
  /** Creates a new Constants. */

    public static class ClimberConstants{
      public static final int climberMotorID = 1; 
      public static final int climber2MotorID = 2; 
      public static final double ClimberMaxVoltage = 2; // raise this later 
      public static final int ClimbControllerAxis = 1;

    }

    public static class ElevatorConstants{


      public static final int leftElevatatorMotorID = 6; // placeholder
      public static final int rightElevatorMotorID = 5; // placeholder

      public static final double FloorPosition = 0.4; // placeholder
      public static final double L1Position = 0.4;// placeholder
      public static final double L2Position = 34.45 ; // placeholder
      public static final double L3Position = 23.87; // placeholder
      public static final double L4Position = 52.5;
      public static final double AlgaeLowPosition = 7; // placeholder 
      public static final double AlgaeHighPosition = 13; // placeholder 
      public static final double StationPosition = 10; // placeholder
      public static final double LlAimPosition = 17; // position used while aiming using the limelights 


      // motion magic configs
      
      public static final double MotionMagicAcceleration = 1;
      public static final double MotionMagicCruiseVelocity = 3;
      
      // motor configs (power/motion magic configs)

      public static final CurrentLimitsConfigs ElevatorCurrentLimitConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(40)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40) 
        ;

      public static final MotionMagicConfigs ElevatorMotionMagicConfigs = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(0)
      .withMotionMagicExpo_kA(0.1)
      .withMotionMagicExpo_kV(0.1) // rotations per second, velocity control doens't use this, expo and position do.
     
      ;


      public static final Slot0Configs ElevatorSlot0Configs = new Slot0Configs()
      .withKP(0.15)
      .withKG(0.0225)
      
      ;

      public static final MotorOutputConfigs ElevatorMotorOutputConfigs = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(InvertedValue.CounterClockwise_Positive)
      ;

    }

    /** constants for the Arm and Wrist of End Effector */
    public static class EndEffectorConstants{


      // motor ids 
      public static final int rightCollectorMotorID = 7; // placeholder
      public static final int leftCollectorMotorID = 8; // placeholder
      public static final int endEffectorWristMotorID = 26; // placeholder
      public static final int endEffectorArmMotorID = 19; // placeholder

      public static final int endEffectorArmEncoderID = 5; // remember that the swerve encoders are the same device type so can't have the same ID
      public static final int endEffectorWristEncoderID = 6;


      public static final double collectorRPM = 10; // check this 

      // wrist angles
      public static final double FloorWristAngle = -0.09; // placeholder angle to collect off the floor   
      public static final double L1WristAngle = -0.43; // placeholder, angle to score L1
      public static final double L2WristAngle = -.11; // also for L3
      public static final double L3WristAngle = -0.177;
      public static final double L4WristAngle = -0.14; // placeholder
      public static final double AlgaeWristAngle = 0.1; // placeholder
      public static final double StowedWristAngle =  -.09; // the angle for defence, placeholder
      public static final double VerticalWristAngle = -0.11 ;
      public static final double StationWristAngle = 2; // placeholder angle for collecting at station

      //arm angles
      public static final double FloorArmAngle = -0.09; // placeholder
      public static final double L1ArmAngle = 0.09; // placeholder, might not be used
      public static final double L2ArmAngle = -0.15; // also for L3
      public static final double L3ArmAngle = 0.127;
      public static final double L4ArmAngle = .1; // placeholder
      public static final double AlgaeArmAngle = .32; // placeholder
      public static final double StowedArmAngle = 0.14; // the angle for starting configuration, placeholder
      public static final double VerticalArmAngle = 0.13;
      public static final double StationArmAngle = .20; // placeholder angle for collecting at the station

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

      public static final Slot0Configs ArmSlot0Configs = new Slot0Configs()
      .withGravityType(GravityTypeValue.Arm_Cosine)
      .withKP(11)
      .withKG(0); // 0.0175


      public static final Slot0Configs WristSlot0Configs = new Slot0Configs()
      .withKP(1.5)
      ;


      public static final CANcoderConfiguration WristCanCoderConfig =
       new CANcoderConfiguration()
      .withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(-0.303955078125));

      public static final CANcoderConfiguration ArmCanCoderConfig = 
      new CANcoderConfiguration()
      .withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(-0.28173828125)
      .withAbsoluteSensorDiscontinuityPoint(0.8));
    
      public static final MotorOutputConfigs ArmMotorConfig = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(InvertedValue.Clockwise_Positive);

      public static final MotorOutputConfigs WristMotorConfig = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(InvertedValue.CounterClockwise_Positive);

    }


    public static class LEDConstants{
      public static final int CandleID = 0;
      public static final int TotalLEDs = 99; // placeholder
    }

    public static class LazerCanConstants{
      public static final int LcID = 0; // placeholder
      public static final int LcMaxDistance = 50; // mm
      public static final int LcMinDistance = 10; //mm

    }

    public static class LimelightConstants{




     public static final double rightTargetLeftEdge = 0.41; // meters, farthest left the left limelight sees the apriltag (for aiming right side)
     public static final double rightTargetRightEdge = -0.66; //meters 
     public static final double rightTargetCenter = -0.205; // meters 
    

     public static final double leftTargetLeftEdge = 0.73; // meters, farthest left the left limelight sees the apriltag (for aiming right side)
     public static final double leftTargetRightEdge = -0.46; //meters 
     public static final double leftTargetCenter = 0.14; // meters
    
     public static final double AimingSpeedMultiplier = 0.7; // kP for lateral aiming        0.84 for the right target/left limelight/LL3
     public static final double AimingTurnSpeedMultiplier = 1.1;
     public static final double RightLateralFF = 0.01; // feed forward on aiming right target
     public static final double LeftLateralFF = 0.005;

     public static final double MaxAimSpeed = 3;// meters per second

     public static final double TargetYDeadband = 0.05; // meters
     public static final double TargetYawDeadband = 4; // deg
     public static final double TargetYaw = 0;// RAD
      public static final double TargetZ = 0.43
      ; // meters
      public static final double MaxTargetZ = 1.85; // meters
      public static final double TargetZSpeedMultiplier = 1.1;

    }


  


}
