// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
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

      public static final double ConversionFactor = 2*(Math.PI*1.432)/9; 
      public static final double FloorPosition = 0.2/ConversionFactor;
      public static final double L1Position = FloorPosition;
      public static final double L2Position = FloorPosition + 11.1/ConversionFactor;//41.6 ; // placeholder?
      public static final double L3Position = FloorPosition+ 27.8/ConversionFactor; //28.9;
      public static final double L4Position = FloorPosition+ 54.3/ConversionFactor;//54.5;
      public static final double AlgaeLowPosition = 31.08/ConversionFactor; // placeholder 
      public static final double AlgaeHighPosition = 38.22/ConversionFactor; // placeholder 
      public static final double StationPosition = FloorPosition +13.4/ConversionFactor;// 11.5; // placeholder
      public static final double StationHalfWayPosition = FloorPosition +5/ConversionFactor;// 11.5; // placeholder
      public static final double CoralStuckPosition = FloorPosition +17/ConversionFactor; // for if coral is stuck
      public static final double AlgaeL2 = FloorPosition + 13.17/ConversionFactor;

      public static final double LlAimPosition = 23; // position used while aiming using the limelights 
//

      // motion magic configs
      
      public static final double MotionMagicAcceleration = 1;
      public static final double MotionMagicCruiseVelocity = 3;
      
      // motor configs (power/motion magic configs)

      public static final CurrentLimitsConfigs ElevatorCurrentLimitConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(30)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(30)
        .withSupplyCurrentLowerLimit(29)
        .withSupplyCurrentLowerTime(0.75)        ;

      public static final MotionMagicConfigs ElevatorMotionMagicConfigs = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(0)
      .withMotionMagicExpo_kA(0.01)
      .withMotionMagicExpo_kV(0.2); // rotations per second, velocity control doens't use this, expo and position do.    ;


      public static final Slot0Configs ElevatorSlot0Configs = new Slot0Configs()
      .withKP(0.75)
      .withKG(0.0225)
      .withKD(0.03)
      .withKS(0.0137);

      public static final ClosedLoopRampsConfigs ElevatorClosedLoopRampConfigs = new ClosedLoopRampsConfigs()
      .withDutyCycleClosedLoopRampPeriod(0);

      public static final MotorOutputConfigs ElevatorMotorOutputConfigs = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.CounterClockwise_Positive);

    }

    /** constants for the Arm and Wrist of End Effector */
    public static class EndEffectorConstants{


      // motor ids 
      public static final int rightCollectorMotorID = 8;
      public static final int leftCollectorMotorID = 7; 
      public static final int topCollectorMotorID = 0;
      public static final int endEffectorWristMotorID = 26; 
      public static final int endEffectorArmMotorID = 19;

      public static final int endEffectorArmEncoderID = 5; // remember that the swerve encoders are the same device type so can't have the same ID
      public static final int endEffectorWristEncoderID = 6;


      public static final double collectorRPM = 10; // check this 

      // wrist angles
      public static final double FloorWristAngle = -0.05; //  angle to collect off the floor   
      public static final double L1WristAngle = 0.23; // angle to score L1
      public static final double L2WristAngle = -0.15; // 
      public static final double L3WristAngle = -0.175;
      public static final double L4WristAngle = -0.105 ; 
      public static final double AlgaeWristAngle = 0.12; // placeholder
      public static final double StowedWristAngle =  .01; // the angle for defence
      public static final double VerticalWristAngle = -0.02;
      public static final double StationWristAngle = -0.25; // placeholder angle for collecting at station

      //arm angles
      public static final double FloorArmAngle = -0.07; 
      public static final double L1ArmAngle = 0.128; 
      public static final double L2ArmAngle = 0.138; // underflows at -0.2
      public static final double L3ArmAngle = 0.154;

      
      public static final double L4ArmAngle = .14;
      public static final double AlgaeArmAngle = .122; // placeholder
      public static final double StowedArmAngle = 0.2; // the angle for starting configuration
      public static final double VerticalArmAngle = 0.21;
 
      public static final double StationArmAngle = 0.22; // placeholder angle for collecting at the station
      

      public static final FeedbackConfigs ArmFeedbackConfigs = new FeedbackConfigs()
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
      .withFeedbackRemoteSensorID(endEffectorArmEncoderID)
      .withRotorToSensorRatio(50)
      .withSensorToMechanismRatio(1.5)
      ;

      public static final MotionMagicConfigs ArmMotionMagicConfigs = new MotionMagicConfigs()
      .withMotionMagicExpo_kV(10)
      .withMotionMagicExpo_kA(8)
      ;

      public static final MotionMagicConfigs WristMotionMagicConfigs = new MotionMagicConfigs()
      .withMotionMagicExpo_kA(0.1)
      .withMotionMagicExpo_kV(3.5)
      ;

      public static final FeedbackConfigs WristFeedbackConfigs = new FeedbackConfigs()
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
      .withFeedbackRemoteSensorID(endEffectorWristEncoderID)
      .withRotorToSensorRatio(20)
      .withSensorToMechanismRatio(1);
      ;

      public static final CurrentLimitsConfigs EECurrentLimitConfigs = new CurrentLimitsConfigs()
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(20)
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(20)  
      .withSupplyCurrentLowerLimit(20)
      .withSupplyCurrentLowerTime(0.75)
      ;

      public static final CurrentLimitsConfigs CollectorCurrentLimitConfigs = new CurrentLimitsConfigs()
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(20)
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(20)
      .withSupplyCurrentLowerLimit(20)
      .withSupplyCurrentLowerTime(0.75);  
      ;

      public static final Slot0Configs ArmSlot0Configs = new Slot0Configs()
      .withGravityType(GravityTypeValue.Arm_Cosine)
     .withKG(0.016)
     .withKP(2.5);

      public static final Slot0Configs WristSlot0Configs = new Slot0Configs()
      .withKP(1);
    //  public static final MotorOutputConfigs M_ArmOutputConfigs = new MotorOutputConfigs()
    //  .withNeutralMode(NeutralModeValue.Coast);

      public static final ClosedLoopRampsConfigs closedLoopRampConfigs = new ClosedLoopRampsConfigs()
    .withDutyCycleClosedLoopRampPeriod(0.0);

    public static final ClosedLoopRampsConfigs openLoopRampConfigs = new ClosedLoopRampsConfigs()
    .withDutyCycleClosedLoopRampPeriod(0);


      


      public static final CANcoderConfiguration WristCanCoderConfig =
       new CANcoderConfiguration()
      .withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(0.31982421875));

      public static final CANcoderConfiguration ArmCanCoderConfig = 
      new CANcoderConfiguration()
      .withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(-0.444091796875)
      .withAbsoluteSensorDiscontinuityPoint(0.8)
      );
    
      public static final MotorOutputConfigs ArmMotorConfig = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.CounterClockwise_Positive);

      public static final MotorOutputConfigs WristMotorConfig = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.CounterClockwise_Positive);
      
      public static final MotorOutputConfigs EEClimbedConfigs = new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Brake);
    }

  


    public static class LEDConstants{
      public static final int CandleID = 0;
      public static final int TotalLEDs = 56; // placeholder
    }

    public static class TOFConstants{
      public static final int TOFID = 0; 
      public static final int TOFID2 = 1; // the TOF that is further back into the collector
      public static final double TOFMaxDistance = 75; // mm
      public static final double TOFMinDistance = 1; //mm
      
      public static final double TOF2MaxDistance = 50; // mm
      public static final double TOF2MinDistance = 0; //mm

    }

    public static class LimelightConstants{



// -0.25, 0.07
     public static final double rightTargetLeftEdge = 0.07; // meters, farthest left the left limelight sees the apriltag (for aiming right side)
     public static final double rightTargetRightEdge = -0.25; //meters 
     public static final double rightTargetCenter = -0.18; // meters 
    

     public static final double leftTargetLeftEdge = 0.73; // meters, farthest left the left limelight sees the apriltag (for aiming right side)
     public static final double leftTargetRightEdge = -0.46; //meters 
     public static final double leftTargetCenter = 0.17; // meters
    
     public static final double AimingSpeedMultiplier = 0.7; // kP for lateral aiming        0.84 for the right target/left limelight/LL3
     public static final double AimingTurnSpeedMultiplier = 1.05;
     public static final double RightLateralFF = 0.005; // feed forward on aiming right target
     public static final double LeftLateralFF = 0.002;

     public static final double MaxAimSpeed = 3;// meters per second

     public static final double TargetYDeadband = 0.05; // meters
     public static final double TargetYawDeadband = 4; // deg
     public static final double TargetYaw = 0;// RAD
      public static final double TargetZ = 0.39; // meters
      public static final double L2TargetZ = 0.78; // target for backing up from the reef to put the arm into a stowed position
      public static final double MaxTargetZ = 1.85; // meters
      public static final double TargetZSpeedMultiplier = 0.68;

    }


  


}
