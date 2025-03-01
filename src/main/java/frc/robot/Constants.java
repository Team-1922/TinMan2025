// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

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

      public static final double FloorPosition = 1; // placeholder
      public static final double L1Position = 5;// placeholder
      public static final double L2Position = 10 ; // placeholder
      public static final double L3Position = 15; // placeholder
      public static final double L4Position = 24;
      public static final double AlgaeLowPosition = 7; // placeholder 
      public static final double AlgaeHighPosition = 13; // placeholder 
      public static final double StationPosition = 10; // placeholder
      


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
      .withKP(0.05)
      .withKG(0.02)
      
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


      public static final double collectorRPM = 100; // check this 

      // wrist angles
      public static final double FloorWristAngle = 1; // placeholder angle to collect off the floor   
      public static final double L1WristAngle = 1; // placeholder, angle to score L1
      public static final double L2WristAngle = 4; // Placeholder
      public static final double L4WristAngle = 8; // placeholder
      public static final double AlgaeWristAngle = 2; // placeholder
      public static final double StartingWristAngle = 5; // the angle for starting configuration, placeholder
      public static final double StationWristAngle = 2; // placeholder angle for collecting at station

      //arm angles
      public static final double FloorArmAngle = 70; // placeholder
      public static final double L1ArmAngle = 100; // placeholder, might not be used
      public static final double L2ArmAngle = 120; // Placeholder
      public static final double L4ArmAngle = 180; // placeholder
      public static final double AlgaeArmAngle = -.325; // placeholder
      public static final double StartingArmAngle = -0.05; // the angle for starting configuration, placeholder
      public static final double StationArmAngle = 20; // placeholder angle for collecting at the station

      public static final FeedbackConfigs ArmFeedbackConfigs = new FeedbackConfigs()
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
      .withFeedbackRemoteSensorID(endEffectorArmEncoderID)
      .withRotorToSensorRatio(50)
      .withSensorToMechanismRatio(2)
      ;

      public static final FeedbackConfigs WristFeedbackConfigs = new FeedbackConfigs()
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
      .withFeedbackRemoteSensorID(endEffectorWristEncoderID)
      ;

      public static final CurrentLimitsConfigs EECurrentLimitConfigs = new CurrentLimitsConfigs()
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(40)
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(40)  
      ;

      public static final Slot0Configs ArmSlot0Configs = new Slot0Configs()
      .withGravityType(GravityTypeValue.Arm_Cosine)
      .withKP(0.01)
      .withKG(0.012);
    }


    public static class LEDConstants{
      public static final int CandleID = 0;
      public static final int TotalLEDs = 99; // placeholder
    }

    public static class TimeOfFlightConstants{
      public static final int TOFID = 3; // placeholder
      public static final double MaxRange = 100;// placeholder
      public static final double MinRange = 20; // placeholder
    }

    public static class LimelightConstants{

     public static final double LeftMinAngle = -20;
     public static final double LeftMaxAngle = 20;
     public static final double RightMinAngle = -20;
     public static final double RightMaxAngle = 20;
      
     public static final double MaxAimSpeed = 4;// meters per second

     public static final double TargetYDeadband = 0.025; // meters
     public static final double TargetYawDeadband = 5; // deg

    }


  


}
