// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.pathplanner.lib.config.RobotConfig;

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

      public static final double FloorPosition = 2; // placeholder
      public static final double L1Position = 3;// placeholder
      public static final double L2Position = 5 ; // placeholder
      public static final double L3Position = 10; // placeholder
      public static final double L4Position = 15; // placeholder 
      public static final double StationPosition = 10; // placeholder
      

      public static final CurrentLimitsConfigs ElevatorCurrentLimitConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(40)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40) 
        ;

    }

    public static class EndEffectorConstants{
      // motor ids 
      public static final int rightCollectorMotorID = 7; // placeholder
      public static final int leftCollectorMotorID = 8; // placeholder
      public static final int endEffectorWristMotorID = 9; // placeholder
      public static final int endEffectorArmMotorID = 10; // placeholder

      public static final double collectorRPM = 100; // check this 


      // wrist angles
      public static final double FloorWristAngle = 10; // placeholder angle to collect off the floor
      public static final double L1WristAngle = 30; // placeholder, angle to score L1
      public static final double L2WristAngle = 50; // Placeholder
      public static final double L4WristAngle = 20; // placeholder
      public static final double StartingWristAngle = 50; // the angle for starting configuration, placeholder
      public static final double StationWristAngle = 20; // placeholder angle for collecting at station

      //arm angles
      public static final double FloorArmAngle = 10; // placeholder
      public static final double L1ArmAngle = 30; // placeholder, might not be used
      public static final double L2ArmAngle = 50; // Placeholder
      public static final double L4ArmAngle = 20; // placeholder
      public static final double StartingArmAngle = 50; // the angle for starting configuration, placeholder
      public static final double StationArmAngle = 20; // placeholder angle for collecting at the station

      public static final CurrentLimitsConfigs EECurrentLimitConfigs = new CurrentLimitsConfigs()
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(40)
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(40)  
      ;
    }


    public static class LEDConstants{
      public static final int CandleID = 1; // placeholder
      public static final int TotalLEDs = 8; // placeholder
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


    }


  


}
