// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;

public final class Constants {
  /** Creates a new Constants. */




    public static class ClimberConstants{
      public static final int climberMotorID = 3; // placeholder

      public static final double ClimberMaxVoltage = 5; // PLACEHOLDER
      public static final int ClimbControllerAxis = 7;// placeholder, high number so it won't do anything until changed

    }

    public static class ElevatorConstants{


      public static final int leftElevatatorMotorID = 4; // placeholder
      public static final int rightElevatorMotorID = 5; // placeholder

      public static final double FloorPosition = 2; // placeholder
      public static final double L1Position = 3;// placeholder
      public static final double L2Position = 5 ; // placeholder
      public static final double L3Position = 6; // placeholder
      public static final double L4Position = 10; // placeholder 
      public static final double StationPosition = 30; // placeholder
      

      public static final CurrentLimitsConfigs ElevatorCurrentLimitConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(40)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40) 
        ;

    }

    public static class EndEffectorConstants{
      public static final int rightCollectorMotorID = 1; // placeholder
      public static final int leftCollectorMotorID = 2; // placeholder
      public static final int endEffectorAngleMotorID = 6; // placeholder

      public static final double collectorRPM = 300; // check this 

      public static final double FloorAngle = 10; // placeholder
      public static final double L1Angle = 30; // placeholder, might not be used
      public static final double L2Angle = 50; // Placeholder
      public static final double L4Angle = 20; // placeholder
      public static final double StoredAngle = 50; // the angle for starting configuration
  

      public static final double StationAngle = 20; // placeholder

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
