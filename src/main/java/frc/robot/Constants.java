// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



public final class Constants {
  /** Creates a new Constants. */




    public static class ClimberConstants{
      public static final int climberMotorID = 3; // placeholder

      public static final double ClimberMaxRPM = 150; // PLACEHOLDER
      public static final int ClimbControllerAxis = 7;// placeholder, high number so it won't do anything until changed

    }

    public static class ElevatorConstants{
      public static final int leftElevatatorMotorID = 4; // placeholder
      public static final int rightElevatorMotorID = 5; // placeholder


    }

    public static class EndEffectorConstants{
      public static final int rightCollectorMotorID = 1; // placeholder
      public static final int leftCollectorMotorID = 2; // placeholder
      public static final int endEffectorAngleMotorID = 6; // placeholder

      public static final double collectorRPM = 300; // check this 
    }

    public static class LimelightConstants{

     public static final double LeftMinAngle = -20;
     public static final double LeftMaxAngle = 20;
     public static final double RightMinAngle = -20;
     public static final double RightMaxAngle = 20;


    }


  


}
