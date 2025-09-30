// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.TOFConstants;
import frc.robot.generated.TunerConstants;

public class LedSubsystem extends SubsystemBase {
  CANdle m_Candle; // candle is on RIO canbus
  EndEffector m_EE; // = new EndEffector();
  // public final CommandSwerveDrivetrain m_drivetrain =
  // TunerConstants.createDrivetrain();
  AutoScoringSubsystem m_AutoScoringSubsystem;// = new AutoScoringSubsystem(m_drivetrain);
  LimelightSubsystem m_LeftLL;// = new LimelightSubsystem("left");
  LimelightSubsystem m_RightLL;// = new LimelightSubsystem("right");

  /** Creates a new LedSubsystem. */
  public LedSubsystem(EndEffector EE, AutoScoringSubsystem autoScoringSubsystem,
      LimelightSubsystem LeftLimelightSubsystem, LimelightSubsystem RightLimelightSubsystem) {
    m_Candle = new CANdle(LEDConstants.CandleID);
    m_EE = EE;
    m_AutoScoringSubsystem = autoScoringSubsystem;
    m_LeftLL = LeftLimelightSubsystem;
    m_RightLL = RightLimelightSubsystem;

    // m_AutoScoringSubsystem = autoScoringSubsystem;
  }

  /** clears animation running in given animation slot */
  public void stopAnimation(int AnimationSlot) {
    m_Candle.clearAnimation(AnimationSlot);
  }

  public void LEDControl() {

    // top 1/3
    if (m_EE.HasCoral()) {
      m_Candle.setLEDs(0, 255, 0, 0, 8, 8); // does have coral, turn LEDs green
      m_Candle.setLEDs(0, 255, 0, 0, 32, 8);
    } else {
      m_Candle.setLEDs(255, 0, 0, 0, 32, 8);
      m_Candle.setLEDs(255, 0, 0, 0, 8, 8); // doesn't have coral, turn LEDs red
    }

    // middle 1/3
    if (m_LeftLL.HasTarget() || m_RightLL.HasTarget()) {
      // m_Candle.animate(, 0)
      m_Candle.setLEDs(255, 255, 0, 0, 16, 8);
      m_Candle.setLEDs(255, 255, 0, 0, 40, 8);
    } else {
      m_Candle.setLEDs(0, 0, 255, 0, 16, 8);
      m_Candle.setLEDs(0, 0, 255, 0, 40, 8);
    }
    ;

    // bottom 1/3
    if (m_AutoScoringSubsystem.GetTargetLevel() == 0) {
      m_Candle.setLEDs(0, 255, 0, 0, 24, 8);// L2
      m_Candle.setLEDs(0, 255, 0, 0, 48, 8);
    } else if (m_AutoScoringSubsystem.GetTargetLevel() == 1) {
      m_Candle.setLEDs(255, 255, 0, 0, 24, 8); // L3 
      m_Candle.setLEDs(255, 255, 0, 0, 48, 8);
    } else {
      m_Candle.setLEDs(255, 0, 0, 0, 24, 8); // L4
      m_Candle.setLEDs(255, 0, 0, 0, 48, 8);
    }
    // 24 per side
  }

  public void disabledAnimation() {
    m_Candle.animate( new RainbowAnimation(1, 0.5, LEDConstants.TotalLEDs));
    // m_Candle.setLEDs(255, 255, 255, 255, 0, 80);
    m_Candle.animate(new LarsonAnimation(255, 166, 0, 0, 0, 24, BounceMode.Back, 4, 8) , 0);
    m_Candle.animate(new LarsonAnimation(255, 166, 0, 0, 0, 24, BounceMode.Back, 4, 32), 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}