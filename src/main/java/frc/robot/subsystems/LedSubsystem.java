// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

  public static Spark m_blinkin = new Spark(0);
  /** Creates a new LedSubsystem. */
  public LedSubsystem() {}

  public void setColor(double pwmValue) {
    m_blinkin.set(pwmValue);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
