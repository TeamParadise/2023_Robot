// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LEDProfiles;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  // Initalize LED strip and LED buffer
  int ledStrip = LEDConstants.kLEDPWMPort;
  int ledBuffer = LEDConstants.kLEDLength;

  LEDProfiles strip1 = new LEDProfiles(ledStrip, ledBuffer);
  public LED() {
    setAll(Color.kRed);
  }

  @Override
  public void periodic() {
  }

  public void setAll(Color color){
    strip1.setAll(color);
  }

  public void blink(int times) {
    strip1.blink(times);
  }

  public void halfAndHalf(double side) {
    if (side < 0) {
      strip1.halfAndHalf(Color.kGreen, Color.kPurple);
    } else {
      strip1.halfAndHalf(Color.kPurple, Color.kGreen);
    }
  }
}