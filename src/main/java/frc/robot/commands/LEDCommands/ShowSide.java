// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class ShowSide extends CommandBase {
  /** Creates a new ShowSide. */
  public static double LedColor;
  public ShowSide() {
    addRequirements(RobotContainer.m_LED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_Arm.getEncoderPosition() * RobotContainer.m_Drivetrain.speedMultiplier >= 0) {
      // RobotContainer.m_LED.setColor(0.77);
      LedColor = 0.77;
    } else {
      // RobotContainer.m_LED.setColor(0.91);
      LedColor = 0.91;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
