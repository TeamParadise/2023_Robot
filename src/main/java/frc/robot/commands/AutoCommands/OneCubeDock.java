// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.PidConstants;
import frc.robot.commands.DriveDist;
import frc.robot.commands.DriveDistBackHalf;
import frc.robot.commands.ArmCommands.flipArmParallel;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class OneCubeDock extends SequentialCommandGroup {
  

  public OneCubeDock() {
    addCommands(
      RobotContainer.m_Arm.autoNudge(),
      RobotContainer.m_Arm.waitForArm(),
      RobotContainer.m_Arm.setPosition(2),
      new ParallelCommandGroup(new DriveDist(220, 0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(2.5), new flipArmParallel()),
      new ParallelCommandGroup(new DriveDistBackHalf())
      // RobotContainer.m_Vision.setToBackPipeline(),     
      // new TurnToTarget(),
      // RobotContainer.m_Arm.setPosition(3)
    );
  }
}
