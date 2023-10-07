// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.PidConstants;
import frc.robot.commands.DriveDist;
import frc.robot.commands.DriveDistBack;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.ArmCommands.flipArmParallel;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class TwoCube extends SequentialCommandGroup {
  

  public TwoCube() {
    addCommands(
      RobotContainer.m_Arm.autoNudge(),
      RobotContainer.m_Arm.waitForArm(),
      RobotContainer.m_Arm.setPosition(1),
      RobotContainer.m_Drivetrain.resetGyro(),
      
      //Drive to Cube 2
      new ParallelCommandGroup(new DriveDist(0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(2.6), new flipArmParallel()), //try to see if this will flip the arm and drive at the same time. if not delete this line and uncomment below.
      new WaitCommand(1),
      
      //Drive to Grid 2
      new ParallelCommandGroup(new DriveDist(0, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(2.4), new flipArmParallel()),
      new WaitCommand(0.5),
      //Shoot cube 2
      RobotContainer.m_Arm.setPosition(3),

      new ParallelCommandGroup(new DriveDist(0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(2), new flipArmParallel())

    );
  }
}
