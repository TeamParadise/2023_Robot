// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.BackupRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.PidConstants;
import frc.robot.commands.ArmCommands.flipArmParallel;
import frc.robot.commands.AutoCommands.DriveDist;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeCubeLeft extends SequentialCommandGroup {
  /** Creates a new ThreeCube. */
  public ThreeCubeLeft() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      RobotContainer.m_Arm.autoNudgeThreeCube(),
      RobotContainer.m_Arm.waitForArmThreeCube(),
      RobotContainer.m_Arm.setPosition(2),
      RobotContainer.m_Drivetrain.resetGyro(),
      RobotContainer.m_Drivetrain.resetLeftEncoder(),
      RobotContainer.m_Drivetrain.resetRightEncoder(),
      new ParallelCommandGroup(new DriveDist(220, 0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(2.3), new flipArmParallel()),
      new DriveDist(220, 6, 0.2 - PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(0.3),
      new DriveDist(220, 0, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(0.4),
      new ParallelCommandGroup(new DriveDist(220, 0, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(2.03), new flipArmParallel()),
      // new DriveDist(0, PidConstants.DRIVE_SPEED - 0.3, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(0.37),
      RobotContainer.m_Arm.setPosition(1),
      new ParallelCommandGroup(new DriveDist(220, 0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(1.46), new flipArmParallel()),
      new DriveDist(220, 45, -PidConstants.TURN_SPEED, PidConstants.kp_TURN, PidConstants.ki_TURN, PidConstants.kd_TURN).withTimeout(2),
      new DriveDist(220, 40, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(0.95),
      new ParallelCommandGroup(new DriveDist(220, 0, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(1.4), new flipArmParallel()),
      // new DriveDist(5, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_ADJUST, PidConstants.kd_DRIVE).withTimeout(0.4),
      RobotContainer.m_Arm.setPosition(3)

    );
  }
}
