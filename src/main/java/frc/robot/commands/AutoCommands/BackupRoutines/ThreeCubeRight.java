// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.BackupRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.PidConstants;
import frc.robot.commands.ArmCommands.flipArmParallel;
import frc.robot.commands.DriveCommands.TurnAngle;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeCubeRight extends SequentialCommandGroup {
  /** Creates a new ThreeCube. */
  public ThreeCubeRight() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      RobotContainer.m_Drivetrain.resetGyro().andThen(RobotContainer.m_Arm.autoNudgeThreeCube()),
      RobotContainer.m_Arm.waitForArmThreeCube(),
      RobotContainer.m_Arm.setPosition(2),
      RobotContainer.m_Drivetrain.resetGyro(),
      RobotContainer.m_Drivetrain.resetLeftEncoder(),
      RobotContainer.m_Drivetrain.resetRightEncoder(),
      //Get cube 2      RobotContainer.m_Drivetrain.resetEncoders().andThen(new ParallelCommandGroup(new DriveDist(150, 0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT), new flipArmParallel())),

      RobotContainer.m_Drivetrain.resetEncoders().andThen(new ParallelCommandGroup(new DriveDistPid(150, 0, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT), new flipArmParallel())),
      //Shoot cube 2
      RobotContainer.m_Arm.setPosition(1),
      //Drive cube 3
      RobotContainer.m_Drivetrain.resetEncoders().andThen(new ParallelCommandGroup(new DriveDistPid(100, 0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT), new flipArmParallel())),
      //Turn to cube 3
      new TurnAngle(-45).withTimeout(1.5),
      //Drive past charge station 
      RobotContainer.m_Drivetrain.resetEncoders().andThen(new DriveDistPid(50, -120, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT)),
      RobotContainer.m_Drivetrain.resetEncoders().andThen(new DriveDistPid(50, -120, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT)),
      //Turn to grid
      new TurnAngle(0).withTimeout(1.5),
      //Drive to grid
      RobotContainer.m_Drivetrain.resetEncoders().andThen(new ParallelCommandGroup(new DriveDistPid(100, 0, PidConstants.TURN_SPEEDRIGHT, PidConstants.kp_DRIVERIGHT, PidConstants.ki_TURNRIGHT, PidConstants.kd_DRIVERIGHT), new flipArmParallel())),
      // new DriveDist(5, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_ADJUST, PidConstants.kd_DRIVE).withTimeout(0.4),
      RobotContainer.m_Arm.setPosition(3)

    );
  }
}
