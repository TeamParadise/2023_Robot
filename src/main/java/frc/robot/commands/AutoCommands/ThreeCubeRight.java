// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.PidConstants;
import frc.robot.commands.DriveDist;
import frc.robot.commands.DriveDistBack;
import frc.robot.commands.TurnAngle;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.ArmCommands.flipArmParallel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeCubeRight extends SequentialCommandGroup {
  /** Creates a new ThreeCube. */
  public ThreeCubeRight() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      RobotContainer.m_Arm.autoNudgeThreeCube(),
      RobotContainer.m_Arm.waitForArmThreeCube(),
      RobotContainer.m_Arm.setPosition(2),
      RobotContainer.m_Drivetrain.resetGyro(),
      RobotContainer.m_Drivetrain.resetLeftEncoder(),
      RobotContainer.m_Drivetrain.resetRightEncoder(),
      //Get cube 2
      new ParallelCommandGroup(new DriveDist(0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT).withTimeout(2.3), new flipArmParallel()),
      new DriveDist(0, 0.3 - PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT).withTimeout(0.4),
      //Shoot cube 2
      new DriveDist(0, PidConstants.DRIVE_SPEED - 0.2, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT).withTimeout(0.45),
      new ParallelCommandGroup(new DriveDist(0, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT).withTimeout(2.15), new flipArmParallel()),
      // new DriveDist(0, PidConstants.DRIVE_SPEED - 0.3, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(0.37),
      RobotContainer.m_Arm.setPosition(1),
      //Drive cube 3
      new ParallelCommandGroup(new DriveDist(0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT).withTimeout(1.53), new flipArmParallel()),
      //Turn to cube 3
      new DriveDist(-115, -PidConstants.TURN_SPEEDRIGHT, PidConstants.kp_TURNRIGHT, PidConstants.ki_TURNRIGHT, PidConstants.kd_TURNRIGHT).withTimeout(1.05),
      //Drive past charge station 
      new DriveDist(-100, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT).withTimeout(1.05),
      //Drive to grid
      new ParallelCommandGroup(new DriveDist(0, PidConstants.TURN_SPEEDRIGHT, PidConstants.kp_DRIVERIGHT, PidConstants.ki_TURNRIGHT, PidConstants.kd_DRIVERIGHT).withTimeout(1.3), new flipArmParallel()),
      // new DriveDist(5, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_ADJUST, PidConstants.kd_DRIVE).withTimeout(0.4),
      RobotContainer.m_Arm.setPosition(3)

    );
  }
}
