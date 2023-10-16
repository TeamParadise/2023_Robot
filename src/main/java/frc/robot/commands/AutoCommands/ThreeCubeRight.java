// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
      
      //Drive to Cube 2
      new ParallelCommandGroup(new DriveDist(0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(2.5), new flipArmParallel()), //try to see if this will flip the arm and drive at the same time. if not delete this line and uncomment below.
      
      new WaitCommand(0.75),
      
      //Drive to Grid 2
      new ParallelCommandGroup(new DriveDist(0, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(2.3), new flipArmParallel()),
      
      //Shoot cube 2
      new WaitCommand(1),
      new DriveDist(0, PidConstants.DRIVE_SPEED - 0.2, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT).withTimeout(0.45),
      RobotContainer.m_Arm.setPosition(2),
      new ParallelCommandGroup(new DriveDist(0, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT).withTimeout(2.15), new flipArmParallel()),
      RobotContainer.m_Arm.setPosition(1),
      
      //Drive past charge station 3
      new ParallelCommandGroup(new DriveDist(0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(2.03), new flipArmParallel()),

      new WaitCommand(0.5),

      //Turn to cube 3
      new DriveDist(45, 0, PidConstants.kp_TURN, PidConstants.ki_TURN, PidConstants.kd_TURN).withTimeout(2),
      
      new WaitCommand(0.25),

      //Drive to cube 3
      new DriveDist(45, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(0.95),
      
      new WaitCommand(0.5),

      //Drive past Station
      new DriveDist(45, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(0.95),

      new WaitCommand(0.5),

      //Turn to grid
      new DriveDist(0, 0, PidConstants.kp_TURN, PidConstants.ki_TURN, PidConstants.kd_TURN).withTimeout(2),

      new WaitCommand(0.25),

      //Drive to grid
      new ParallelCommandGroup(new DriveDist(0, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE).withTimeout(2.03), new flipArmParallel()),

      //Shoot low
      RobotContainer.m_Arm.setPosition(3)
    );
  }
}
