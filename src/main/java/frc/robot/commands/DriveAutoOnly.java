// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PidConstants;
import frc.robot.commands.AutoCommands.DriveDistPid;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAutoOnly extends SequentialCommandGroup {
  /** Creates a new DriveAutoOnly. */
  public DriveAutoOnly() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      RobotContainer.m_Drivetrain.resetEncoders().andThen(new DriveDist(-150, 0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT)),
      RobotContainer.m_Drivetrain.resetEncoders().andThen(new DriveDist(150, 0, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT)),
      RobotContainer.m_Drivetrain.resetEncoders().andThen(new DriveDist(100, 0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT)),
      new TurnAngle(-55).withTimeout(1.5),
      RobotContainer.m_Drivetrain.resetGyro().andThen(RobotContainer.m_Drivetrain.resetEncoders().andThen(new DriveDist(50, 55, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT)),
      RobotContainer.m_Drivetrain.resetEncoders().andThen(new DriveDist(50, 0, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT))),
      RobotContainer.m_Drivetrain.resetEncoders().andThen(new DriveDist(100, 0, PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT)),
      new TurnAngle(-55).withTimeout(1.5),
      RobotContainer.m_Drivetrain.resetEncoders().andThen(new DriveDist(50, 0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT)),
      RobotContainer.m_Drivetrain.resetEncoders().andThen(new DriveDist(-50, 0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVERIGHT, PidConstants.ki_DRIVERIGHT, PidConstants.kd_DRIVERIGHT))



    );
  } 
}   
