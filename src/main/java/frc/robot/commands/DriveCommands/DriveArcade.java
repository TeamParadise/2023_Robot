// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import frc.robot.RobotContainer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveArcade extends CommandBase {
  public DriveArcade() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    RobotContainer.m_Drivetrain.setBrakeMode();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double rotateSpeed = -RobotContainer.driverController.getLeftY();
    double moveSpeed = RobotContainer.driverController.getRightX();
    
    MathUtil.applyDeadband(moveSpeed, 0.1);
    MathUtil.applyDeadband(rotateSpeed, 0.1);

    RobotContainer.m_Drivetrain.arcadeDrive(moveSpeed, rotateSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Drivetrain.arcadeDrive(0, 0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }
}