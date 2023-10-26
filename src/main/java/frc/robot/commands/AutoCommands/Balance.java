// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;

public class Balance extends CommandBase {
  /** Creates a new Balance. */

  PIDController tiltController;
  public Balance() {
    addRequirements(RobotContainer.m_Drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tiltController = new PIDController(0.1, 0, 0.01);
    tiltController.setSetpoint(0);
    tiltController.setTolerance(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = tiltController.calculate(RobotContainer.m_Drivetrain.getRobotPitch());
    double rightSpeed = leftSpeed * -1;

    RobotContainer.m_Drivetrain.tankDrive(leftSpeed, rightSpeed);
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
