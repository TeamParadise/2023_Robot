// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveArcade extends CommandBase {

  public double pushSpeed;

  public DriveArcade() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    pushSpeed = 0.9;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if(RobotContainer.driverController.getLeftTriggerAxis() > 0.6){
      pushSpeed = 1;
    }else{
      pushSpeed = 0.9;
    }

    double rotateSpeed = -RobotContainer.driverController.getLeftY() * pushSpeed;
    double moveSpeed = RobotContainer.driverController.getRightX() * 0.7;

    SmartDashboard.putNumber("Move Speed", rotateSpeed);
    SmartDashboard.putNumber("Turn Speed", moveSpeed);
    
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