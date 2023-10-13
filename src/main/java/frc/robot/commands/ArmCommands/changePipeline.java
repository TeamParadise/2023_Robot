// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class changePipeline extends CommandBase {
  public changePipeline() {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(RobotContainer.m_Vision);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    RobotContainer.m_Vision.changePipeline();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }
}